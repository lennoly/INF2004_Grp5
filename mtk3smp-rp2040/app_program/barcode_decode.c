/** @file barcode_decode.c
 *
 * @brief Buddy 3 - Code 39 decoding (see barcode_decode.h).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "car_types.h"
#include "car_config.h"
#include "barcode_decode.h"

#define ELEMENTS_PER_SLOT (10u) /* 9 elements + 1 gap        */
#define MIN_CHARS         (3u)  /* start + data + stop        */
#define MIN_ELEMENTS      ((MIN_CHARS * ELEMENTS_PER_SLOT) - 1u)
#define MIN_OUT_SIZE      (2u) /* one character + terminator */
#define NARROW_LAST       (5u) /* sorted index: widest narrow */
#define WIDE_FIRST        (6u) /* sorted index: narrowest wide */
#define START_STOP        ('*')
#define DECODE_OK         (0)
#define TERMINATOR        ('\0')

typedef struct
{
    uint32_t pattern; /* 9 bits, MSB = first bar, 1 = wide */
    char     symbol;
} code39_entry_t;

/* Standard Code 39 table (bar, space, bar, ...; 1 = wide). */
static code39_entry_t const g_table[] = {
    {0x034u, '0'}, {0x121u, '1'}, {0x061u, '2'}, {0x160u, '3'}, {0x031u, '4'},
    {0x130u, '5'}, {0x070u, '6'}, {0x025u, '7'}, {0x124u, '8'}, {0x064u, '9'},
    {0x109u, 'A'}, {0x049u, 'B'}, {0x148u, 'C'}, {0x019u, 'D'}, {0x118u, 'E'},
    {0x058u, 'F'}, {0x00Du, 'G'}, {0x10Cu, 'H'}, {0x04Cu, 'I'}, {0x01Cu, 'J'},
    {0x103u, 'K'}, {0x043u, 'L'}, {0x142u, 'M'}, {0x013u, 'N'}, {0x112u, 'O'},
    {0x052u, 'P'}, {0x007u, 'Q'}, {0x106u, 'R'}, {0x046u, 'S'}, {0x016u, 'T'},
    {0x181u, 'U'}, {0x0C1u, 'V'}, {0x1C0u, 'W'}, {0x091u, 'X'}, {0x190u, 'Y'},
    {0x0D0u, 'Z'}, {0x085u, '-'}, {0x184u, '.'}, {0x0C4u, ' '}, {0x094u, '*'},
    {0x0A8u, '$'}, {0x0A2u, '/'}, {0x08Au, '+'}, {0x02Au, '%'}};

#define TABLE_SIZE (sizeof(g_table) / sizeof(g_table[0]))

/* Names indexed by barcode_decode_nav_t. */
static char const * const g_nav_names[] = {"NONE", "LEFT", "RIGHT", "STRAIGHT",
                                           "UTURN"};

static char    lookup(uint32_t pattern);
static void    sort_widths(uint32_t const * p_widths, uint32_t * p_sorted);
static char    decode_char(uint32_t const * p_widths);
static int32_t decode_direction(uint32_t const * p_widths, uint32_t count,
                                char * p_out, uint32_t out_size);

/*!
 * @brief Decode a Code 39 frame "*DATA*" read in either direction.
 *
 * @param[in]  p_widths Element widths, first element a bar.
 * @param[in]  count    Number of widths; trailing extras are ignored.
 * @param[out] p_out    Decoded data characters, NUL-terminated.
 * @param[in]  out_size Size of p_out in bytes (at least 2).
 *
 * @return Number of data characters written, or a BARCODE_ERR_* code.
 */
int32_t
barcode_decode_frame (uint32_t const * p_widths, uint32_t count, char * p_out,
                      uint32_t out_size)
{
    uint32_t reversed[BARCODE_MAX_ELEMENTS];
    uint32_t used      = count;
    uint32_t idx       = 0u;
    int32_t  result    = BARCODE_ERR_LENGTH;
    bool     b_args_ok = false;

    b_args_ok =
        (NULL != p_widths) && (NULL != p_out) && (out_size >= MIN_OUT_SIZE);

    /* Drop trailing noise so the length is 10k - 1. */
    while ((used > 0u) && (0u != ((used + 1u) % ELEMENTS_PER_SLOT)))
    {
        used--;
    }

    if (b_args_ok && (used >= MIN_ELEMENTS) && (used <= BARCODE_MAX_ELEMENTS))
    {
        result = decode_direction(p_widths, used, p_out, out_size);

        if (result < 0)
        {
            /* Not readable forwards: the car may have crossed it the
               other way round. */
            for (idx = 0u; idx < used; idx++)
            {
                reversed[idx] = p_widths[used - 1u - idx];
            }

            result = decode_direction(reversed, used, p_out, out_size);
        }
    }

    return (result);
}

/*!
 * @brief Brief mapping: A = left, B = right, C = straight, D = U-turn.
 *
 * @param[in] symbol First decoded data character.
 *
 * @return The navigation command, or NAV_NONE.
 */
barcode_decode_nav_t
barcode_decode_to_command (char symbol)
{
    barcode_decode_nav_t command = NAV_NONE;

    switch (symbol)
    {
        case 'A':
            command = NAV_LEFT;
        break;

        case 'B':
            command = NAV_RIGHT;
        break;

        case 'C':
            command = NAV_STRAIGHT;
        break;

        case 'D':
            command = NAV_UTURN;
        break;

        default:
            command = NAV_NONE;
        break;
    }

    return (command);
}

/*!
 * @brief Printable name of a navigation command.
 *
 * @param[in] command Navigation command.
 *
 * @return Upper-case name, or "?" for an out-of-range value.
 */
char const *
barcode_decode_command_name (barcode_decode_nav_t command)
{
    /* Cast: enumeration constants are non-negative, so the conversion to
       an unsigned index keeps their value; the range is checked below. */
    uint32_t     index  = (uint32_t) command;
    char const * p_name = "?";

    if (index < (sizeof(g_nav_names) / sizeof(g_nav_names[0])))
    {
        p_name = g_nav_names[index];
    }

    return (p_name);
}

/*!
 * @brief Look up a 9-bit pattern in the Code 39 table.
 *
 * @param[in] pattern Wide / narrow pattern, first element in the MSB.
 *
 * @return The symbol, or NUL if the pattern is not a Code 39 character.
 */
static char
lookup (uint32_t pattern)
{
    char     symbol = TERMINATOR;
    uint32_t idx    = 0u;

    for (idx = 0u; (idx < TABLE_SIZE) && (TERMINATOR == symbol); idx++)
    {
        if (pattern == g_table[idx].pattern)
        {
            symbol = g_table[idx].symbol;
        }
    }

    return (symbol);
}

/*!
 * @brief Copy one character's widths and sort them (insertion sort).
 *
 * @param[in]  p_widths The 9 element widths.
 * @param[out] p_sorted The same widths in ascending order.
 */
static void
sort_widths (uint32_t const * p_widths, uint32_t * p_sorted)
{
    uint32_t idx   = 0u;
    uint32_t pos   = 0u;
    uint32_t value = 0u;

    for (idx = 0u; idx < BARCODE_ELEMENTS_PER_CHAR; idx++)
    {
        p_sorted[idx] = p_widths[idx];
    }

    for (idx = 1u; idx < BARCODE_ELEMENTS_PER_CHAR; idx++)
    {
        value = p_sorted[idx];
        pos   = idx;

        while ((pos > 0u) && (p_sorted[pos - 1u] > value))
        {
            p_sorted[pos] = p_sorted[pos - 1u];
            pos--;
        }

        p_sorted[pos] = value;
    }
}

/*!
 * @brief Classify one character: its 3 widest elements are wide.
 *
 * @param[in] p_widths The character's 9 element widths.
 *
 * @return The symbol, or NUL if the wide / narrow contrast is too small
 *         or the pattern is not a Code 39 character.
 */
static char
decode_char (uint32_t const * p_widths)
{
    uint32_t sorted[BARCODE_ELEMENTS_PER_CHAR];
    uint32_t idx       = 0u;
    uint32_t threshold = 0u;
    uint32_t pattern   = 0u;
    char     symbol    = TERMINATOR;

    sort_widths(p_widths, sorted);

    /* Casts: widths are timer ticks far below 2^24, so float32_t holds
       them exactly. */
    if ((float32_t) sorted[WIDE_FIRST]
        >= (BARCODE_MIN_WIDE_RATIO * (float32_t) sorted[NARROW_LAST]))
    {
        threshold = (sorted[NARROW_LAST] + sorted[WIDE_FIRST]) / 2u;

        for (idx = 0u; idx < BARCODE_ELEMENTS_PER_CHAR; idx++)
        {
            pattern = (pattern << 1) | ((p_widths[idx] > threshold) ? 1u : 0u);
        }

        symbol = lookup(pattern);
    }

    return (symbol);
}

/*!
 * @brief Decode in one direction.
 *
 * @param[in]  p_widths Element widths (10k - 1 of them).
 * @param[in]  count    Number of widths.
 * @param[out] p_out    Decoded data characters, NUL-terminated.
 * @param[in]  out_size Size of p_out in bytes.
 *
 * @return Number of data characters, or a BARCODE_ERR_* code.
 */
static int32_t
decode_direction (uint32_t const * p_widths, uint32_t count, char * p_out,
                  uint32_t out_size)
{
    uint32_t chars      = (count + 1u) / ELEMENTS_PER_SLOT;
    uint32_t char_idx   = 0u;
    uint32_t len        = 0u;
    int32_t  status     = DECODE_OK;
    char     symbol     = TERMINATOR;
    bool     b_is_frame = false;

    for (char_idx = 0u; (char_idx < chars) && (DECODE_OK == status); char_idx++)
    {
        symbol     = decode_char(&p_widths[char_idx * ELEMENTS_PER_SLOT]);
        b_is_frame = (0u == char_idx) || ((chars - 1u) == char_idx);

        if (TERMINATOR == symbol)
        {
            status = BARCODE_ERR_SYMBOL;
        }
        else if (b_is_frame && (START_STOP != symbol))
        {
            status = BARCODE_ERR_FRAME;
        }
        else if (b_is_frame)
        {
            /* Start / stop character: framing, not data. */
        }
        else if (len >= (out_size - 1u))
        {
            status = BARCODE_ERR_LENGTH;
        }
        else
        {
            p_out[len] = symbol;
            len++;
        }
    }

    if (DECODE_OK == status)
    {
        p_out[len] = TERMINATOR;

        /* Cast: len < out_size, a small buffer size, so it fits int32_t. */
        status = (int32_t) len;
    }

    return (status);
}

/*** end of file ***/
