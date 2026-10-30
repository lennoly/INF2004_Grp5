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
#include "packed.h"

#define ELEMENTS_PER_SLOT (10u) /* 9 elements + 1 gap        */
#define MIN_CHARS         (3u)  /* start + data + stop        */
#define MIN_ELEMENTS      ((MIN_CHARS * ELEMENTS_PER_SLOT) - 1u)
#define MIN_OUT_SIZE      (2u) /* one character + terminator */
#define BARS_PER_CHAR     (5u) /* even elements                */
#define SPACES_PER_CHAR   (4u) /* odd elements                 */
#define NARROW_LAST       (2u) /* sorted index: widest narrow  */
#define WIDE_FIRST        (3u) /* sorted index: narrowest wide */
#define TAIL_ELEMENTS     ((2u * ELEMENTS_PER_SLOT) - 1u) /* data+gap+stop */
#define PAIR_ELEMENTS     (2u) /* a bar and a space: bars stay on even indexes */
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

/* Letter per command.  None at boot: the instructor gives the four
   letters just before the demo and the PC sends them with map=, so no
   letter is built in.  Change Requirement 1: one 6-bit field per command
   (Code 39 index + 1, 0 = not set), NAV_LEFT in the lowest bits.  Only the
   telemetry task writes it (map=), with one store; the barcode task reads
   the whole word once per lookup, so it never sees half a change. */
static volatile uint32_t g_map = 0u;

#define MAP_FIELD_BITS (6u)
#define MAP_FIELD_MASK ((1u << MAP_FIELD_BITS) - 1u)
#define MAP_NOT_SET    (0u)
#define MAP_COMMANDS   ((uint32_t) NAV_UTURN) /* NAV_LEFT..NAV_UTURN */
#define MAP_UNSET_CHAR ('-')

static uint32_t map_shift(barcode_decode_nav_t command);
static uint32_t map_field(uint32_t map, barcode_decode_nav_t command);
static char     map_char(uint32_t map, barcode_decode_nav_t command);

static char    lookup(uint32_t pattern);
static void    sort_widths(uint32_t const * p_widths, uint32_t first,
                           uint32_t count, uint32_t * p_sorted);
static bool    wide_split(uint32_t const * p_sorted, uint32_t * p_threshold);
static char    decode_char(uint32_t const * p_widths);
static int32_t decode_direction(uint32_t const * p_widths, uint32_t count,
                                char * p_out, uint32_t out_size);
static int32_t decode_tail(uint32_t const * p_widths, char * p_out);

/*!
 * @brief Decode a Code 39 frame "*DATA*" read in either direction; a
 *        frame missing its first elements falls back to its last 19
 *        (one data character and the far '*', see decode_tail()); failing
 *        that, any 19 starting on a bar, for a frame broken at both ends
 *        or with stray elements after the '*'.
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
    uint32_t pos       = 0u;
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

    if ((result < 0) && b_args_ok && (count >= TAIL_ELEMENTS)
        && (count <= BARCODE_MAX_ELEMENTS))
    {
        result = decode_tail(&p_widths[count - TAIL_ELEMENTS], p_out);

        /* decode_tail() needs a whole character next to a '*', so a window
           cannot invent a letter.  Each is also tried read backwards: "*X"
           is "X*" crossed the other way round. */
        for (idx = 0u; (result < 0) && ((idx + TAIL_ELEMENTS) <= count);
             idx += PAIR_ELEMENTS)
        {
            result = decode_tail(&p_widths[idx], p_out);

            if (result < 0)
            {
                for (pos = 0u; pos < TAIL_ELEMENTS; pos++)
                {
                    reversed[pos] = p_widths[idx + TAIL_ELEMENTS - 1u - pos];
                }

                result = decode_tail(reversed, p_out);
            }
        }
    }

    return (result);
}

/*!
 * @brief Letter to command, as set by map= (no letter gives a command
 *        until then).
 *
 * @param[in] symbol First decoded data character.
 *
 * @return The navigation command, or NAV_NONE.
 */
barcode_decode_nav_t
barcode_decode_to_command (char symbol)
{
    barcode_decode_nav_t command = NAV_NONE;
    uint32_t             map     = g_map; /* one read: a consistent map */
    uint32_t             index   = packed_symbol_index(symbol);
    uint32_t             idx     = 0u;

    /* Lowest command wins if two share a letter (then the car will not
       start: barcode_decode_map_ready()). */
    for (idx = MAP_COMMANDS; (idx > 0u) && (index < PACKED_SYMBOL_NONE);
         idx--)
    {
        /* Cast: idx is NAV_LEFT..NAV_UTURN. */
        if ((index + 1u) == map_field(map, (barcode_decode_nav_t) idx))
        {
            command = (barcode_decode_nav_t) idx;
        }
    }

    return (command);
}

/*!
 * @brief Set the letter that gives a command (demo letters, map=).  Call
 *        from one task only (telemetry).
 *
 * @param[in] command NAV_LEFT..NAV_UTURN; anything else is ignored.
 * @param[in] symbol  Code 39 data character; any other value (e.g. NUL)
 *                    clears the command's letter.
 */
void
barcode_decode_set_symbol (barcode_decode_nav_t command, char symbol)
{
    uint32_t shift = 0u;
    uint32_t index = packed_symbol_index(symbol);
    uint32_t field = (index < PACKED_SYMBOL_NONE) ? (index + 1u) : MAP_NOT_SET;

    if ((command > NAV_NONE) && (command <= NAV_UTURN))
    {
        shift = map_shift(command);

        /* Single writer: this read-modify-write cannot lose an update, and
           the result is published with one store. */
        g_map = (g_map & ~(MAP_FIELD_MASK << shift)) | (field << shift);
    }
}

/*!
 * @brief Letter that gives a command now.
 *
 * @param[in] command NAV_LEFT..NAV_UTURN.
 *
 * @return The letter, '-' if none is set, or '?' for another value.
 */
char
barcode_decode_symbol (barcode_decode_nav_t command)
{
    return (((command > NAV_NONE) && (command <= NAV_UTURN))
                ? map_char(g_map, command)
                : '?');
}

/*!
 * @brief Whether all four commands have a letter, each a different one:
 *        a run must not start before the PC has sent the demo letters.
 *
 * @return true if the map is complete.
 */
bool
barcode_decode_map_ready (void)
{
    uint32_t map     = g_map;
    uint32_t idx     = 0u;
    uint32_t other   = 0u;
    uint32_t field   = MAP_NOT_SET;
    bool     b_ready = true;

    /* Cast: NAV_LEFT is a small non-negative enumeration constant. */
    for (idx = (uint32_t) NAV_LEFT; idx <= MAP_COMMANDS; idx++)
    {
        /* Cast: idx is NAV_LEFT..NAV_UTURN. */
        field   = map_field(map, (barcode_decode_nav_t) idx);
        b_ready = b_ready && (MAP_NOT_SET != field);

        for (other = idx + 1u; other <= MAP_COMMANDS; other++)
        {
            /* Cast: as above. */
            b_ready =
                b_ready
                && (field != map_field(map, (barcode_decode_nav_t) other));
        }
    }

    return (b_ready);
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
 * @brief Bit position of a command's field in g_map.
 *
 * @param[in] command NAV_LEFT..NAV_UTURN.
 *
 * @return The shift.
 */
static uint32_t
map_shift (barcode_decode_nav_t command)
{
    /* Cast: the command is NAV_LEFT..NAV_UTURN, so this is 0..18. */
    return (((uint32_t) command - 1u) * MAP_FIELD_BITS);
}

/*!
 * @brief One command's field in a copy of the map.
 *
 * @param[in] map     Copy of g_map.
 * @param[in] command NAV_LEFT..NAV_UTURN.
 *
 * @return Code 39 index + 1, or MAP_NOT_SET.
 */
static uint32_t
map_field (uint32_t map, barcode_decode_nav_t command)
{
    return ((map >> map_shift(command)) & MAP_FIELD_MASK);
}

/*!
 * @brief Letter of one command in a copy of the map.
 *
 * @param[in] map     Copy of g_map.
 * @param[in] command NAV_LEFT..NAV_UTURN.
 *
 * @return The letter, or '-' if none is set.
 */
static char
map_char (uint32_t map, barcode_decode_nav_t command)
{
    uint32_t field = map_field(map, command);

    return ((MAP_NOT_SET == field) ? MAP_UNSET_CHAR
                                   : packed_symbol_char(field - 1u));
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
 * @brief Copy every other width of one character and sort them
 *        (insertion sort).
 *
 * @param[in]  p_widths The 9 element widths.
 * @param[in]  first    0 for the bars, 1 for the spaces.
 * @param[in]  count    Number of widths taken (5 bars or 4 spaces).
 * @param[out] p_sorted The taken widths in ascending order.
 */
static void
sort_widths (uint32_t const * p_widths, uint32_t first, uint32_t count,
             uint32_t * p_sorted)
{
    uint32_t idx   = 0u;
    uint32_t pos   = 0u;
    uint32_t value = 0u;

    for (idx = 0u; idx < count; idx++)
    {
        p_sorted[idx] = p_widths[first + (2u * idx)];
    }

    for (idx = 1u; idx < count; idx++)
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
 * @brief Wide / narrow split of sorted widths.  Index 3 is the narrowest
 *        wide in both groups: 2 of 5 bars and 1 of 4 spaces are wide.
 *
 * @param[in]  p_sorted    Ascending widths (5 bars or 4 spaces).
 * @param[out] p_threshold Midway between the widest narrow and the
 *                         narrowest wide.
 *
 * @return true if the contrast reaches BARCODE_MIN_WIDE_RATIO.
 */
static bool
wide_split (uint32_t const * p_sorted, uint32_t * p_threshold)
{
    *p_threshold = (p_sorted[NARROW_LAST] + p_sorted[WIDE_FIRST]) / 2u;

    /* Casts: widths are timer ticks far below 2^24, so float32_t holds
       them exactly. */
    return ((float32_t) p_sorted[WIDE_FIRST]
            >= (BARCODE_MIN_WIDE_RATIO * (float32_t) p_sorted[NARROW_LAST]));
}

/*!
 * @brief Classify one character: its 2 widest bars and its widest space
 *        are wide.  Bars and spaces are split separately because the
 *        sensor reads bars thinner and spaces wider than printed (S3-U03);
 *        $ / + % (no wide bar) are not read.
 *
 * @param[in] p_widths The character's 9 element widths.
 *
 * @return The symbol, or NUL if the wide / narrow contrast is too small
 *         or the pattern is not a Code 39 character.
 */
static char
decode_char (uint32_t const * p_widths)
{
    uint32_t bars[BARS_PER_CHAR];
    uint32_t spaces[SPACES_PER_CHAR];
    uint32_t idx       = 0u;
    uint32_t bar_cut   = 0u;
    uint32_t space_cut = 0u;
    uint32_t threshold = 0u;
    uint32_t pattern   = 0u;
    char     symbol    = TERMINATOR;

    sort_widths(p_widths, 0u, BARS_PER_CHAR, bars);
    sort_widths(p_widths, 1u, SPACES_PER_CHAR, spaces);

    if ((wide_split(bars, &bar_cut)) && (wide_split(spaces, &space_cut)))
    {
        for (idx = 0u; idx < BARCODE_ELEMENTS_PER_CHAR; idx++)
        {
            threshold = (0u == (idx % 2u)) ? bar_cut : space_cut;
            pattern   = (pattern << 1)
                        | ((p_widths[idx] > threshold) ? 1u : 0u);
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

/*!
 * @brief Decode a frame that lost its first elements: the car misses the
 *        first narrow bars when line following (S3-U03, 24 Oct), so only
 *        the last 19 elements, one data character, the gap and the far
 *        '*', are used.  Forwards they read "X*"; crossed the other way
 *        round they reversed read "*X".
 *
 * @param[in]  p_widths The frame's last TAIL_ELEMENTS widths.
 * @param[out] p_out    The data character, NUL-terminated (2 bytes).
 *
 * @return 1, or BARCODE_ERR_FRAME if neither direction has a '*' and a
 *         data character.
 */
static int32_t
decode_tail (uint32_t const * p_widths, char * p_out)
{
    uint32_t reversed[TAIL_ELEMENTS];
    uint32_t idx    = 0u;
    int32_t  status = BARCODE_ERR_FRAME;
    char     first  = decode_char(p_widths);
    char     second = decode_char(&p_widths[ELEMENTS_PER_SLOT]);

    if ((START_STOP == second) && (TERMINATOR != first)
        && (START_STOP != first))
    {
        p_out[0] = first;
        status   = 1;
    }
    else
    {
        for (idx = 0u; idx < TAIL_ELEMENTS; idx++)
        {
            reversed[idx] = p_widths[TAIL_ELEMENTS - 1u - idx];
        }

        first  = decode_char(reversed);
        second = decode_char(&reversed[ELEMENTS_PER_SLOT]);

        if ((START_STOP == first) && (TERMINATOR != second)
            && (START_STOP != second))
        {
            p_out[0] = second;
            status   = 1;
        }
    }

    if (1 == status)
    {
        p_out[1] = TERMINATOR;
    }

    return (status);
}

/*** end of file ***/
