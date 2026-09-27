/** @file command.c
 *
 * @brief Buddy 1 - command parser, serial line assembler and fixed-point
 *        formatter (see command.h).
 *
 * NOTE: numbers are parsed here rather than with strtof(), which needs
 * newlib's system calls (_sbrk, _read ...) that the kernel image lacks.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <ctype.h>
#include "car_types.h"
#include "car_math.h"
#include "command.h"

#define MAX_FRACTION_DIGITS (6u)
#define ASCII_PRINTABLE_MIN (0x20u)
#define ASCII_PRINTABLE_MAX (0x7Eu)
#define ASCII_BACKSPACE     (0x08u)
#define ASCII_DELETE        (0x7Fu)
#define ASCII_CR            (0x0Du)
#define ASCII_LF            (0x0Au)
#define DECIMAL_BASE_F      (10.0f)
#define DECIMAL_BASE_U      (10u)
#define DECIMAL_SCALE_F     (1000.0f) /* 3 decimals                      */
#define FRACTION_DIGITS     (3u)
#define MIN_DIGITS          (4u)     /* "0.000" has 4 digits            */
#define DIGITS_MAX          (12u)    /* 2e9 has 10 digits               */
#define FORMAT_LIMIT        (2.0e6f) /* x1000 still fits in int32_t     */
#define TERMINATOR          ('\0')

typedef struct
{
    char const * p_name;
    command_id_t kind;
    uint32_t     n_args; /* required number of arguments */
} command_spec_t;

static command_spec_t const g_specs[] = {{"start", COMMAND_START, 0u},
                                         {"stop", COMMAND_STOP, 0u},
                                         {"calibrate", COMMAND_CALIBRATE, 0u},
                                         {"gains", COMMAND_GAINS, 0u},
                                         {"help", COMMAND_HELP, 0u},
                                         {"speed", COMMAND_SPEED, 1u},
                                         {"pid", COMMAND_PID, 3u},
                                         {"ff", COMMAND_FF, 2u},
                                         {"line", COMMAND_LINE, 3u},
                                         {"rate", COMMAND_RATE, 1u}};

#define SPEC_COUNT (sizeof(g_specs) / sizeof(g_specs[0]))

static char const g_digit_chars[] = "0123456789";

/* Reason for the last failure.  Used only by the task that parses. */
static char const * gp_error = "";

static char const * skip_space(char const * p_text);
static char const * parse_decimal(char const * p_text, float32_t * p_value);
static char const * match_keyword(char const * p_text, char const * p_name);
static char const * find_keyword(char const * p_text, uint32_t * p_index);
static char const * parse_values(char const * p_text, float32_t * p_args,
                                 uint32_t * p_count);
static bool         validate(command_t const * p_cmd);

/*!
 * @brief Parse one command line.
 *
 * @param[in]  p_text NUL-terminated command line.
 * @param[out] p_cmd  Parsed command.
 *
 * @return true on success; on failure p_cmd->kind is COMMAND_INVALID and
 *         command_error() describes the problem.
 */
bool
command_parse (char const * p_text, command_t * p_cmd)
{
    char const * p_cursor = NULL;
    uint32_t     spec_idx = 0u;
    uint32_t     n_values = 0u;
    bool         b_ok     = false;

    p_cmd->kind   = COMMAND_INVALID;
    p_cmd->n_args = 0u;
    gp_error      = "unknown command (try help)";

    /* Step 1: the keyword. */
    p_cursor = find_keyword(skip_space(p_text), &spec_idx);
    b_ok     = (NULL != p_cursor);

    /* Step 2: the optional "=value,value,..." list. */
    if (b_ok)
    {
        p_cursor = parse_values(skip_space(p_cursor), p_cmd->arg, &n_values);
        b_ok     = (NULL != p_cursor);
    }

    /* Step 3: nothing may follow, and the value count must match. */
    if (b_ok && (TERMINATOR != *skip_space(p_cursor)))
    {
        gp_error = "unexpected text after command";
        b_ok     = false;
    }

    if (b_ok && (n_values != g_specs[spec_idx].n_args))
    {
        gp_error = "wrong number of values";
        b_ok     = false;
    }

    /* Step 4: range checks. */
    if (b_ok)
    {
        p_cmd->kind   = g_specs[spec_idx].kind;
        p_cmd->n_args = n_values;
        b_ok          = validate(p_cmd);
    }

    if (b_ok)
    {
        gp_error = "";
    }
    else
    {
        p_cmd->kind = COMMAND_INVALID;
    }

    return (b_ok);
}

/*!
 * @brief Reason for the last command_parse() failure.
 *
 * @return A short message ("" after a success).
 */
char const *
command_error (void)
{
    return (gp_error);
}

/*!
 * @brief Empty a line assembler.
 *
 * @param[out] p_line Line assembler.
 */
void
command_line_init (command_line_t * p_line)
{
    p_line->len        = 0u;
    p_line->b_overflow = false;
}

/*!
 * @brief Feed one received byte to a line assembler.
 *
 * CR or LF ends a line; backspace edits; other control bytes are ignored;
 * over-long lines are discarded.
 *
 * @param[in,out] p_line   Line assembler.
 * @param[in]     rx_byte  Received byte.
 * @param[out]    p_out    Completed line, NUL-terminated.
 * @param[in]     out_size Size of p_out in bytes.
 *
 * @return true when a complete, non-empty line was copied to p_out.
 */
bool
command_line_feed (command_line_t * p_line, uint8_t rx_byte, char * p_out,
                   uint32_t out_size)
{
    uint32_t idx    = 0u;
    bool     b_done = false;

    if ((ASCII_CR == rx_byte) || (ASCII_LF == rx_byte))
    {
        b_done = (!p_line->b_overflow) && (p_line->len > 0u)
                 && (p_line->len < out_size);

        for (idx = 0u; b_done && (idx < p_line->len); idx++)
        {
            p_out[idx] = p_line->buf[idx];
        }

        if (b_done)
        {
            p_out[p_line->len] = TERMINATOR;
        }

        command_line_init(p_line);
    }
    else if ((ASCII_BACKSPACE == rx_byte) || (ASCII_DELETE == rx_byte))
    {
        p_line->len = (p_line->len > 0u) ? (p_line->len - 1u) : 0u;
    }
    else if ((rx_byte >= ASCII_PRINTABLE_MIN)
             && (rx_byte <= ASCII_PRINTABLE_MAX))
    {
        if (p_line->len < (COMMAND_LINE_MAX - 1u))
        {
            /* Cast: printable ASCII (0x20..0x7E) is a valid char. */
            p_line->buf[p_line->len] = (char) rx_byte;
            p_line->len++;
        }
        else
        {
            p_line->b_overflow = true;
        }
    }
    else
    {
        /* Ignore other control bytes (e.g. 0x00 from a UART break). */
    }

    return (b_done);
}

/*!
 * @brief Format a value with exactly 3 decimals, e.g. -0.08 gives
 *        "-0.080" (tm_sprintf() has no floating-point conversion).
 *
 * @param[out] p_out At least COMMAND_DECIMAL_MAX bytes.
 * @param[in]  value Value; limited to +-2e6, and 0 if not finite.
 */
void
command_format_decimal (char * p_out, float32_t value)
{
    char      digits[DIGITS_MAX];
    float32_t limited   = 0.0f;
    int32_t   milli     = 0;
    uint32_t  magnitude = 0u;
    uint32_t  count     = 0u;
    uint32_t  pos       = 0u;

    if (car_math_is_finite(value))
    {
        limited = car_math_clamp(value, -FORMAT_LIMIT, FORMAT_LIMIT);
    }

    milli = car_math_round(limited * DECIMAL_SCALE_F);

    if (milli < 0)
    {
        p_out[pos] = '-';
        pos++;

        /* Cast: |milli| <= 2e9 < 2^31, so -milli is positive and fits. */
        magnitude = (uint32_t) (-milli);
    }
    else
    {
        /* Cast: milli is not negative here. */
        magnitude = (uint32_t) milli;
    }

    /* Collect digits, least significant first; at least MIN_DIGITS so
       that values below 1 print as "0.xxx". */
    do
    {
        digits[count] = g_digit_chars[magnitude % DECIMAL_BASE_U];
        count++;
        magnitude /= DECIMAL_BASE_U;
    } while ((magnitude > 0u) || (count < MIN_DIGITS));

    while (count > 0u)
    {
        count--;
        p_out[pos] = digits[count];
        pos++;

        if (FRACTION_DIGITS == count)
        {
            p_out[pos] = '.';
            pos++;
        }
    }

    p_out[pos] = TERMINATOR;
}

/*!
 * @brief Skip white space.
 *
 * @param[in] p_text Text.
 *
 * @return Pointer to the first character that is not white space.
 */
static char const *
skip_space (char const * p_text)
{
    char const * p_cursor = p_text;

    /* Cast: the ctype functions take the character as unsigned char. */
    while (0 != isspace((unsigned char) *p_cursor))
    {
        p_cursor++;
    }

    return (p_cursor);
}

/*!
 * @brief Parse a decimal number such as "-12.345".
 *
 * @param[in]  p_text  Text starting with the number.
 * @param[out] p_value The number (written only on success).
 *
 * @return Pointer after the number, or NULL if no digits were found.
 */
static char const *
parse_decimal (char const * p_text, float32_t * p_value)
{
    char const * p_cursor = p_text;
    char const * p_end    = NULL;
    float32_t    value    = 0.0f;
    float32_t    scale    = 1.0f;
    float32_t    sign     = 1.0f;
    bool         b_digits = false;
    uint32_t     fraction = 0u;

    if (('-' == *p_cursor) || ('+' == *p_cursor))
    {
        sign = ('-' == *p_cursor) ? -1.0f : 1.0f;
        p_cursor++;
    }

    /* Casts: a digit minus '0' is 0..9, which float32_t holds exactly. */
    while ((*p_cursor >= '0') && (*p_cursor <= '9'))
    {
        value    = (value * DECIMAL_BASE_F) + (float32_t) (*p_cursor - '0');
        b_digits = true;
        p_cursor++;
    }

    if ('.' == *p_cursor)
    {
        p_cursor++;

        /* Digits beyond MAX_FRACTION_DIGITS are read but ignored. */
        while ((*p_cursor >= '0') && (*p_cursor <= '9'))
        {
            /* Cast: a digit minus '0' is 0..9, exact in float32_t. */
            if (fraction < MAX_FRACTION_DIGITS)
            {
                scale /= DECIMAL_BASE_F;
                value += scale * (float32_t) (*p_cursor - '0');
                fraction++;
            }

            b_digits = true;
            p_cursor++;
        }
    }

    if (b_digits)
    {
        *p_value = sign * value;
        p_end    = p_cursor;
    }

    return (p_end);
}

/*!
 * @brief Match a lower-case keyword at the start of the text.
 *
 * @param[in] p_text Text (any case).
 * @param[in] p_name Keyword (lower case).
 *
 * @return Pointer after the keyword, or NULL if the text does not start
 *         with it followed by '=', white space or the end.
 */
static char const *
match_keyword (char const * p_text, char const * p_name)
{
    char const * p_cursor = p_text;
    char const * p_letter = p_name;
    bool         b_match  = true;

    while (b_match && (TERMINATOR != *p_letter))
    {
        /* Cast: the ctype functions take the character as unsigned char. */
        b_match = (tolower((unsigned char) *p_cursor) == *p_letter);

        if (b_match)
        {
            p_cursor++;
            p_letter++;
        }
    }

    /* The keyword must end here: "starter" is not "start".
       Cast: the ctype functions take the character as unsigned char. */
    b_match = b_match
              && ((TERMINATOR == *p_cursor) || ('=' == *p_cursor)
                  || (0 != isspace((unsigned char) *p_cursor)));

    return (b_match ? p_cursor : NULL);
}

/*!
 * @brief Find which command keyword the text starts with.
 *
 * @param[in]  p_text  Text starting with a keyword.
 * @param[out] p_index Index into g_specs of the match.
 *
 * @return Pointer after the keyword, or NULL if none matches.
 */
static char const *
find_keyword (char const * p_text, uint32_t * p_index)
{
    char const * p_after = NULL;
    uint32_t     idx     = 0u;

    for (idx = 0u; (idx < SPEC_COUNT) && (NULL == p_after); idx++)
    {
        p_after  = match_keyword(p_text, g_specs[idx].p_name);
        *p_index = idx;
    }

    return (p_after);
}

/*!
 * @brief Parse an optional "=value,value,..." list.
 *
 * @param[in]  p_text  Text after the keyword (white space skipped).
 * @param[out] p_args  Up to COMMAND_MAX_ARGS values.
 * @param[out] p_count Number of values read.
 *
 * @return Pointer after the list (p_text if there is no '='), or NULL on
 *         error with gp_error set.
 */
static char const *
parse_values (char const * p_text, float32_t * p_args, uint32_t * p_count)
{
    char const * p_cursor = p_text;
    bool         b_more   = ('=' == *p_cursor);

    *p_count = 0u;

    while (b_more)
    {
        /* Step over the '=' or ',' in front of the next value. */
        p_cursor = skip_space(&p_cursor[1]);

        if (*p_count >= COMMAND_MAX_ARGS)
        {
            gp_error = "too many values";
            p_cursor = NULL;
        }
        else
        {
            p_cursor = parse_decimal(p_cursor, &p_args[*p_count]);
        }

        if (NULL != p_cursor)
        {
            (*p_count)++;
            p_cursor = skip_space(p_cursor);
            b_more   = (',' == *p_cursor);
        }
        else if (*p_count < COMMAND_MAX_ARGS)
        {
            gp_error = "bad number";
            b_more   = false;
        }
        else
        {
            b_more = false;
        }
    }

    return (p_cursor);
}

/*!
 * @brief Range checks per command.
 *
 * @param[in] p_cmd Parsed command with the right number of values.
 *
 * @return true if every value is in range; otherwise gp_error is set.
 */
static bool
validate (command_t const * p_cmd)
{
    uint32_t idx     = 0u;
    bool     b_ok    = true;
    int32_t  rate_ms = 0;

    switch (p_cmd->kind)
    {
        case COMMAND_PID:
        case COMMAND_LINE:
            for (idx = 0u; idx < p_cmd->n_args; idx++)
            {
                b_ok = b_ok && (car_math_is_finite(p_cmd->arg[idx]))
                       && (p_cmd->arg[idx] >= 0.0f)
                       && (p_cmd->arg[idx] <= COMMAND_GAIN_MAX);
            }

            gp_error = b_ok ? gp_error : "gain out of range 0..1000";
        break;

        case COMMAND_FF:
            b_ok = (p_cmd->arg[0] >= 0.0f)
                   && (p_cmd->arg[0] <= COMMAND_GAIN_MAX)
                   && (p_cmd->arg[1] >= 0.0f)
                   && (p_cmd->arg[1] <= COMMAND_OFFSET_MAX);
            gp_error = b_ok ? gp_error : "ff: kf 0..1000, offset 0..100";
        break;

        case COMMAND_RATE:
            /* Integer comparison: floats are never tested for equality
               (Rule 5.4.b.iv). */
            rate_ms = car_math_round(p_cmd->arg[0]);
            b_ok    = (0 == rate_ms)
                   || ((rate_ms >= COMMAND_RATE_MIN_MS)
                       && (rate_ms <= COMMAND_RATE_MAX_MS));
            gp_error = b_ok ? gp_error : "rate: 0 (off) or 50..5000 ms";
        break;

        case COMMAND_SPEED:
            b_ok     = (p_cmd->arg[0] > 0.0f);
            gp_error = b_ok ? gp_error : "speed must be > 0";
        break;

        default:
            /* No values to check. */
        break;
    }

    return (b_ok);
}

/*** end of file ***/
