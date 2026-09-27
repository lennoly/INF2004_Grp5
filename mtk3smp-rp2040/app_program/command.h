/** @file command.h
 *
 * @brief Buddy 1 - text command parser shared by the MQTT "cmd" topic and
 *        the USB / UART serial console (pure C, host-testable).
 *
 * Grammar (keywords are case-insensitive, spaces around '=' and ',' are
 * ignored, one command per line):
 *
 *   start | stop | calibrate | gains | help
 *   speed=<mm/s>                 cruise speed for line following
 *   pid=<kp>,<ki>,<kd>           wheel speed PID
 *   ff=<kf>,<offset_pct>         speed feed-forward + static-friction offset
 *   line=<kp>,<ki>,<kd>          line-following PID
 *   rate=<ms>                    telemetry period, 0 = off, 50..5000
 *
 * Numbers are plain decimals ("0.08", "-1.5", "12"), without exponents.
 */

#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"

#define COMMAND_MAX_ARGS    (3u)
#define COMMAND_LINE_MAX    (64u) /* longest accepted command line       */
#define COMMAND_DECIMAL_MAX (16u) /* buffer for command_format_decimal() */

#define COMMAND_GAIN_MAX    (1000.0f)
#define COMMAND_OFFSET_MAX  (100.0f)
#define COMMAND_RATE_MIN_MS (50)
#define COMMAND_RATE_MAX_MS (5000)

typedef enum
{
    COMMAND_START = 0,
    COMMAND_STOP,
    COMMAND_CALIBRATE,
    COMMAND_GAINS,
    COMMAND_HELP,
    COMMAND_SPEED,
    COMMAND_PID,
    COMMAND_FF,
    COMMAND_LINE,
    COMMAND_RATE,
    COMMAND_INVALID
} command_id_t;

typedef struct
{
    command_id_t kind;
    uint32_t     n_args;
    float32_t    arg[COMMAND_MAX_ARGS];
} command_t;

/* Line assembler for byte streams (serial console). */
typedef struct
{
    char     buf[COMMAND_LINE_MAX];
    uint32_t len;
    bool     b_overflow;
} command_line_t;

bool         command_parse(char const * p_text, command_t * p_cmd);
char const * command_error(void);
void         command_line_init(command_line_t * p_line);
bool command_line_feed(command_line_t * p_line, uint8_t rx_byte, char * p_out,
                       uint32_t out_size);
void command_format_decimal(char * p_out, float32_t value);

#endif /* COMMAND_H */

/*** end of file ***/
