/** @file command.h
 *
 * @brief Buddy 1 - text command parser shared by the MQTT "cmd" topic and
 *        the USB / UART serial console (pure C, host-testable).
 *
 * Grammar (keywords are case-insensitive, spaces around '=' and ',' are
 * ignored, one command per line).  A command may start with a request id,
 * "#<id> " (1..999999999), which the reply repeats, e.g. "#17 gains":
 *
 *   start | stop | calibrate | gains | help | timing
 *   speed=<mm/s>                 cruise speed for line following, 50..300
 *   pid=<kp>,<ki>,<kd>           wheel speed PID
 *   ff=<kf>,<offset_pct>         speed feed-forward + static-friction offset
 *   line=<kp>,<ki>,<kd>          line-following PID
 *   rate=<ms>                    telemetry period, 0 = off, 50..5000
 *   nav=<1..4>                   as if a barcode was read: 1 left,
 *                                2 right, 3 straight, 4 U-turn (testing
 *                                turns without barcodes; while running)
 *   map=<1..4>,<0..42>           barcode letter for a command (same
 *                                numbers as nav=) as its Code 39 index,
 *                                0-9 then A=10 .. Z=35 (the dashboard
 *                                converts letters); back to car_config.h
 *                                at reset
 *
 * Numbers are plain decimals ("0.08", "-1.5", "12"), without exponents.
 * The tuning commands (speed=, pid=, ff=, line=, map=) are accepted only
 * while the car is IDLE or STOPPED (command_allowed()).
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
#define COMMAND_NAV_MIN     (1) /* NAV_LEFT  */
#define COMMAND_NAV_MAX     (4) /* NAV_UTURN */
#define COMMAND_SYMBOL_MAX  (42) /* last Code 39 index ('%') */
#define COMMAND_DETECT_MIN_MM (100) /* above OBST_TOO_CLOSE_MM        */
#define COMMAND_DETECT_MAX_MM (279) /* below OBST_FLOOR_MM            */

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
    COMMAND_TIMING,
    COMMAND_NAV,
    COMMAND_MAP,
    COMMAND_DETECT,
    COMMAND_INVALID
} command_id_t;

typedef struct
{
    command_id_t kind;
    uint32_t     n_args;
    float32_t    arg[COMMAND_MAX_ARGS];
    uint32_t     req_id; /* "#<id>" prefix, 0 if none */
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
bool         command_allowed(command_id_t kind, bool b_stationary);
bool         command_check(command_t * p_cmd);
uint32_t     command_arg_count(command_id_t kind);
void         command_line_init(command_line_t * p_line);
bool command_line_feed(command_line_t * p_line, uint8_t rx_byte, char * p_out,
                       uint32_t out_size);
void command_format_decimal(char * p_out, float32_t value);

#endif /* COMMAND_H */

/*** end of file ***/
