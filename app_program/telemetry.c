/** @file telemetry.c
 *
 * @brief Buddy 1 - telemetry task (see telemetry.h).
 *
 * All values are integers in documented units (tm_sprintf() has no
 * floating-point conversion): mm, mm/s, percent, permille (IR) and tenths
 * of a degree (angles).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "car_math.h"
#include "motion.h"
#include "ir_sensor.h"
#include "barcode.h"
#include "imu.h"
#include "obstacle.h"
#include "vehicle.h"
#include "mqtt_bridge.h"
#include "command.h"
#include "console_in.h"
#include "line_follow.h"
#include "telemetry.h"

#define TASK_PRIORITY      (10)
#define TASK_STACK         (3072)
#define MAX_BYTES_PER_POLL (64u) /* bound the console work per tick     */
#define GAINS_REPORT_TICKS (3u)  /* 60 ms: the vehicle applies first    */
#define TENTHS             (10.0f)
#define PERMILLE           (1000.0f)
#define MS_PER_S           (1000u)
#define NO_STATE_SEEN      (0xFFFFFFFFu)
#define GAIN_TEXTS         (8u) /* 3 PID + 2 feed-forward + 3 line     */
#define LINE_GAINS         (3u)
#define TERMINATOR         ('\0')

/* JSON output buffer; used only by the telemetry task. */
static char g_buf[MQTT_BRIDGE_PAYLOAD_MAX];

/* Cast: tm_sprintf() writes the kernel's UB bytes; the buffer only ever
   holds ASCII text, so it is equally valid as a char string. */
static UB * const gp_out = (UB *) g_buf;

/* Set by telemetry_init() (init task) and by commands, read by the
   telemetry task: volatile (Rule 1.8.c). */
static volatile uint32_t g_period_ms = 0u;    /* telemetry period, 0 = off */
static volatile bool     gb_events   = false; /* mission build: events  */

/* Telemetry-task private state. */
static uint32_t       g_hb_seq    = 0u;
static uint32_t       g_gains_due = 1u; /* ticks until the gains report */
static command_line_t g_line;           /* serial console line assembler */

/* Change detectors for events (telemetry task only). */
static uint32_t g_seen_state   = NO_STATE_SEEN;
static uint32_t g_seen_barcode = 0u;
static uint32_t g_seen_humps   = 0u;
static uint32_t g_seen_scans   = 0u;
static uint32_t g_seen_impacts = 0u;

/* Names indexed by mqtt_bridge_link_t. */
static char const * const g_link_names[] = {
    "DISABLED", "WAIT_NET", "CONNECTING", "CONNECTED", "BACKOFF"};

static void         telemetry_task(INT stacd, void * p_exinf);
static void         handle_commands(void);
static void         handle_command_text(char const * p_text);
static bool         execute(command_t const * p_cmd);
static void         json_safe(char * p_dst, char const * p_src, uint32_t size);
static void         emit(mqtt_bridge_topic_t topic, bool b_console);
static void         build_telemetry(void);
static void         send_heartbeat(void);
static void         check_events(void);
static void         emit_obstacle(vehicle_status_t const * p_veh);
static void         emit_gains(void);
static void         emit_help(void);
static char const * line_name(line_follow_state_t line_state);
static char const * link_name(mqtt_bridge_link_t link);
static INT          to_int(float32_t value);
static INT          tenths(float32_t value);
static INT          permille(float32_t value);
static INT          as_int(int32_t value);
static UINT         as_uint(uint32_t value);

/*!
 * @brief Choose the output streams and start the telemetry task.
 *
 * @param[in] b_stream true for the mission build (telemetry, heartbeat and
 *                     events); false for test builds (commands only).
 *
 * @return E_OK or a kernel error code.
 */
int32_t
telemetry_init (bool b_stream)
{
    T_CTSK  ctsk   = {0};
    ID      h_task = 0;
    int32_t ercd   = E_OK;

    gb_events = b_stream;
#if defined(TM_WIFI_MQTT) && TM_WIFI_MQTT
    mqtt_bridge_set_link(LINK_WAIT_NETWORK);
    g_period_ms = b_stream ? TELEMETRY_PERIOD_MS : 0u;
#else
    g_period_ms = b_stream ? TELEMETRY_CONSOLE_PERIOD_MS : 0u;
#endif
    ctsk.tskatr = TA_HLNG | TA_RNG3;

    /* Cast: the kernel stores every task entry as the generic FP type and
       calls a TA_HLNG task as (INT stacd, void * exinf), matching
       telemetry_task(). */
    ctsk.task    = (FP) telemetry_task;
    ctsk.itskpri = TASK_PRIORITY;
    ctsk.stksz   = TASK_STACK;
    h_task       = tk_cre_tsk(&ctsk);
    ercd         = (h_task >= E_OK) ? tk_sta_tsk(h_task, 0) : h_task;

    return (ercd);
}

/*!
 * @brief 20 ms loop: commands every tick (so "stop" reacts within 20 ms),
 *        events every 100 ms, telemetry every g_period_ms and the
 *        heartbeat at 1 Hz.
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
telemetry_task (INT stacd, void * p_exinf)
{
    SYSTIM              tim        = {0, 0u};
    uint32_t            now        = 0u;
    uint32_t            last_tel   = 0u;
    uint32_t            last_hb    = 0u;
    uint32_t            last_event = 0u;
    mqtt_bridge_stats_t link_stats;

    (void) stacd;
    (void) p_exinf;
    command_line_init(&g_line);
    console_in_init();

    for (;;)
    {
        /* The kernel's millisecond clock; unsigned differences are safe
           across its wrap-around. */
        (void) tk_get_tim(&tim);
        now = tim.lo;
        handle_commands();

        /* Report the gains a few ticks after a change, once the owning
           tasks have applied them. */
        if (g_gains_due > 0u)
        {
            g_gains_due--;

            if (0u == g_gains_due)
            {
                emit_gains();
            }
        }

        if (gb_events && ((now - last_event) >= EVENT_CHECK_MS))
        {
            last_event = now;
            check_events();
        }

        if ((0u != g_period_ms) && ((now - last_tel) >= g_period_ms))
        {
            last_tel = now;
            build_telemetry();
            emit(TOPIC_TELEMETRY, false);
        }

        mqtt_bridge_get_stats(&link_stats);

        if ((LINK_DISABLED != link_stats.link)
            && ((now - last_hb) >= HEARTBEAT_PERIOD_MS))
        {
            last_hb = now;
            send_heartbeat();
        }

        (void) tk_dly_tsk(COMMAND_POLL_MS);
    }
}

/*!
 * @brief Commands from the serial console and from MQTT.
 */
static void
handle_commands (void)
{
    char     line[COMMAND_LINE_MAX];
    uint8_t  rx_byte = 0u;
    uint32_t count   = 0u;

    while ((count < MAX_BYTES_PER_POLL) && (console_in_getc(&rx_byte)))
    {
        count++;

        if (command_line_feed(&g_line, rx_byte, line, sizeof(line)))
        {
            handle_command_text(line);
        }
    }

    while (mqtt_bridge_get_command(line, sizeof(line)))
    {
        handle_command_text(line);
    }
}

/*!
 * @brief Parse, execute and acknowledge one command line.
 *
 * @param[in] p_text NUL-terminated command line.
 */
static void
handle_command_text (char const * p_text)
{
    command_t cmd;
    char      safe[COMMAND_LINE_MAX];
    bool      b_ok = false;

    json_safe(safe, p_text, sizeof(safe));

    /* Casts in this function: tm_sprintf() takes the kernel's UB string
       type; the formats are ASCII. */
    if (command_parse(p_text, &cmd))
    {
        b_ok = execute(&cmd);
        (void) tm_sprintf(gp_out,
                          (UB const *) "{\"type\":\"cmd\",\"cmd\":\"%s\","
                                       "\"ok\":%d%s}",
                          safe, b_ok ? 1 : 0,
                          b_ok ? ""
                               : ",\"err\":\"not available in this build\"");
    }
    else
    {
        (void) tm_sprintf(gp_out,
                          (UB const *) "{\"type\":\"cmd\",\"cmd\":\"%s\","
                                       "\"ok\":0,\"err\":\"%s\"}",
                          safe, command_error());
    }

    emit(TOPIC_EVENT, true);
}

/*!
 * @brief Carry out one parsed command.
 *
 * @param[in] p_cmd Valid command.
 *
 * @return false if the command had no effect in this build.
 */
static bool
execute (command_t const * p_cmd)
{
    bool b_ok = true;

    switch (p_cmd->kind)
    {
        case COMMAND_START:
            b_ok = vehicle_command(VCMD_START, 0);
        break;

        case COMMAND_STOP:
            /* Test builds have no vehicle task: stop the motors directly. */
            if (!vehicle_command(VCMD_STOP, 0))
            {
                motion_stop();
            }
        break;

        case COMMAND_CALIBRATE:
            b_ok = vehicle_command(VCMD_CALIBRATE, 0);
        break;

        case COMMAND_SPEED:
            b_ok =
                vehicle_command(VCMD_SET_SPEED, car_math_round(p_cmd->arg[0]));
            g_gains_due = GAINS_REPORT_TICKS;
        break;

        case COMMAND_PID:
            motion_set_speed_gains(p_cmd->arg[0], p_cmd->arg[1], p_cmd->arg[2]);
            g_gains_due = GAINS_REPORT_TICKS;
        break;

        case COMMAND_FF:
            motion_set_feedforward(p_cmd->arg[0], p_cmd->arg[1]);
            g_gains_due = GAINS_REPORT_TICKS;
        break;

        case COMMAND_LINE:
            /* Without a vehicle task (test build) the gains are stored
               for the next line_follow_init(). */
            if (!vehicle_set_line_gains(p_cmd->arg[0], p_cmd->arg[1],
                                        p_cmd->arg[2]))
            {
                line_follow_set_gains(NULL, p_cmd->arg[0], p_cmd->arg[1],
                                      p_cmd->arg[2]);
            }

            g_gains_due = GAINS_REPORT_TICKS;
        break;

        case COMMAND_RATE:
            /* Cast: command_parse() only accepts 0 or 50..5000 ms. */
            g_period_ms = (uint32_t) car_math_round(p_cmd->arg[0]);
            g_gains_due = GAINS_REPORT_TICKS;
        break;

        case COMMAND_GAINS:
            g_gains_due = GAINS_REPORT_TICKS;
        break;

        case COMMAND_HELP:
            emit_help();
        break;

        default:
            b_ok = false;
        break;
    }

    return (b_ok);
}

/*!
 * @brief Copy text into a JSON string safely (double quotes and
 *        backslashes become apostrophes).
 *
 * @param[out] p_dst Destination, NUL-terminated.
 * @param[in]  p_src Source text.
 * @param[in]  size  Size of p_dst in bytes (at least 1).
 */
static void
json_safe (char * p_dst, char const * p_src, uint32_t size)
{
    uint32_t idx = 0u;

    for (idx = 0u; (idx < (size - 1u)) && (TERMINATOR != p_src[idx]); idx++)
    {
        p_dst[idx] =
            (('"' == p_src[idx]) || ('\\' == p_src[idx])) ? '\'' : p_src[idx];
    }

    p_dst[idx] = TERMINATOR;
}

/*!
 * @brief Publish g_buf if MQTT is enabled; echo it to the console when
 *        asked to or when there is no MQTT.
 *
 * @param[in] topic     Topic to publish on.
 * @param[in] b_console true to print it on the console as well.
 */
static void
emit (mqtt_bridge_topic_t topic, bool b_console)
{
    mqtt_bridge_stats_t link_stats;

    mqtt_bridge_get_stats(&link_stats);

    if (LINK_DISABLED != link_stats.link)
    {
        (void) mqtt_bridge_publish(topic, g_buf);
    }

    /* Cast: tm_printf() takes the kernel's UB string type; ASCII. */
    if (b_console || (LINK_DISABLED == link_stats.link))
    {
        (void) tm_printf((UB const *) "%s\n", g_buf);
    }
}

/*!
 * @brief Full snapshot into g_buf (about 350 bytes worst case, which fits
 *        the 512-byte slot).
 */
static void
build_telemetry (void)
{
    vehicle_status_t     veh;
    motion_status_t      motion;
    ir_sensor_snapshot_t ir_snap;
    barcode_stats_t      bc_stats;
    imu_status_t         imu;
    INT                  count = 0;

    vehicle_get_status(&veh);
    motion_get_status(&motion);
    ir_sensor_read(&ir_snap);
    barcode_get_stats(&bc_stats);
    imu_get_status(&imu);

    /* Casts in this function: tm_sprintf() takes the kernel's UB string
       type; the formats are ASCII. */
    count = tm_sprintf(
        gp_out,
        (UB const *) "{\"t\":%u,\"st\":\"%s\",\"spd\":[%d,%d],"
                     "\"tgt\":[%d,%d],\"pwm\":[%d,%d],\"enc\":[%u,%u],"
                     "\"odo\":%d,\"hdg\":%d,",
        as_uint(veh.run_ms), vehicle_state_name(veh.state),
        to_int(motion.speed_l_mm_s), to_int(motion.speed_r_mm_s),
        to_int(motion.target_l_mm_s), to_int(motion.target_r_mm_s),
        to_int(motion.pwm_l_pct), to_int(motion.pwm_r_pct),
        as_uint(motion.ticks_l), as_uint(motion.ticks_r), to_int(motion.odo_mm),
        tenths(motion.heading_deg));
    count += tm_sprintf(
        &gp_out[count],
        (UB const *) "\"ir\":[%d,%d,%d],\"le\":%d,\"ls\":\"%s\","
                     "\"bc\":\"%s\",\"nav\":\"%s\",\"nbc\":%u,",
        permille(ir_snap.norm[IR_LEFT]), permille(ir_snap.norm[IR_RIGHT]),
        permille(ir_snap.norm[IR_BARCODE]), permille(veh.line_error),
        line_name(veh.line_state), bc_stats.last.text,
        barcode_decode_command_name(veh.last_nav), as_uint(veh.barcodes));
    (void) tm_sprintf(
        &gp_out[count],
        (UB const *) "\"pitch\":%d,\"roll\":%d,\"yaw\":%d,\"rate\":%d,"
                     "\"hump\":{\"h\":%d,\"pk\":%d,\"max\":%d,\"n\":%u},"
                     "\"ev\":\"%s\",\"front\":%d,\"act\":\"%s\","
                     "\"nobs\":%u}",
        tenths(imu.pitch_deg), tenths(imu.roll_deg), tenths(imu.heading_deg),
        tenths(imu.enc_rate_dps), to_int(imu.hump.height_mm),
        to_int(imu.hump.peak_mm), to_int(imu.hump.max_peak_mm),
        as_uint(imu.hump.humps), terrain_event_name(imu.event),
        as_int(veh.front_mm), avoidance_action_name(veh.last_action),
        as_uint(veh.obstacles_passed));
}

/*!
 * @brief Publish the heartbeat (uptime, link statistics, health).
 */
static void
send_heartbeat (void)
{
    vehicle_status_t    veh;
    imu_status_t        imu;
    mqtt_bridge_stats_t link_stats;
    SYSTIM              now = {0, 0u};

    vehicle_get_status(&veh);
    imu_get_status(&imu);
    mqtt_bridge_get_stats(&link_stats);
    (void) tk_get_tim(&now);
    g_hb_seq++;

    /* Cast: tm_sprintf() takes the kernel's UB string type; ASCII. */
    (void) tm_sprintf(
        gp_out,
        (UB const *) "{\"up\":%u,\"seq\":%u,\"st\":\"%s\",\"link\":\"%s\","
                     "\"pub\":%u,\"drop\":%u,\"rx\":%u,\"recon\":%u,"
                     "\"imu\":%d,\"i2cerr\":%u}",
        as_uint(now.lo / MS_PER_S), as_uint(g_hb_seq),
        vehicle_state_name(veh.state), link_name(link_stats.link),
        as_uint(link_stats.published), as_uint(link_stats.dropped),
        as_uint(link_stats.commands), as_uint(link_stats.reconnects),
        imu.b_ok ? 1 : 0, as_uint(imu.i2c_errors));

    if (LINK_DISABLED != link_stats.link)
    {
        (void) mqtt_bridge_publish(TOPIC_HEARTBEAT, g_buf);
    }
}

/*!
 * @brief Emit one event per detected change.
 */
static void
check_events (void)
{
    vehicle_status_t veh;
    barcode_stats_t  bc_stats;
    imu_status_t     imu;

    vehicle_get_status(&veh);
    barcode_get_stats(&bc_stats);
    imu_get_status(&imu);

    /* Casts in this function: tm_sprintf() takes the kernel's UB string
       type; the formats are ASCII. */
    if (veh.state_seq != g_seen_state)
    {
        g_seen_state = veh.state_seq;
        (void) tm_sprintf(gp_out,
                          (UB const *) "{\"type\":\"state\",\"st\":\"%s\","
                                       "\"why\":\"%s\"}",
                          vehicle_state_name(veh.state), veh.p_reason);
        emit(TOPIC_EVENT, false);
    }

    if (bc_stats.decoded != g_seen_barcode)
    {
        g_seen_barcode = bc_stats.decoded;
        (void) tm_sprintf(gp_out,
                          (UB const *) "{\"type\":\"barcode\",\"code\":"
                                       "\"%s\",\"cmd\":\"%s\"}",
                          bc_stats.last.text,
                          barcode_decode_command_name(bc_stats.last.cmd));
        emit(TOPIC_EVENT, false);
    }

    if (imu.hump.humps != g_seen_humps)
    {
        g_seen_humps = imu.hump.humps;
        (void) tm_sprintf(gp_out,
                          (UB const *) "{\"type\":\"hump\",\"n\":%u,"
                                       "\"peak_mm\":%d,\"max_mm\":%d}",
                          as_uint(imu.hump.humps), to_int(imu.hump.peak_mm),
                          to_int(imu.hump.max_peak_mm));
        emit(TOPIC_EVENT, true);
    }

    emit_obstacle(&veh);

    if (imu.impacts < g_seen_impacts)
    {
        g_seen_impacts = imu.impacts; /* counter cleared for a new run */
    }

    /* Cast: tm_sprintf() takes the kernel's UB string type; ASCII. */
    if (imu.impacts > g_seen_impacts)
    {
        g_seen_impacts = imu.impacts;
        (void) tm_sprintf(gp_out, (UB const *) "{\"type\":\"impact\",\"n\":%u}",
                          as_uint(imu.impacts));
        emit(TOPIC_EVENT, true);
    }
}

/*!
 * @brief Emit an obstacle event after each new scan.
 *
 * @param[in] p_veh Vehicle status (for the chosen action).
 */
static void
emit_obstacle (vehicle_status_t const * p_veh)
{
    avoidance_profile_t prof;
    uint32_t            scans = 0u;

    obstacle_get_last(&prof, &scans);

    /* Cast: tm_sprintf() takes the kernel's UB string type; ASCII. */
    if (scans != g_seen_scans)
    {
        g_seen_scans = scans;
        (void) tm_sprintf(
            gp_out,
            (UB const *) "{\"type\":\"obstacle\",\"found\":%d,\"dist\":%d,"
                         "\"ang\":%d,\"width\":%d,\"centre\":%d,\"cl\":%d,"
                         "\"cr\":%d,\"act\":\"%s\"}",
            prof.b_found ? 1 : 0, as_int(prof.closest_mm),
            as_int(prof.closest_angle), to_int(prof.width_mm),
            to_int(prof.centre_mm), as_int(prof.clear_left_mm),
            as_int(prof.clear_right_mm),
            avoidance_action_name(p_veh->last_action));
        emit(TOPIC_EVENT, false);
    }
}

/*!
 * @brief Current gains as one event line (plain decimal values).
 */
static void
emit_gains (void)
{
    motion_gains_t   gains;
    vehicle_status_t veh;
    float32_t        line_gains[LINE_GAINS];
    char             text[GAIN_TEXTS][COMMAND_DECIMAL_MAX];

    motion_get_gains(&gains);
    vehicle_get_status(&veh);
    line_follow_get_gains(&line_gains[0], &line_gains[1], &line_gains[2]);
    command_format_decimal(text[0], gains.gain_p);
    command_format_decimal(text[1], gains.gain_i);
    command_format_decimal(text[2], gains.gain_d);
    command_format_decimal(text[3], gains.gain_ff);
    command_format_decimal(text[4], gains.offset_pct);
    command_format_decimal(text[5], line_gains[0]);
    command_format_decimal(text[6], line_gains[1]);
    command_format_decimal(text[7], line_gains[2]);

    /* Cast: tm_sprintf() takes the kernel's UB string type; ASCII. */
    (void) tm_sprintf(gp_out,
                      (UB const *) "{\"type\":\"gains\",\"pid\":[%s,%s,%s],"
                                   "\"ff\":[%s,%s],\"line\":[%s,%s,%s],"
                                   "\"speed\":%d,\"rate\":%u}",
                      text[0], text[1], text[2], text[3], text[4], text[5],
                      text[6], text[7], as_int(veh.cruise_mm_s),
                      as_uint(g_period_ms));
    emit(TOPIC_EVENT, true);
}

/*!
 * @brief Human-readable usage on the console plus a compact event.
 */
static void
emit_help (void)
{
    /* Casts: tm_printf() and tm_sprintf() take the kernel's UB string
       type; the texts are ASCII. */
    (void) tm_printf(
        (UB const *) "# commands: start | stop | calibrate | gains | help\n"
                     "#   speed=<mm/s>        cruise speed (60..300)\n"
                     "#   pid=<kp>,<ki>,<kd>  wheel speed PID\n"
                     "#   ff=<kf>,<offset>    feed-forward + friction "
                     "offset (pct)\n"
                     "#   line=<kp>,<ki>,<kd> line-following PID\n"
                     "#   rate=<ms>           telemetry period, 0 = off\n");
    (void) tm_sprintf(gp_out,
                      (UB const *) "{\"type\":\"help\",\"cmds\":\"start|stop|"
                                   "calibrate|gains|help|speed=|pid=|ff=|"
                                   "line=|rate=\"}");
    emit(TOPIC_EVENT, false);
}

/*!
 * @brief Short name of a line state for the JSON output.
 *
 * @param[in] line_state Line state.
 *
 * @return "ON", "LOST" or "JUNC".
 */
static char const *
line_name (line_follow_state_t line_state)
{
    char const * p_name = "JUNC";

    if (LINE_ON == line_state)
    {
        p_name = "ON";
    }
    else if (LINE_LOST == line_state)
    {
        p_name = "LOST";
    }
    else
    {
        /* LINE_JUNCTION keeps the default. */
    }

    return (p_name);
}

/*!
 * @brief Printable name of an MQTT link state.
 *
 * @param[in] link Link state.
 *
 * @return Upper-case name, or "?" for an out-of-range value.
 */
static char const *
link_name (mqtt_bridge_link_t link)
{
    /* Cast: enumeration constants are non-negative, so the conversion to
       an unsigned index keeps their value; the range is checked below. */
    uint32_t     index  = (uint32_t) link;
    char const * p_name = "?";

    if (index < (sizeof(g_link_names) / sizeof(g_link_names[0])))
    {
        p_name = g_link_names[index];
    }

    return (p_name);
}

/*!
 * @brief Round a measurement to a whole unit for %d.
 *
 * @param[in] value Measurement (well inside the int32_t range).
 *
 * @return Nearest integer (0 if not finite).
 */
static INT
to_int (float32_t value)
{
    return (car_math_round(value));
}

/*!
 * @brief An angle in tenths of a degree for %d.
 *
 * @param[in] value Angle in degrees.
 *
 * @return Nearest tenth of a degree, times 10.
 */
static INT
tenths (float32_t value)
{
    return (car_math_round(value * TENTHS));
}

/*!
 * @brief A 0..1 fraction in permille for %d.
 *
 * @param[in] value Fraction.
 *
 * @return Nearest permille.
 */
static INT
permille (float32_t value)
{
    return (car_math_round(value * PERMILLE));
}

/*!
 * @brief A 32-bit signed value as the INT that %d expects (same width on
 *        this target, so the implicit conversion keeps the value).
 *
 * @param[in] value Value.
 *
 * @return The same value.
 */
static INT
as_int (int32_t value)
{
    return (value);
}

/*!
 * @brief A 32-bit unsigned value as the UINT that %u expects (same width
 *        on this target, so the implicit conversion keeps the value).
 *
 * @param[in] value Value.
 *
 * @return The same value.
 */
static UINT
as_uint (uint32_t value)
{
    return (value);
}

/*** end of file ***/
