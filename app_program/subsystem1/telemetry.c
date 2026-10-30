/** @file telemetry.c
 *
 * @brief Buddy 1 - telemetry task (see telemetry.h).
 *
 * Change Requirement 1: every message is a compact binary frame
 * (frames.h).  On MQTT the frame is the payload; on the serial console it
 * is printed as '@' followed by its bytes in hex, so the dashboard reads
 * both links the same way.  Command replies, gains and timing also get a
 * short human-readable '#' line on the console.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "car_math.h"
#include "motion.h"
#include "encoder.h"
#include "ir_sensor.h"
#include "barcode.h"
#include "imu.h"
#include "obstacle.h"
#include "vehicle.h"
#include "mqtt_bridge.h"
#include "command.h"
#include "console_in.h"
#include "line_follow.h"
#include "timing.h"
#include "fault.h"
#include "packed.h"
#include "frames.h"
#include "telemetry.h"

#define TASK_PRIORITY      (10)
#define TASK_STACK         (3072)
#define MAX_BYTES_PER_POLL (64u) /* bound the console work per tick     */
#define GAINS_REPORT_TICKS (3u)  /* 60 ms: the vehicle applies first    */
#define PERCENT            (100u)
#define MS_PER_S           (1000u)
#define NO_STATE_SEEN      (0xFFFFFFFFu)
#define GAIN_TEXTS         (8u) /* 3 PID + 2 feed-forward + 3 line     */
#define LINE_GAINS         (3u)
#define TERMINATOR         ('\0')
#define HEX_DIGITS         (2u)
#define NIBBLE_BITS        (4u)
#define NIBBLE_MASK        (0xFu)
#define HEX_MAX            ((FRAMES_MAX_BYTES * HEX_DIGITS) + 2u)

/* The WiFi test build also prints the heartbeat on the USB console, so the
   link can be watched there even while MQTT is down. */
#if APP_MODE == APP_MODE_TEST_TELEMETRY
#define HEARTBEAT_ON_CONSOLE (true)
#else
#define HEARTBEAT_ON_CONSOLE (false)
#endif

/* Frame and its hex text; used only by the telemetry task. */
static uint8_t  g_frame[FRAMES_MAX_BYTES];
static uint32_t g_frame_len = 0u;
static char     g_hex[HEX_MAX];

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
/* Peak jerk of the last two event checks (imu_take_peak_jerk()): the
   impact's own jolt can land just before or after the check that sees the
   new count.  Taken only here; events are off in the TEST_IMU / TEST_CRASH
   builds, which read it themselves. */
static float32_t g_jerk_prev_mg = 0.0f;

static char const g_hex_chars[] = "0123456789abcdef";

static void           telemetry_task(INT stacd, void * p_exinf);
static void           handle_commands(void);
static void           reply(command_t const * p_cmd, frames_error_t error,
                            char const * p_text);
static frames_error_t execute(command_t const * p_cmd);
static char const *   error_text(frames_error_t error);
static bool           stationary(void);
static void           emit_timing(void);
static void           emit(mqtt_bridge_topic_t topic, bool b_console);
static void           build_telemetry(void);
static void           send_heartbeat(void);
static void           check_events(void);
static void           emit_obstacle(vehicle_status_t const * p_veh);
static void           emit_gains(void);
static void           emit_help(void);

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
    vehicle_status_t    veh;

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

        /* A WiFi chip restart stalls the cyw43 task (above motion), so it
           is allowed only while the car is not driving. */
        vehicle_get_status(&veh);
        mqtt_bridge_set_restart_ok((VS_IDLE == veh.state)
                                   || (VS_STOPPED == veh.state));
        mqtt_bridge_get_stats(&link_stats);

        /* Every link: without WiFi, emit() prints it on the console
           (uptime, state and IMU health; the link counters stay 0). */
        if ((now - last_hb) >= HEARTBEAT_PERIOD_MS)
        {
            last_hb = now;
            send_heartbeat();
        }

        (void) tk_dly_tsk(COMMAND_POLL_MS);
    }
}

/*!
 * @brief Commands: text lines from the serial console, binary command
 *        frames from MQTT.
 */
static void
handle_commands (void)
{
    char      line[COMMAND_LINE_MAX];
    uint8_t   frame[MQTT_BRIDGE_CMD_MAX];
    uint32_t  len     = 0u;
    uint8_t   rx_byte = 0u;
    uint32_t  count   = 0u;
    command_t cmd     = {COMMAND_INVALID, 0u, {0.0f, 0.0f, 0.0f}, 0u};

    while ((count < MAX_BYTES_PER_POLL) && (console_in_getc(&rx_byte)))
    {
        count++;

        if (command_line_feed(&g_line, rx_byte, line, sizeof(line)))
        {
            cmd.req_id = 0u;

            if (command_parse(line, &cmd))
            {
                reply(&cmd, execute(&cmd), line);
            }
            else
            {
                reply(&cmd, FRAMES_ERR_INVALID, line);
            }
        }
    }

    while (mqtt_bridge_get_command(frame, sizeof(frame), &len))
    {
        if (frames_parse_command(frame, len, &cmd))
        {
            reply(&cmd, execute(&cmd), "(mqtt)");
        }
        else
        {
            reply(&cmd, FRAMES_ERR_INVALID, "(mqtt)");
        }
    }
}

/*!
 * @brief Send the reply frame and a readable line on the console.
 *
 * @param[in] p_cmd  The command (kind COMMAND_INVALID if it failed).
 * @param[in] error  FRAMES_OK or why it was refused.
 * @param[in] p_text Where it came from (console text or "(mqtt)").
 */
static void
reply (command_t const * p_cmd, frames_error_t error, char const * p_text)
{
    /* Cast: tm_printf() takes the kernel's UB string type; the literal is
       ASCII and the id is printed as an unsigned 32-bit value. */
    (void) tm_printf((UB const *) "# cmd #%u %s: %s%s\n",
                     (UINT) p_cmd->req_id, p_text,
                     (FRAMES_OK == error) ? "ok" : "refused: ",
                     (FRAMES_OK == error) ? "" : error_text(error));
    g_frame_len = frames_reply(g_frame, sizeof(g_frame), p_cmd->req_id,
                               p_cmd->kind, error);
    emit(TOPIC_EVENT, true);
}

/*!
 * @brief Carry out one valid command.  Tuning commands are refused while
 *        the car is driving (CSC05 review, FR10).
 *
 * @param[in] p_cmd Valid command.
 *
 * @return FRAMES_OK, or the reason it did not run.
 */
static frames_error_t
execute (command_t const * p_cmd)
{
    bool b_allowed = command_allowed(p_cmd->kind, stationary());
    bool b_ok      = b_allowed;

    /* A refused command runs nothing: COMMAND_INVALID is the default. */
    switch (b_allowed ? p_cmd->kind : COMMAND_INVALID)
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
            /* Cast: command_check() only accepts 0 or 50..5000 ms. */
            g_period_ms = (uint32_t) car_math_round(p_cmd->arg[0]);
            g_gains_due = GAINS_REPORT_TICKS;
        break;

        case COMMAND_GAINS:
            g_gains_due = GAINS_REPORT_TICKS;
        break;

        case COMMAND_HELP:
            emit_help();
        break;

        case COMMAND_TIMING:
            emit_timing();
        break;

        case COMMAND_NAV:
            /* As if this barcode was read (testing turns on the line). */
            b_ok = vehicle_command(VCMD_NAV, car_math_round(p_cmd->arg[0]));
        break;

        case COMMAND_MAP:
            /* Casts: command_check() only accepts 1..4 and 0..42. */
            barcode_decode_set_symbol(
                (barcode_decode_nav_t) car_math_round(p_cmd->arg[0]),
                packed_symbol_char((uint32_t) car_math_round(p_cmd->arg[1])));
            (void) tm_printf((UB const *) "# map L=%c R=%c S=%c U=%c\n",
                             (INT) barcode_decode_symbol(NAV_LEFT),
                             (INT) barcode_decode_symbol(NAV_RIGHT),
                             (INT) barcode_decode_symbol(NAV_STRAIGHT),
                             (INT) barcode_decode_symbol(NAV_UTURN));
        break;

        default:
            /* Refused: the result below says why. */
        break;
    }

    return ((!b_allowed) ? FRAMES_ERR_DRIVING
                         : (b_ok ? FRAMES_OK : FRAMES_ERR_UNAVAILABLE));
}

/*!
 * @brief Readable text for a reply error code.
 *
 * @param[in] error Error code.
 *
 * @return Text for the console.
 */
static char const *
error_text (frames_error_t error)
{
    char const * p_text = command_error();

    if (FRAMES_ERR_DRIVING == error)
    {
        p_text = "only while IDLE or STOPPED";
    }
    else if (FRAMES_ERR_UNAVAILABLE == error)
    {
        p_text = "not available in this build";
    }
    else
    {
        /* FRAMES_ERR_INVALID: the parser's reason. */
    }

    return (p_text);
}

/*!
 * @brief Publish g_frame if MQTT is enabled; print it on the console as
 *        '@' + hex when asked to or when there is no MQTT.
 *
 * @param[in] topic     Topic to publish on.
 * @param[in] b_console true to print it on the console as well.
 */
static void
emit (mqtt_bridge_topic_t topic, bool b_console)
{
    mqtt_bridge_stats_t link_stats;
    uint32_t            idx = 0u;

    mqtt_bridge_get_stats(&link_stats);

    if ((LINK_DISABLED != link_stats.link) && (g_frame_len > 0u))
    {
        (void) mqtt_bridge_publish(topic, g_frame, g_frame_len);
    }

    if ((b_console || (LINK_DISABLED == link_stats.link))
        && (g_frame_len > 0u))
    {
        g_hex[0] = '@';

        for (idx = 0u; idx < g_frame_len; idx++)
        {
            g_hex[1u + (idx * HEX_DIGITS)] =
                g_hex_chars[(g_frame[idx] >> NIBBLE_BITS) & NIBBLE_MASK];
            g_hex[2u + (idx * HEX_DIGITS)] =
                g_hex_chars[g_frame[idx] & NIBBLE_MASK];
        }

        g_hex[1u + (g_frame_len * HEX_DIGITS)] = TERMINATOR;

        /* Cast: tm_printf() takes the kernel's UB string type; ASCII. */
        (void) tm_printf((UB const *) "%s\n", g_hex);
    }
}

/*!
 * @brief Telemetry frame into g_frame (43 bytes).
 */
static void
build_telemetry (void)
{
    frames_telemetry_t   tel;
    ir_sensor_snapshot_t ir_snap;
    barcode_stats_t      bc_stats;
    uint32_t             idx = 0u;

    vehicle_get_status(&tel.veh);
    motion_get_status(&tel.motion);
    imu_get_status(&tel.imu);
    ir_sensor_read(&ir_snap);
    barcode_get_stats(&bc_stats);

    for (idx = 0u; idx < FRAMES_IR_COUNT; idx++)
    {
        tel.ir_norm[idx] = ir_snap.norm[idx];
    }

    tel.barcode = bc_stats.last.symbol;
    tel.faults  = fault_mask();
    g_frame_len = frames_telemetry(g_frame, sizeof(g_frame), &tel);
}

/*!
 * @brief Publish the heartbeat (uptime, link statistics, health).
 */
static void
send_heartbeat (void)
{
    frames_heartbeat_t  beat;
    vehicle_status_t    veh;
    imu_status_t        imu;
    mqtt_bridge_stats_t link_stats;
    SYSTIM              now = {0, 0u};
    uint32_t            idx = 0u;

    vehicle_get_status(&veh);
    imu_get_status(&imu);
    mqtt_bridge_get_stats(&link_stats);
    /* Operating time since boot: the system clock (tk_get_tim) starts at
       a calendar base, so its seconds always clamped to the 20-bit
       maximum (up=1048575 in every heartbeat, 29 Oct). */
    (void) tk_get_otm(&now);
    g_hb_seq++;

    /* Casts: enumeration constants are small and non-negative. */
    beat.up_s          = now.lo / MS_PER_S;
    beat.seq           = g_hb_seq;
    beat.state         = (uint32_t) veh.state;
    beat.link          = (uint32_t) link_stats.link;
    beat.published     = link_stats.published;
    beat.dropped       = link_stats.dropped;
    beat.commands      = link_stats.commands;
    beat.reconnects    = link_stats.reconnects;
    beat.rejoins       = link_stats.rejoins;
    beat.wifi_restarts = link_stats.wifi_restarts;
    beat.wifi_code     = link_stats.wifi_code;
    beat.address       = link_stats.address;
    beat.attempts      = link_stats.attempts;
    beat.why_code      = link_stats.why_code;
    beat.b_imu_ok      = imu.b_ok;
    beat.i2c_errors    = imu.i2c_errors;
    beat.faults        = fault_mask();

    for (idx = 0u; idx < (uint32_t) FAULT_COUNT; idx++)
    {
        /* Cast: idx is below FAULT_COUNT, a valid fault_id_t. */
        beat.fault_counts[idx] = fault_count((fault_id_t) idx);
    }

    g_frame_len = frames_heartbeat(g_frame, sizeof(g_frame), &beat);
    emit(TOPIC_HEARTBEAT, HEARTBEAT_ON_CONSOLE);
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

    float32_t jerk_mg = imu_take_peak_jerk();
    float32_t shock_mg = (jerk_mg > g_jerk_prev_mg) ? jerk_mg : g_jerk_prev_mg;

    g_jerk_prev_mg = jerk_mg;
    vehicle_get_status(&veh);
    barcode_get_stats(&bc_stats);
    imu_get_status(&imu);

    if (veh.state_seq != g_seen_state)
    {
        g_seen_state = veh.state_seq;
        g_frame_len  = frames_state(g_frame, sizeof(g_frame), veh.state,
                                    packed_reason_code(veh.p_reason));
        emit(TOPIC_EVENT, false);
    }

    if (bc_stats.decoded != g_seen_barcode)
    {
        g_seen_barcode = bc_stats.decoded;
        g_frame_len    = frames_barcode(g_frame, sizeof(g_frame),
                                        bc_stats.last.symbol,
                                        bc_stats.last.cmd);
        emit(TOPIC_EVENT, false);
    }

    if (imu.hump.humps != g_seen_humps)
    {
        g_seen_humps = imu.hump.humps;
        g_frame_len  = frames_hump(g_frame, sizeof(g_frame), imu.hump.humps,
                                   imu.hump.peak_mm, imu.hump.max_peak_mm);
        emit(TOPIC_EVENT, true);
    }

    emit_obstacle(&veh);

    if (imu.impacts < g_seen_impacts)
    {
        g_seen_impacts = imu.impacts; /* counter cleared for a new run */
    }

    if (imu.impacts > g_seen_impacts)
    {
        g_seen_impacts = imu.impacts;
        g_frame_len    = frames_impact(g_frame, sizeof(g_frame), imu.impacts,
                                       shock_mg);
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

    if (scans != g_seen_scans)
    {
        g_seen_scans = scans;
        g_frame_len  = frames_obstacle(g_frame, sizeof(g_frame), &prof,
                                       p_veh->last_action);
        emit(TOPIC_EVENT, false);
    }
}

/*!
 * @brief Current gains: one frame plus a readable console line.
 */
static void
emit_gains (void)
{
    frames_gains_t   gains;
    motion_gains_t   speed;
    vehicle_status_t veh;
    char             text[GAIN_TEXTS][COMMAND_DECIMAL_MAX];

    motion_get_gains(&speed);
    vehicle_get_status(&veh);
    gains.pid[0] = speed.gain_p;
    gains.pid[1] = speed.gain_i;
    gains.pid[2] = speed.gain_d;
    gains.ff[0]  = speed.gain_ff;
    gains.ff[1]  = speed.offset_pct;
    line_follow_get_gains(&gains.line[0], &gains.line[1], &gains.line[2]);
    gains.speed_mm_s = veh.cruise_mm_s;
    gains.rate_ms    = g_period_ms;
    command_format_decimal(text[0], gains.pid[0]);
    command_format_decimal(text[1], gains.pid[1]);
    command_format_decimal(text[2], gains.pid[2]);
    command_format_decimal(text[3], gains.ff[0]);
    command_format_decimal(text[4], gains.ff[1]);
    command_format_decimal(text[5], gains.line[0]);
    command_format_decimal(text[6], gains.line[1]);
    command_format_decimal(text[7], gains.line[2]);

    /* Cast: tm_printf() takes the kernel's UB string type; ASCII.  The
       speed and rate are small integers. */
    (void) tm_printf((UB const *) "# gains pid=%s,%s,%s ff=%s,%s "
                                  "line=%s,%s,%s speed=%d rate=%u\n",
                     text[0], text[1], text[2], text[3], text[4], text[5],
                     text[6], text[7], (INT) gains.speed_mm_s,
                     (UINT) gains.rate_ms);
    g_frame_len = frames_gains(g_frame, sizeof(g_frame), &gains);
    emit(TOPIC_EVENT, true);
}

/*!
 * @brief Human-readable usage on the console (no frame: it is text for a
 *        person at the console).
 */
static void
emit_help (void)
{
    /* Cast: tm_printf() takes the kernel's UB string type; ASCII. */
    (void) tm_printf(
        (UB const *) "# commands: start | stop | calibrate | gains | help "
                     "| timing\n"
                     "#   speed=<mm/s>        cruise speed (50..300)\n"
                     "#   pid=<kp>,<ki>,<kd>  wheel speed PID\n"
                     "#   ff=<kf>,<offset>    feed-forward + friction "
                     "offset (pct)\n"
                     "#   line=<kp>,<ki>,<kd> line-following PID\n"
                     "#   rate=<ms>           telemetry period, 0 = off\n"
                     "#   nav=<1..4>          as if a barcode was read: 1 "
                     "left, 2 right, 3 straight, 4 U-turn\n"
                     "#   map=<1..4>,<idx>    barcode letter for that turn, "
                     "Code 39 index (A=10 .. Z=35)\n"
                     "#   speed=, pid=, ff=, line=, map= only while IDLE "
                     "or STOPPED\n"
                     "#   timing: task times, control jitter and CPU load "
                     "since the last timing\n"
                     "# MQTT takes the same commands as binary frames "
                     "(tools/dashboard/frames.py)\n");
    encoder_check_filter();
}

/*!
 * @brief Report the timing window and start a new one (CSC05 review: WCET,
 *        CPU load and the A2 control-jitter test): every section on the
 *        console as [max_us, avg_us, count], and a summary frame.
 */
static void
emit_timing (void)
{
    timing_report_t report;
    timing_stat_t   stat;
    frames_timing_t frame = {0u, 0u, 0u, 0u, {0u}};
    uint32_t        idx   = 0u;
    uint32_t        busy = 0u;

    timing_take(&report);
    busy = report.window_ms - report.idle_ms;
    frame.window_ms = report.window_ms;
    frame.load_pct =
        (0u == report.window_ms) ? 0u : ((busy * PERCENT) / report.window_ms);
    frame.jitter_max_us = report.jitter_max_us;
    frame.periods       = report.periods;

    /* Casts: tm_printf() takes the kernel's UB string type (ASCII); the
       values are 32-bit unsigned. */
    (void) tm_printf((UB const *) "# timing win_ms=%u load_pct=%u "
                                  "jit_max_us=%u periods=%u\n",
                     (UINT) frame.window_ms, (UINT) frame.load_pct,
                     (UINT) frame.jitter_max_us, (UINT) frame.periods);

    /* Cast: TIMING_COUNT is a small positive enumeration constant. */
    for (idx = 0u; idx < (uint32_t) TIMING_COUNT; idx++)
    {
        stat = report.stat[idx];
        if (idx < FRAMES_TIMING_SLOTS)
        {
            frame.max_us[idx] = stat.max_us;
        }

        /* Casts: idx is below TIMING_COUNT, a valid timing_id_t; the
           values are 32-bit unsigned. */
        (void) tm_printf(
            (UB const *) "#   %s [%u, %u, %u]\n",
            timing_name((timing_id_t) idx), (UINT) stat.max_us,
            (UINT) ((0u == stat.count) ? 0u : (stat.sum_us / stat.count)),
            (UINT) stat.count);
    }

    g_frame_len = frames_timing(g_frame, sizeof(g_frame), &frame);
    emit(TOPIC_EVENT, true);
}

/*!
 * @brief Whether the car is standing still (IDLE or STOPPED); test builds
 *        have no running vehicle task and report IDLE.
 *
 * @return true if tuning commands may run.
 */
static bool
stationary (void)
{
    vehicle_status_t veh;

    vehicle_get_status(&veh);

    return ((VS_IDLE == veh.state) || (VS_STOPPED == veh.state));
}

/*** end of file ***/
