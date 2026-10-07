/** @file app_tests.c
 *
 * @brief Per-subsystem test programs (see app_tests.h).
 *
 * The START button (GP20) runs or repeats the selected test.  Printed
 * values are rounded to whole units (mm, mm/s, percent) or scaled by 10
 * (tenths of a degree) or 1000 (permille) where noted in the CSV header.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "car_math.h"
#include "hal.h"
#include "encoder.h"
#include "motion.h"
#include "ir_sensor.h"
#include "line_follow.h"
#include "barcode.h"
#include "imu.h"
#include "obstacle.h"
#include "telemetry.h"
#include "command.h"
#include "app_tests.h"

#if (APP_MODE != APP_MODE_MISSION) && (APP_MODE != APP_MODE_TEST_TELEMETRY)
#define TEST_TASK_BUILD (1)
#else
#define TEST_TASK_BUILD (0)
#endif

#define TASK_PRIORITY    (11)
#define TASK_STACK       (3072)
#define PRINT_MS         (100u)
#define STEP_MS          (1000u)
#define RELEASE_MS       (500u) /* time to let go of the car        */
#define DUTY_STEP        (10)
#define DUTY_MAX         (100)
#define LOG_MS           (40u)
#define STEP_LOG_MS      (3000u)
#define TEST_DURATION_MS (30000u) /* streaming tests run for 30 s     */
#define CALIBRATION_MS   (5000u)
#define MOVE_TIMEOUT_MS  (10000)
#define SLOW_TIMEOUT_MS  (20000)
#define TENTHS           (10.0f)
#define PERMILLE         (1000.0f)
#define TEST_DIST_MM     (500.0f)
#define BARCODE_RUN_MM   (1000.0f)
#define QUARTER_TURN_DEG (90.0f)
#define HALF_TURN_DEG    (180.0f)
#define GAIN_COUNT       (5u)

#if TEST_TASK_BUILD
static void test_task(INT stacd, void * p_exinf);
static void wait_start_button(void);
static void run_test(void);
#endif

/* The barcode test prints counters only. */
#if TEST_TASK_BUILD && (APP_MODE != APP_MODE_TEST_BARCODE)
#define PRINT_INT_NEEDED (1)
static INT print_int(float32_t value);
#else
#define PRINT_INT_NEEDED (0)
#endif

#if APP_MODE == APP_MODE_TEST_MOTOR
static void log_step(float32_t target);
static void sweep(void);

/* Whether the open-loop sweep has run; used only by the test task. */
static bool gb_swept = false;
#endif

#if APP_MODE == APP_MODE_TEST_MOTION
static void report_move(char const * p_what, float32_t before_odo,
                        float32_t before_heading);
#endif

#if APP_MODE == APP_MODE_TEST_ULTRASONIC
static char const * side_name(int32_t angle_deg);
#endif

/*!
 * @brief Initialise only what the selected test needs and start its task.
 *
 * @return E_OK or a kernel error code; always E_OK in mission and
 *         telemetry builds, which have no separate test task.
 */
int32_t
app_tests_start (void)
{
    int32_t ercd = E_OK;

#if TEST_TASK_BUILD
    T_CTSK ctsk   = {0};
    ID     h_task = 0;

#if APP_MODE == APP_MODE_TEST_MOTOR
    (void) encoder_init();
    (void) motion_init();
#elif APP_MODE == APP_MODE_TEST_MOTION
    (void) encoder_init();
    (void) motion_init();
#elif APP_MODE == APP_MODE_TEST_IR
    (void) ir_sensor_init();
#elif APP_MODE == APP_MODE_TEST_BARCODE
    (void) encoder_init();
    (void) motion_init();
    (void) ir_sensor_init();
    (void) barcode_init();
#elif APP_MODE == APP_MODE_TEST_IMU
    (void) encoder_init();
    (void) motion_init();
    (void) imu_init();
#elif APP_MODE == APP_MODE_TEST_ULTRASONIC
    (void) obstacle_init();
#else
#error "Unknown APP_MODE"
#endif

    /* Commands (pid=, ff=, line=, rate=, stop ...) work in every test
       build over the USB / UART console; telemetry streams on "rate=". */
    (void) telemetry_init(false);

    ctsk.tskatr = TA_HLNG | TA_RNG3;

    /* Cast: the kernel stores every task entry as the generic FP type and
       calls a TA_HLNG task as (INT stacd, void * exinf), matching
       test_task(). */
    ctsk.task    = (FP) test_task;
    ctsk.itskpri = TASK_PRIORITY;
    ctsk.stksz   = TASK_STACK;
    h_task       = tk_cre_tsk(&ctsk);
    ercd         = (h_task >= E_OK) ? tk_sta_tsk(h_task, 0) : h_task;
#endif

    return (ercd);
}

#if TEST_TASK_BUILD
/*!
 * @brief Test task: run the selected test each time START is pressed.
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
test_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;
    hal_gpio_init_in(PIN_BTN_START, true);

    for (;;)
    {
        wait_start_button();
        run_test();

        /* Cast: tm_printf() takes the kernel's UB string type; the literal
           is ASCII. */
        (void) tm_printf((UB const *) "# test done\n");
    }
}

/*!
 * @brief Wait until START (active low) is pressed, then give the user
 *        time to let go of the car.
 */
static void
wait_start_button (void)
{
    /* Cast: tm_printf() takes the kernel's UB string type; the literal is
       ASCII. */
    (void) tm_printf((UB const *) "# press START (GP20)\n");

    while (hal_gpio_get(PIN_BTN_START))
    {
        (void) tk_dly_tsk(PRINT_MS);
    }

    (void) tk_dly_tsk(RELEASE_MS);
}
#endif

#if PRINT_INT_NEEDED
/*!
 * @brief Round a measurement for printing with %d.
 *
 * @param[in] value Measurement (well inside the int32_t range).
 *
 * @return Nearest integer.
 */
static INT
print_int (float32_t value)
{
    return (car_math_round(value));
}
#endif

#if APP_MODE == APP_MODE_TEST_MOTOR
/*!
 * @brief Buddy 2 motor test: open-loop duty sweep (motor curve for the
 *        feed-forward), then closed-loop step responses for PID tuning.
 *
 * Tuning loop: type e.g. "pid=0.1,0.8,0" or "ff=0.1,18" on the serial
 * console (or in the dashboard) and press START again; only the step test
 * is repeated, with the new gains printed above the CSV.
 * CSV: t_ms,target,spdL,spdR,pwmL,pwmR
 */
static void
run_test (void)
{
    motion_gains_t gains;
    char           gain_text[GAIN_COUNT][COMMAND_DECIMAL_MAX];

    /* The sweep runs on the first press only; START repeats the steps. */
    if (!gb_swept)
    {
        sweep();
        gb_swept = true;
    }

    motion_get_gains(&gains);
    command_format_decimal(gain_text[0], gains.gain_p);
    command_format_decimal(gain_text[1], gains.gain_i);
    command_format_decimal(gain_text[2], gains.gain_d);
    command_format_decimal(gain_text[3], gains.gain_ff);
    command_format_decimal(gain_text[4], gains.offset_pct);

    /* Casts: tm_printf() takes the kernel's UB string type; the literals
       are ASCII. */
    (void) tm_printf((UB const *) "# step test with pid=%s,%s,%s ff=%s,%s\n",
                     gain_text[0], gain_text[1], gain_text[2], gain_text[3],
                     gain_text[4]);
    (void) tm_printf((UB const *) "# closed loop steps: t,target,spdL,spdR,"
                                  "pwmL,pwmR\n");
    log_step(SPEED_SLOW_MM_S);
    log_step(SPEED_CRUISE_MM_S);
    log_step(0.0f);
    motion_stop();
}

/*!
 * @brief Command one wheel speed step and log the response.
 *
 * @param[in] target Speed set-point for both wheels in mm/s.
 */
static void
log_step (float32_t target)
{
    motion_status_t motion;
    uint32_t        elapsed_ms = 0u;

    motion_set_velocity(target, target);

    for (elapsed_ms = 0u; elapsed_ms < STEP_LOG_MS; elapsed_ms += LOG_MS)
    {
        motion_get_status(&motion);

        /* Casts: tm_printf() takes the kernel's UB string type (the
           literal is ASCII); elapsed_ms is below STEP_LOG_MS, so it fits
           UINT. */
        (void) tm_printf((UB const *) "%u,%d,%d,%d,%d,%d\n", (UINT) elapsed_ms,
                         print_int(target), print_int(motion.speed_l_mm_s),
                         print_int(motion.speed_r_mm_s),
                         print_int(motion.pwm_l_pct),
                         print_int(motion.pwm_r_pct));
        (void) tk_dly_tsk(LOG_MS);
    }
}

/*!
 * @brief Open-loop duty sweep through the motion task (so that nothing
 *        fights the PID).
 */
static void
sweep (void)
{
    motion_status_t motion;
    int32_t         duty = 0;

    /* Casts in the messages: tm_printf() takes the kernel's UB string
       type; the literals are ASCII. */
    (void) tm_printf((UB const *) "# open loop: duty,spdL,spdR (mm/s)\n");

    for (duty = 0; duty <= DUTY_MAX; duty += DUTY_STEP)
    {
        /* Casts: duty is 0..100, which float32_t holds exactly and which
           fits INT. */
        motion_set_open_loop((float32_t) duty, (float32_t) duty);
        (void) tk_dly_tsk(STEP_MS);
        motion_get_status(&motion);
        (void) tm_printf((UB const *) "%d,%d,%d\n", (INT) duty,
                         print_int(motion.speed_l_mm_s),
                         print_int(motion.speed_r_mm_s));
    }

    motion_stop();
    (void) tk_dly_tsk(STEP_MS);
}
#endif

#if APP_MODE == APP_MODE_TEST_MOTION
/*!
 * @brief Buddy 2 accuracy test.  Measure the real distance and angle with
 *        a ruler and protractor and compare them with the encoder figures
 *        printed here.
 */
static void
run_test (void)
{
    motion_status_t motion;

    motion_get_status(&motion);
    motion_move_forward(TEST_DIST_MM, SPEED_CRUISE_MM_S);
    (void) motion_wait_done(MOVE_TIMEOUT_MS);
    report_move("forward 500", motion.odo_mm, motion.heading_deg);

    motion_get_status(&motion);
    motion_turn_left(QUARTER_TURN_DEG);
    (void) motion_wait_done(MOVE_TIMEOUT_MS);
    report_move("left 90", motion.odo_mm, motion.heading_deg);

    motion_get_status(&motion);
    motion_turn_right(QUARTER_TURN_DEG);
    (void) motion_wait_done(MOVE_TIMEOUT_MS);
    report_move("right 90", motion.odo_mm, motion.heading_deg);

    motion_get_status(&motion);
    motion_move_backward(TEST_DIST_MM, SPEED_CRUISE_MM_S);
    (void) motion_wait_done(MOVE_TIMEOUT_MS);
    report_move("backward 500", motion.odo_mm, motion.heading_deg);

    motion_get_status(&motion);
    motion_turn_left(HALF_TURN_DEG);
    (void) motion_wait_done(MOVE_TIMEOUT_MS);
    report_move("u-turn 180", motion.odo_mm, motion.heading_deg);
}

/*!
 * @brief Print the distance and heading change of the last move.
 *
 * @param[in] p_what         Move description.
 * @param[in] before_odo     Odometer before the move (mm).
 * @param[in] before_heading Heading before the move (degrees).
 */
static void
report_move (char const * p_what, float32_t before_odo,
             float32_t before_heading)
{
    motion_status_t motion;
    char            heading_text[COMMAND_DECIMAL_MAX];

    motion_get_status(&motion);
    command_format_decimal(heading_text, motion.heading_deg - before_heading);

    /* Cast: tm_printf() takes the kernel's UB string type; the literal is
       ASCII. */
    (void) tm_printf((UB const *) "%s: odo=%d mm heading=%s deg ok=%d\n",
                     p_what, print_int(motion.odo_mm - before_odo),
                     heading_text, motion.b_last_ok ? 1 : 0);
}
#endif

#if APP_MODE == APP_MODE_TEST_IR
/*!
 * @brief Buddy 3 line-sensor test: a 5 s manual calibration (slide the car
 *        across the line), then raw and normalised values, line error and
 *        state at 10 Hz.  Normalised values are printed in permille.
 */
static void
run_test (void)
{
    ir_sensor_snapshot_t ir_snap;
    line_follow_t        follower;
    uint32_t             elapsed_ms = 0u;

    /* Casts in the messages: tm_printf() takes the kernel's UB string
       type; the literals are ASCII. */
    (void) tm_printf((UB const *) "# calibrating 5 s: sweep sensors over "
                                  "line+floor\n");
    ir_sensor_calib_begin();
    (void) tk_dly_tsk(CALIBRATION_MS);
    (void) tm_printf((UB const *) "# calibration %s\n",
                     ir_sensor_calib_end() ? "OK" : "FAILED (low contrast)");
    line_follow_init(&follower);
    (void) tm_printf((UB const *) "# rawL,rawR,rawB,nL,nR,nB,err,state\n");

    for (elapsed_ms = 0u; elapsed_ms < TEST_DURATION_MS; elapsed_ms += PRINT_MS)
    {
        ir_sensor_read(&ir_snap);
        (void) line_follow_classify(&follower, ir_snap.norm[IR_LEFT],
                                    ir_snap.norm[IR_RIGHT], 0.0f, PRINT_MS);

        /* Casts: raw values are 12-bit ADC counts and the enumeration
           value is 0..2, so both fit INT. */
        (void) tm_printf(
            (UB const *) "%d,%d,%d,%d,%d,%d,%d,%d\n",
            (INT) ir_snap.raw[IR_LEFT], (INT) ir_snap.raw[IR_RIGHT],
            (INT) ir_snap.raw[IR_BARCODE],
            print_int(ir_snap.norm[IR_LEFT] * PERMILLE),
            print_int(ir_snap.norm[IR_RIGHT] * PERMILLE),
            print_int(ir_snap.norm[IR_BARCODE] * PERMILLE),
            print_int((ir_snap.norm[IR_LEFT] - ir_snap.norm[IR_RIGHT])
                      * PERMILLE),
            (INT) follower.state);
        (void) tk_dly_tsk(PRINT_MS);
    }
}
#endif

#if APP_MODE == APP_MODE_TEST_BARCODE
/*!
 * @brief Buddy 3 barcode test: drive slowly and straight over a barcode
 *        (pushing the car by hand also works); barcode.c prints each
 *        decode.
 */
static void
run_test (void)
{
    barcode_stats_t bc_stats;

    motion_move_forward(BARCODE_RUN_MM, SPEED_SLOW_MM_S);
    (void) motion_wait_done(SLOW_TIMEOUT_MS);
    barcode_get_stats(&bc_stats);

    /* Casts: tm_printf() takes the kernel's UB string type (the literal is
       ASCII); the counters are 32-bit unsigned like UINT. */
    (void) tm_printf((UB const *) "# frames=%u decoded=%u errors=%u "
                                  "last='%s'\n",
                     (UINT) bc_stats.frames, (UINT) bc_stats.decoded,
                     (UINT) bc_stats.errors, bc_stats.last.text);
}
#endif

#if APP_MODE == APP_MODE_TEST_IMU
/*!
 * @brief Buddy 4 IMU test: push or drive the car over a hump; prints tilt
 *        (tenths of a degree), rates, hump height and motion events.
 */
static void
run_test (void)
{
    imu_status_t imu_state;
    uint32_t     elapsed_ms = 0u;

    /* Casts in the messages: tm_printf() takes the kernel's UB string
       type; the literals are ASCII. */
    (void) tm_printf((UB const *) "# ok,pitch,roll,yaw,encRate,magRate,h,"
                                  "peak,max,n,event,impacts\n");

    for (elapsed_ms = 0u; elapsed_ms < TEST_DURATION_MS; elapsed_ms += PRINT_MS)
    {
        imu_get_status(&imu_state);

        /* Casts: the counters are 32-bit unsigned like UINT. */
        (void) tm_printf(
            (UB const *) "%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%s,%u\n",
            imu_state.b_ok ? 1 : 0, print_int(imu_state.pitch_deg * TENTHS),
            print_int(imu_state.roll_deg * TENTHS),
            print_int(imu_state.heading_deg * TENTHS),
            print_int(imu_state.enc_rate_dps),
            print_int(imu_state.mag_rate_dps),
            print_int(imu_state.hump.height_mm),
            print_int(imu_state.hump.peak_mm),
            print_int(imu_state.hump.max_peak_mm), (UINT) imu_state.hump.humps,
            terrain_event_name(imu_state.event), (UINT) imu_state.impacts);
        (void) tk_dly_tsk(PRINT_MS);
    }
}
#endif

#if APP_MODE == APP_MODE_TEST_ULTRASONIC
/*!
 * @brief Buddy 5 test: full coarse-to-fine scan, profile and plan, plus a
 *        servo direction check (assumption A9): a box ahead but offset to
 *        the car's right must be found on the RIGHT (closest angle below
 *        90 degrees); otherwise set SERVO_INVERT in car_config.h.  The
 *        raw scan points follow as "pt,angle,mm" lines for
 *        tools/scan_plot.py.
 */
static void
run_test (void)
{
    avoidance_profile_t profile = {0};
    avoidance_plan_t    plan;
    avoidance_point_t   points[SCAN_MAX_POINTS];
    uint32_t            count = 0u;
    uint32_t            idx   = 0u;

    /* Casts in the messages: tm_printf() takes the kernel's UB string type
       (the literals are ASCII); distances, angles and clearances are at
       most a few thousand, so they fit INT. */
    (void) tm_printf((UB const *) "# front=%d mm\n", (INT) obstacle_front_mm());
    (void) obstacle_scan(&profile, NULL);
    plan = avoidance_plan(&profile, 0u);
    (void) tm_printf(
        (UB const *) "# found=%d closest=%d@%d edges L=%d R=%d width=%d "
                     "clear L=%d R=%d -> %s offset=%d\n",
        profile.b_found ? 1 : 0, (INT) profile.closest_mm,
        (INT) profile.closest_angle, print_int(profile.left_edge_mm),
        print_int(profile.right_edge_mm), print_int(profile.width_mm),
        (INT) profile.clear_left_mm, (INT) profile.clear_right_mm,
        avoidance_action_name(plan.action), print_int(plan.offset_mm));

    /* Raw points of this scan; casts as above. */
    count = obstacle_get_points(points, SCAN_MAX_POINTS);
    (void) tm_printf((UB const *) "# points=%d (pt,angle_deg,dist_mm; -1 = "
                                  "no echo)\n",
                     (INT) count);

    for (idx = 0u; idx < count; idx++)
    {
        (void) tm_printf((UB const *) "pt,%d,%d\n", (INT) points[idx].angle_deg,
                         (INT) points[idx].dist_mm);
    }

    if (profile.b_found)
    {
        (void) tm_printf((UB const *) "# closest echo on the %s (box offset "
                                      "right must say RIGHT, else set "
                                      "SERVO_INVERT 1)\n",
                         side_name(profile.closest_angle));
    }
}

/*!
 * @brief Side of the car a servo angle points to.
 *
 * @param[in] angle_deg Servo angle, 0 (right) .. 180 (left).
 *
 * @return "RIGHT", "LEFT" or "AHEAD".
 */
static char const *
side_name (int32_t angle_deg)
{
    char const * p_side = "AHEAD";

    if (angle_deg < SERVO_CENTRE_DEG)
    {
        p_side = "RIGHT";
    }
    else if (angle_deg > SERVO_CENTRE_DEG)
    {
        p_side = "LEFT";
    }
    else
    {
        /* Straight ahead. */
    }

    return (p_side);
}
#endif

/*** end of file ***/
