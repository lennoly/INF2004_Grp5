/** @file subsystem2_test.c
 *
 * @brief Buddy 2 test programs: motor curve and speed-loop steps
 *        (TEST_MOTOR), move accuracy (TEST_MOTION), raw motor pin check
 *        (TEST_WHEELS).  Printed values are rounded to whole units (mm,
 *        mm/s, percent).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "hal.h"
#include "encoder.h"
#include "motor.h"
#include "motion.h"
#include "command.h"
#include "app_tests.h"
#include "subsystem2_test.h"

#define STEP_MS          (1000u)
#define DUTY_STEP        (10)
#define DUTY_MAX         (100)
#define LOG_MS           (40u)
#define STEP_LOG_MS      (3000u)
#define MOVE_TIMEOUT_MS  (10000)
#define TEST_DIST_MM     (500.0f)
#define QUARTER_TURN_DEG (90.0f)
#define HALF_TURN_DEG    (180.0f)
#define GAIN_COUNT       (5u)

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

#if APP_MODE == APP_MODE_TEST_WHEELS
#define WHEEL_STEP_MS  (2000u)
#define WHEEL_PWM_PCT  (60.0f)
#define PCT_FULL_POWER (100.0f)

static void wheel_step(char const * p_what);
static void pins_put(bool l_a, bool l_b, bool r_a, bool r_b);
static void pins_report(void);
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
void
subsystem2_test_motor (void)
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
#endif

#if APP_MODE == APP_MODE_TEST_MOTION
/*!
 * @brief Buddy 2 accuracy test.  Measure the real distance and angle with
 *        a ruler and protractor and compare them with the encoder figures
 *        printed here.
 */
void
subsystem2_test_motion (void)
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
#endif

#if APP_MODE == APP_MODE_TEST_MOTOR
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
                         app_tests_print_int(target),
                         app_tests_print_int(motion.speed_l_mm_s),
                         app_tests_print_int(motion.speed_r_mm_s),
                         app_tests_print_int(motion.pwm_l_pct),
                         app_tests_print_int(motion.pwm_r_pct));
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
                         app_tests_print_int(motion.speed_l_mm_s),
                         app_tests_print_int(motion.speed_r_mm_s));
    }

    motion_stop();
    (void) tk_dly_tsk(STEP_MS);
}
#endif

#if APP_MODE == APP_MODE_TEST_MOTION
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
                     p_what, app_tests_print_int(motion.odo_mm - before_odo),
                     heading_text, motion.b_last_ok ? 1 : 0);
}
#endif

#if APP_MODE == APP_MODE_TEST_WHEELS
/*!
 * @brief Run one wheel step for WHEEL_STEP_MS, then print the encoder
 *        edges counted on each side.
 *
 * @param[in] p_what Step description.
 */
static void
wheel_step (char const * p_what)
{
    encoder_reading_t before_l;
    encoder_reading_t before_r;
    encoder_reading_t after_l;
    encoder_reading_t after_r;

    encoder_read(ENCODER_LEFT, &before_l);
    encoder_read(ENCODER_RIGHT, &before_r);

    /* Cast: tm_printf() takes the kernel's UB string type; the literal is
       ASCII. */
    (void) tm_printf((UB const *) "%s ... ", p_what);
    (void) tk_dly_tsk(WHEEL_STEP_MS);

    encoder_read(ENCODER_LEFT, &after_l);
    encoder_read(ENCODER_RIGHT, &after_r);

    /* Casts: as above; a 2 s step gives far fewer than 2^31 edges. */
    (void) tm_printf((UB const *) "ticks L=%d R=%d\n",
                     (INT) (after_l.ticks - before_l.ticks),
                     (INT) (after_r.ticks - before_r.ticks));
}

/*!
 * @brief Drive the four motor pins as plain GPIO levels (no PWM).
 *
 * @param[in] l_a GP8 (M1A) level.
 * @param[in] l_b GP9 (M1B) level.
 * @param[in] r_a GP10 (M2A) level.
 * @param[in] r_b GP11 (M2B) level.
 */
static void
pins_put (bool l_a, bool l_b, bool r_a, bool r_b)
{
    hal_gpio_put(PIN_MOTOR_L_A, l_a);
    hal_gpio_put(PIN_MOTOR_L_B, l_b);
    hal_gpio_put(PIN_MOTOR_R_A, r_a);
    hal_gpio_put(PIN_MOTOR_R_B, r_b);
}

/*!
 * @brief Buddy 2 wheel check: each motor pin driven on its own, first as
 *        a plain GPIO level, then through the PWM driver, with no speed
 *        loop and no encoder feedback in the way.  Watch the GP8..GP11
 *        LEDs and the wheels; the encoder edges are printed per step.
 */
void
subsystem2_test_wheels (void)
{
    /* Casts in the messages: tm_printf() takes the kernel's UB string
       type; the literals are ASCII. */
    (void) tm_printf((UB const *) "# part 1: plain GPIO, full power, "
                                  "2 s each (no PWM)\n");
    hal_gpio_init_out(PIN_MOTOR_L_A, false);
    hal_gpio_init_out(PIN_MOTOR_L_B, false);
    hal_gpio_init_out(PIN_MOTOR_R_A, false);
    hal_gpio_init_out(PIN_MOTOR_R_B, false);

    pins_put(true, false, false, false);
    wheel_step("GP8  high: LEFT  forward ");
    pins_put(false, true, false, false);
    wheel_step("GP9  high: LEFT  backward");
    pins_put(false, false, true, false);
    wheel_step("GP10 high: RIGHT forward ");
    pins_put(false, false, false, true);
    wheel_step("GP11 high: RIGHT backward");
    pins_put(true, false, true, false);
    wheel_step("GP8+GP10 : BOTH  forward ");
    pins_put(false, false, false, false);
    (void) tk_dly_tsk(WHEEL_STEP_MS);

    (void) tm_printf((UB const *) "# part 2: PWM driver (motor_set), "
                                  "2 s each\n");
    motor_init();
    motor_set(WHEEL_PWM_PCT, 0.0f);
    pins_report();
    wheel_step("LEFT  forward  60%       ");
    motor_set(0.0f, WHEEL_PWM_PCT);
    pins_report();
    wheel_step("RIGHT forward  60%       ");
    motor_set(PCT_FULL_POWER, 0.0f);
    pins_report();
    wheel_step("LEFT  forward 100%       ");
    motor_set(0.0f, PCT_FULL_POWER);
    pins_report();
    wheel_step("RIGHT forward 100%       ");
    motor_set(WHEEL_PWM_PCT, WHEEL_PWM_PCT);
    pins_report();
    wheel_step("BOTH  forward  60%       ");
    motor_set(PCT_FULL_POWER, PCT_FULL_POWER);
    pins_report();
    wheel_step("BOTH  forward 100%       ");
    motor_brake();
}

/*!
 * @brief Print the hardware state of the four motor pins.
 */
static void
pins_report (void)
{
    static uint32_t const pins[] = {PIN_MOTOR_L_A, PIN_MOTOR_L_B,
                                    PIN_MOTOR_R_A, PIN_MOTOR_R_B};
    hal_pwm_info_t        info;
    uint32_t              idx = 0u;

    for (idx = 0u; idx < (sizeof(pins) / sizeof(pins[0])); idx++)
    {
        hal_pwm_inspect(pins[idx], &info);

        /* Casts: tm_printf() takes the kernel's UB string type (the
           literal is ASCII); every value is a register or a percentage
           printed in hex or as UINT. */
        (void) tm_printf((UB const *) "  GP%u: func=%u pad=%x csr=%x top=%u "
                                      "cc=%x high=%u%% reset=%u\n",
                         (UINT) pins[idx], (UINT) info.funcsel,
                         (UINT) info.pad, (UINT) info.csr, (UINT) info.top,
                         (UINT) info.cc, (UINT) info.high_pct,
                         info.b_reset ? 1u : 0u);
    }
}
#endif

/*** end of file ***/
