/** @file test_main.c
 *
 * @brief Host unit tests for the hardware-independent PicoCar modules.
 *        Build and run:  make -C tests/host
 *
 * Each check prints its name when it fails; the summary line counts all
 * passes and failures and the exit status is non-zero on any failure.
 */

#include "test_main.h"
#include "avoidance.h"
#include "barcode_decode.h"
#include "car_config.h"
#include "car_math.h"
#include "car_types.h"
#include "command.h"
#include "hal_stub.h"
#include "line_follow.h"
#include "lwip/apps/mqtt.h"
#include "mqtt_bridge.h"
#include "mqtt_lwip.h"
#include "obstacle.h"
#include "pid.h"
#include "servo.h"
#include "sim/vehicle_sim.h"
#include "terrain.h"
#include "vehicle.h"
#include "test_data.h"
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Maths accuracy sweep. */
#define SQRT_FIRST     (0.001f)
#define SQRT_LAST      (1e6f)
#define SQRT_STEP      (1.37f)
#define ANGLE_FIRST    (-3.14f)
#define ANGLE_LAST     (3.14f)
#define ANGLE_STEP     (0.01f)
#define SIN_SCALE      (3.0f)
#define DEG_PER_RAD    (57.29578f)
#define SQRT_TOLERANCE (1e-5f)
#define ATAN_TOLERANCE (0.01f)
#define SIN_TOLERANCE  (1e-4f)
#define LARGE_ANGLE    (100.0f) /* about 16 turns                 */
#define SIN_REDUCED    (1e-3f)  /* float error of the reduction   */
#define HUGE_ANGLE     (1e20f)  /* 2 pi is below its resolution   */

/* PID step response on a first-order motor model. */
#define PID_DT_S        (0.02f)
#define PID_RUN_S       (2.0f)
#define PID_TARGET      (180.0f)
#define PID_PCT_MAX     (100.0f)
#define MOTOR_GAIN      (5.0f)  /* mm/s per percent duty          */
#define MOTOR_TAU_S     (0.15f) /* time constant                  */
#define SETTLE_FRACTION (0.05f)
#define OVERSHOOT_LIMIT (0.15f)
#define WINDUP_STEPS    (500u)
#define WINDUP_SETPOINT (1000.0f)

/* Line follower. */
#define LINE_DT_MS     (20u)
#define LINE_DT_S      (0.02f)
#define LOST_SAMPLES   (10u) /* 200 ms of white                */
#define JUNCTION_EXTRA (3u)  /* 4 samples x 4 mm >= 10 mm      */
#define BAR_NARROW_MS  (40u)  /* about 7 mm at the sim speed    */
#define BAR_JUNCTION_MS (100u) /* about 18 mm                     */
#define FLICKER_SAMPLES (10u) /* 10 x 4 mm, below the re-arm     */
#define REARM_SAMPLES  (25u)  /* 25 x 4 mm = 100 mm              */

/* Barcode. */
#define PHOTO_ELEMENTS  (29u)
#define FRAME_MAX       (32u)
#define TEXT_SIZE       (9u)
#define CHARS_PER_FRAME (3u)
#define NARROW_MM       (3.0f)
#define WIDE_MM         (9.0f)
#define US_PER_MM       (5000.0f) /* 200 mm/s                    */
#define TRIALS          (250u)
#define WIDTH_NOISE     (0.20f)
#define DRIFT_PER_EL    (0.01f)
#define DRIFT_KINDS     (3u)
#define FLAT_WIDTH      (100u)
#define SHORT_ELEMENTS  (12u)
#define BIAS_MM         (2.0f)  /* bars read thinner, spaces wider */
#define BIAS_NOISE      (0.10f)
#define BIAS_TRIALS     (20u)
#define ARRAY_LEN(a)    (sizeof(a) / sizeof((a)[0]))

/* Terrain. */
#define HUMP_SPEED_MM_S (150.0f)
#define LIFT_SAMPLES    (100u) /* 2 s at 50 Hz */
#define HUMP_LENGTH_MM  (250.0f)
#define HUMP_LEAD_MM    (200.0f)
#define HUMP_TAIL_MM    (300.0f)
#define PITCH_NOISE_DEG (1.0f)
#define HEIGHT_COUNT    (4u)
#define HEIGHT_FRACTION (0.1f)
#define HEIGHT_SLACK_MM (2.0f)
#define PI_F            (3.14159f)
#define ROLL_NOISE_DEG  (0.5f)   /* roll is low-pass filtered           */
#define CASTER_MM       (100.0f) /* drive axle to caster (one-wheel sim) */
#define WHEEL_COUNT     (2u)     /* one-wheel hump heights tested       */
#define JOLT_FIRST      (20u)    /* roll step starts at sample 20       */
#define JOLT_SAMPLES    (5u)     /* 15 mm: a hand-push jolt             */
#define JUNK_MM         (70647)  /* what an empty scan printed (cl=)    */
#define NO_ECHO_POINTS  (5u)
#define LEAN_SAMPLES    (8u)     /* 24 mm: a 1 cm one-wheel hump        */
/* 300 ms ignore window: 14 steady 20 ms steps leave 20 ms. */
#define BLANK_STEADY_STEPS ((IMPACT_BLANK_MS / IMU_PERIOD_MS) - 1u)

/* Obstacle scan. */
#define SCAN_FIRST_DEG  (10)
#define SCAN_LAST_DEG   (170)
#define SCAN_STEP_DEG   (5)
#define NO_ECHO_MM      (9999.0f)
#define BOX_WIDTH_MM    (100.0f)
#define BOX_RANGE_MM    (150.0f)
#define WIDTH_TOLERANCE (45.0f)
#define LARGEST_BOX_MM  (150.0f) /* obstacles are 10-15 cm wide   */
#define WIDE_BOX_MM     (235.0f) /* S5-U07 box                         */
#define WIDE_BOX_OFFSET_MM (60.0f)
#define WIDE_BOX_RANGE_MM (200.0f) /* the cone hides the near edge closer */
#define NEAR_BOX_MM     (150.0f)
#define FLOOR_SCAN_POINTS (11u)
#if OBST_BLIND_LEFT
#define BLIND_ACTION    (AVOID_TURN_LEFT)
#else
#define BLIND_ACTION    (AVOID_TURN_RIGHT)
#endif

/* Command parser. */
#define GAIN_TOLERANCE (1e-6f)
#define OVERFLOW_BYTES (100u)

/* MQTT. */
#define PUBLISHES       (10u)
#define NET_UP_MS       (5000u)  /* the network comes up after boot  */
#define FIRST_WINDOW_MS (2000u)  /* the first attempt's time limit   */
#define SESSION_MS      (60000u) /* a session that ran for a while   */
#define REJOIN_AFTER_MS (3000u)  /* first re-join after the link drops */
#define REJOIN_EVERY_MS (5000u)  /* then a re-join every 5 s           */
#define RESTART_MS      (10000u) /* still down: WiFi chip restart      */
#define NOIP_REJOIN_MS  (15000u) /* joined without an IP: retry period  */
#define FAILED_STEPS    (3u)     /* failed-join retries checked         */

/* Vehicle simulation (sim/): one press of START, a 1.5 s drive (past the
   150 mm front-ignore window after "path clear") and a close echo pair. */
#define TAP_AT_MS (20u)
#define TAP_MS    (40u)
#define DRIVE_MS  (1500u)
#define NEAR_MM   (150)
#define NEARER_MM (140)
#define FAR_MM    (500)
#define CENTRE_US (1500u) /* servo pulse at 90 degrees */

typedef struct
{
    char         symbol;
    char const * p_bars; /* 9 elements, '1' = wide */
} c39_pattern_t;

/* Element widths measured (in pixels) from the course's barcode photo. */
static uint32_t const g_photo[PHOTO_ELEMENTS] = {
    15u, 47u, 15u, 15u, 48u, 15u, 47u, 15u, 15u, 15u, 47u, 15u, 15u, 16u, 14u,
    48u, 14u, 16u, 45u, 17u, 15u, 46u, 15u, 15u, 47u, 15u, 47u, 15u, 15u};

static c39_pattern_t const g_c39[] = {{'A', "100001001"},
                                      {'B', "001001001"},
                                      {'C', "101001000"},
                                      {'D', "000011001"},
                                      {'E', "100011000"},
                                      {'F', "001011000"},
                                      {'G', "000001101"},
                                      {'H', "100001100"},
                                      {'I', "001001100"},
                                      {'J', "000011100"},
                                      {'K', "100000011"},
                                      {'L', "001000011"},
                                      {'M', "101000010"},
                                      {'N', "000010011"},
                                      {'O', "100010010"},
                                      {'P', "001010010"},
                                      {'Q', "000000111"},
                                      {'R', "100000110"},
                                      {'S', "001000110"},
                                      {'T', "000010110"},
                                      {'U', "110000001"},
                                      {'V', "011000001"},
                                      {'W', "111000000"},
                                      {'X', "010010001"},
                                      {'Y', "110010000"},
                                      {'Z', "011010000"},
                                      {'*', "010010100"}};

/* TEST_BARCODE over the 14 cm *B* at 100 mm/s (24 Oct), widths in ms at
   the 35 % / 20 % levels: narrow bars 10-20 ms, narrow spaces up to 60,
   wide bars 80, so the 3 widest elements mix bars and spaces. */
static uint32_t const g_car_trace[PHOTO_ELEMENTS] = {
    20u, 100u, 10u, 60u, 80u, 40u, 80u, 60u, 10u, 60u, 10u, 40u, 90u, 50u, 20u,
    110u, 20u, 60u, 60u, 60u, 20u, 100u, 20u, 60u, 80u, 40u, 100u, 40u, 20u};

/* MISSION line following over the 14 cm *A* at 100 mm/s (24 Oct, ms): the
   first narrow bars are missed, so 23-27 elements instead of 29. */
static uint32_t const g_drive_27a[] = {
    13u, 207u, 86u, 64u,  82u, 59u, 9u,   56u, 88u, 56u, 10u, 57u, 17u, 114u,
    21u, 55u,  104u, 41u, 22u, 135u, 21u, 49u, 94u, 49u, 109u, 52u, 18u};
static uint32_t const g_drive_25[] = {
    115u, 66u, 86u, 56u, 8u,   63u, 98u, 58u, 14u, 54u, 19u, 134u, 29u,
    46u,  97u, 52u, 21u, 126u, 18u, 58u, 96u, 57u, 94u, 56u, 12u};
static uint32_t const g_drive_27b[] = {
    20u, 229u, 106u, 60u, 85u, 68u, 11u, 79u, 86u, 57u, 11u, 49u, 20u, 121u,
    21u, 49u,  93u,  58u, 25u, 122u, 19u, 46u, 100u, 49u, 88u, 50u, 17u};
static uint32_t const g_drive_23[] = {
    86u, 87u, 94u, 139u, 98u,  59u, 14u, 55u, 16u, 196u, 19u, 51u,
    93u, 66u, 13u, 130u, 15u,  61u, 104u, 57u, 87u, 61u, 6u};

/* MISSION over the *B* (24 Oct, ms): start '*' and B complete, the stop
   '*' lost its first two narrow bars (the 249 merges gap and spaces). */
static uint32_t const g_drive_b25[] = {
    6u,  122u, 7u,  58u, 87u,  57u, 95u, 58u, 18u, 59u, 16u, 54u, 80u,
    57u, 10u, 130u, 6u, 65u, 89u, 249u, 77u, 51u, 80u, 59u, 10u};

/* TEST_ULTRASONIC scan, 65..115 deg: box right, floor on the left. */
static int16_t const g_floor_scan_mm[FLOOR_SCAN_POINTS] = {
    173, 160, 160, 173, 173, 173, 186, 224, 369, 364, 301};
static uint32_t g_pass = 0u;
static uint32_t g_fail = 0u;

static void check(bool b_ok, char const * p_what);
static bool nearly(float32_t value, float32_t expected, float32_t tolerance);
static float64_t    wide(float32_t value);
static float32_t    random_unit(void);
static void         test_math(void);
static void         test_pid(void);
static void         test_line(void);
static char const * find_bars(char symbol);
static void         test_barcode_bias(void);
static void         test_barcode_lost_start(void);
static uint32_t     make_frame(char data, uint32_t * p_widths, float32_t noise,
                               float32_t ramp);
static void reverse(uint32_t const * p_src, uint32_t * p_dst, uint32_t count);
static void test_barcode(void);
static float32_t simulate_hump(float32_t height, float32_t length,
                               float32_t noise_deg, uint32_t * p_humps);
static void      test_terrain(void);
static uint32_t  scan_box(avoidance_point_t * p_points, float32_t centre_x,
                          float32_t range_y, float32_t width,
                          float32_t wall_left, int32_t first,
                          int32_t last);
static void      test_obstacle(void);
static void      test_bridge(void);
static void      test_mqtt_client(void);
static void      test_rejoin(struct netif * p_netif);
static void      test_noip_rejoin(struct netif * p_netif);
static void      test_failed_join(struct netif * p_netif);
static uint32_t  rejoins(void);
static uint32_t  restarts(void);
static uint32_t  feed_text(command_line_t * p_line, char const * p_text,
                           char * p_out);
static void      test_command(void);
static void      test_command_limits(void);
static void      test_line_gains(void);
static vehicle_status_t vehicle_status(void);
static bool             vehicle_is(vehicle_state_t state, char const * p_why);
static void             vehicle_start(uint32_t drive_ms);
static void             vehicle_close_echo(void);
static void             test_vehicle_buttons(void);
static void             test_vehicle_obstacle(void);
static void             test_vehicle_scan_stop(void);
static void             test_vehicle_manoeuvre_stop(void);
static void             test_vehicle_sonar_health(void);
static void             test_servo_limits(void);
static void             test_vehicle_barcode_watchdog(void);

/*!
 * @brief Run every test group and print the summary.
 *
 * @return 0 if every check passed, otherwise 1.
 */
int
main (void)
{
    test_math();
    test_pid();
    test_line();
    test_barcode();
    test_terrain();
    test_obstacle();
    test_vehicle_buttons(); /* the vehicle tests share one car, in order */
    test_vehicle_obstacle();
    test_vehicle_scan_stop();
    test_vehicle_manoeuvre_stop();
    test_vehicle_sonar_health();
    test_vehicle_barcode_watchdog();
    test_bridge();
    test_mqtt_client();
    test_command();
    test_command_limits();
    test_line_gains();
    test_data(check);
    (void) printf("\n%" PRIu32 " passed, %" PRIu32 " failed\n", g_pass, g_fail);

    return ((0u == g_fail) ? 0 : 1);
}

/*!
 * @brief Count one check; print its name if it failed.
 *
 * @param[in] b_ok   Outcome.
 * @param[in] p_what Description.
 */
static void
check (bool b_ok, char const * p_what)
{
    if (b_ok)
    {
        g_pass++;
    }
    else
    {
        g_fail++;
        (void) printf("  FAIL: %s\n", p_what);
    }
}

/*!
 * @brief Floating-point comparison with a tolerance (Rule 5.4.b.iv).
 *
 * @param[in] value     Value under test.
 * @param[in] expected  Expected value.
 * @param[in] tolerance Largest accepted difference.
 *
 * @return true if |value - expected| <= tolerance.
 */
static bool
nearly (float32_t value, float32_t expected, float32_t tolerance)
{
    return (fabsf(value - expected) <= tolerance);
}

/*!
 * @brief A float32_t for printing with %f.
 *
 * @param[in] value Value.
 *
 * @return The same value as float64_t.
 */
static float64_t
wide (float32_t value)
{
    /* Cast: float to double conversion is exact. */
    return ((float64_t) value);
}

/*!
 * @brief Pseudo-random number, uniform in [-1, 1].
 *
 * @return The number.
 */
static float32_t
random_unit (void)
{
    /* Casts: rand() is 0..RAND_MAX; the float conversion only rounds. */
    return ((((float32_t) rand() / (float32_t) RAND_MAX) * 2.0f) - 1.0f);
}

/*!
 * @brief Accuracy of the maths helpers against the C library.
 */
static void
test_math (void)
{
    float32_t err_sqrt = 0.0f;
    float32_t err_atan = 0.0f;
    float32_t err_sin  = 0.0f;
    float32_t err      = 0.0f;
    float32_t value    = SQRT_FIRST;
    float32_t angle    = ANGLE_FIRST;

    for (value = SQRT_FIRST; value < SQRT_LAST; value *= SQRT_STEP)
    {
        err      = fabsf(car_math_sqrt(value) - sqrtf(value)) / sqrtf(value);
        err_sqrt = (err > err_sqrt) ? err : err_sqrt;
    }

    for (angle = ANGLE_FIRST; angle < ANGLE_LAST; angle += ANGLE_STEP)
    {
        err = fabsf(car_math_atan2(sinf(angle), cosf(angle)) - angle)
              * DEG_PER_RAD;
        err_atan = (err > err_atan) ? err : err_atan;
        err = fabsf(car_math_sin(angle * SIN_SCALE) - sinf(angle * SIN_SCALE));
        err_sin = (err > err_sin) ? err : err_sin;
    }

    (void) printf("math: sqrt rel err %.2e, atan2 err %.4f deg, "
                  "sin err %.2e\n",
                  wide(err_sqrt), wide(err_atan), wide(err_sin));
    check(err_sqrt < SQRT_TOLERANCE, "sqrt");
    check(err_atan < ATAN_TOLERANCE, "atan2");
    check(err_sin < SIN_TOLERANCE, "sin");
    check(!car_math_is_finite(sqrtf(-1.0f)), "NaN is not finite");
    check(0 == car_math_round(car_math_sin(sqrtf(-1.0f))),
          "non-finite input gives 0");
    check(nearly(car_math_sin(LARGE_ANGLE), sinf(LARGE_ANGLE), SIN_REDUCED),
          "whole turns removed in one step");
    check(0 == car_math_round(car_math_sin(HUGE_ANGLE)),
          "huge angle gives 0 instead of looping");
}

/*!
 * @brief Speed PID on a first-order motor model: settling, overshoot and
 *        anti-windup.
 */
static void
test_pid (void)
{
    pid_ctrl_t pid;
    float32_t  speed     = 0.0f;
    float32_t  duty      = 0.0f;
    float32_t  drive     = 0.0f;
    float32_t  time_s    = 0.0f;
    float32_t  overshoot = 0.0f;
    uint32_t   step      = 0u;

    pid_init(&pid, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_KF, 0.0f, PID_PCT_MAX);

    for (time_s = 0.0f; time_s < PID_RUN_S; time_s += PID_DT_S)
    {
        duty = pid_update(&pid, PID_TARGET, speed, PID_DT_S) + SPEED_OFFSET_PCT;
        duty = (duty > PID_PCT_MAX) ? PID_PCT_MAX : duty;
        drive = (duty > SPEED_OFFSET_PCT) ? (duty - SPEED_OFFSET_PCT) : 0.0f;
        speed += (((MOTOR_GAIN * drive) - speed) * PID_DT_S) / MOTOR_TAU_S;
        overshoot = ((speed - PID_TARGET) > overshoot) ? (speed - PID_TARGET)
                                                       : overshoot;
    }

    (void) printf("pid: speed after 2 s = %.1f mm/s (target %.0f), "
                  "overshoot %.1f\n",
                  wide(speed), wide(PID_TARGET), wide(overshoot));
    check(nearly(speed, PID_TARGET, SETTLE_FRACTION * PID_TARGET),
          "pid settles within 5 %");
    check(overshoot < (OVERSHOOT_LIMIT * PID_TARGET), "overshoot < 15 %");

    /* Anti-windup: long saturation must not leave a huge integral. */
    pid_reset(&pid);

    for (step = 0u; step < WINDUP_STEPS; step++)
    {
        (void) pid_update(&pid, WINDUP_SETPOINT, 0.0f, PID_DT_S);
    }

    check(pid.integ <= PID_PCT_MAX, "integral clamped");
    check(nearly(pid_update(&pid, sqrtf(-1.0f), 0.0f, PID_DT_S), 0.0f, 0.0f),
          "non-finite set-point gives 0");
}

/*!
 * @brief Line follower: steering sign, line lost and junction filters.
 */
static void
test_line (void)
{
    line_follow_t follower;
    float32_t     steer = 0.0f;
    uint32_t      idx   = 0u;

    line_follow_init(&follower);
    check(LINE_ON
              == line_follow_classify(&follower, 0.5f, 0.5f, 5.0f, LINE_DT_MS),
          "on");
    steer = line_follow_steer(&follower, 0.9f, 0.1f, LINE_DT_S);
    check(steer > 0.0f, "line under LEFT sensor: steer left (s > 0)");

    line_follow_init(&follower);
    steer = line_follow_steer(&follower, 0.1f, 0.9f, LINE_DT_S);
    check(steer < 0.0f, "line under RIGHT sensor: steer right");

    line_follow_init(&follower);

    for (idx = 0u; idx < LOST_SAMPLES; idx++)
    {
        (void) line_follow_classify(&follower, 0.05f, 0.05f, 3.0f, LINE_DT_MS);
    }

    check(LINE_LOST == follower.state, "lost after 200 ms of white");

    line_follow_init(&follower);
    check(LINE_ON
              == line_follow_classify(&follower, 0.9f, 0.9f, 4.0f, LINE_DT_MS),
          "single junction sample ignored");

    for (idx = 0u; idx < JUNCTION_EXTRA; idx++)
    {
        (void) line_follow_classify(&follower, 0.9f, 0.9f, 4.0f, LINE_DT_MS);
    }

    check(LINE_JUNCTION == follower.state, "junction after 10 mm");

    /* Flicker off black for 40 mm, then black again: same junction. */
    for (idx = 0u; idx < FLICKER_SAMPLES; idx++)
    {
        (void) line_follow_classify(&follower, 0.5f, 0.5f, 4.0f, LINE_DT_MS);
    }

    for (idx = 0u; idx <= JUNCTION_EXTRA; idx++)
    {
        (void) line_follow_classify(&follower, 0.9f, 0.9f, 4.0f, LINE_DT_MS);
    }

    check(LINE_ON == follower.state, "junction reported once");

    /* 100 mm off black re-arms it. */
    for (idx = 0u; idx < REARM_SAMPLES; idx++)
    {
        (void) line_follow_classify(&follower, 0.5f, 0.5f, 4.0f, LINE_DT_MS);
    }

    for (idx = 0u; idx <= JUNCTION_EXTRA; idx++)
    {
        (void) line_follow_classify(&follower, 0.9f, 0.9f, 4.0f, LINE_DT_MS);
    }

    check(LINE_JUNCTION == follower.state, "next junction after 100 mm");
}

/*!
 * @brief Code 39 bar pattern of a symbol used by the tests.
 *
 * @param[in] symbol 'A'..'Z' or '*'.
 *
 * @return Nine characters, '1' = wide (narrow pattern if unknown).
 */
static char const *
find_bars (char symbol)
{
    char const * p_bars = "000000000";
    uint32_t     idx    = 0u;

    for (idx = 0u; idx < (sizeof(g_c39) / sizeof(g_c39[0])); idx++)
    {
        if (symbol == g_c39[idx].symbol)
        {
            p_bars = g_c39[idx].p_bars;
        }
    }

    return (p_bars);
}

/*!
 * @brief Synthesise the element widths of "*<data>*" in microseconds.
 *
 * @param[in]  data     Data character.
 * @param[out] p_widths Element widths (at least FRAME_MAX).
 * @param[in]  noise    Relative width noise, e.g. 0.2 for +-20 %.
 * @param[in]  ramp     Speed factor applied after every element.
 *
 * @return Number of widths (29).
 */
static uint32_t
make_frame (char data, uint32_t * p_widths, float32_t noise, float32_t ramp)
{
    char const   sequence[CHARS_PER_FRAME] = {'*', data, '*'};
    char const * p_bars                    = NULL;
    uint32_t     count                     = 0u;
    uint32_t     chr                       = 0u;
    uint32_t     element                   = 0u;
    float32_t    speed                     = 1.0f;
    float32_t    width_mm                  = 0.0f;
    float32_t    jitter                    = 0.0f;

    for (chr = 0u; chr < CHARS_PER_FRAME; chr++)
    {
        p_bars = find_bars(sequence[chr]);

        for (element = 0u; element < BARCODE_ELEMENTS_PER_CHAR; element++)
        {
            width_mm = ('1' == p_bars[element]) ? WIDE_MM : NARROW_MM;
            jitter   = 1.0f + (noise * random_unit());

            /* Cast: a positive width of a few tens of milliseconds. */
            p_widths[count] =
                (uint32_t) ((width_mm * jitter * US_PER_MM) / speed);
            count++;
            speed *= ramp;
        }

        /* Inter-character gap (cast as above). */
        if ((chr + 1u) < CHARS_PER_FRAME)
        {
            p_widths[count] = (uint32_t) ((NARROW_MM * US_PER_MM) / speed);
            count++;
        }
    }

    return (count);
}

/*!
 * @brief Copy widths in reverse order.
 *
 * @param[in]  p_src Source.
 * @param[out] p_dst Destination (not overlapping p_src).
 * @param[in]  count Number of widths.
 */
static void
reverse (uint32_t const * p_src, uint32_t * p_dst, uint32_t count)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < count; idx++)
    {
        p_dst[idx] = p_src[count - 1u - idx];
    }
}

/*!
 * @brief Code 39 decoder: the course photo, 1000 noisy synthetic frames
 *        in both directions, command mapping and rejects.
 */
static void
test_barcode (void)
{
    char const commands[] = {'A', 'B', 'C', 'D'};
    char       text[TEXT_SIZE];
    uint32_t   widths[FRAME_MAX];
    uint32_t   reversed[FRAME_MAX];
    uint32_t   flat[PHOTO_ELEMENTS];
    uint32_t   count   = 0u;
    uint32_t   trial   = 0u;
    uint32_t   cmd     = 0u;
    uint32_t   decoded = 0u;
    uint32_t   total   = 0u;
    int32_t    result  = 0;
    float32_t  ramp    = 1.0f;

    result = barcode_decode_frame(g_photo, PHOTO_ELEMENTS, text, TEXT_SIZE);
    (void) printf(
        "barcode: course photo decodes to '%s' -> %s\n",
        (result > 0) ? text : "?",
        barcode_decode_command_name(barcode_decode_to_command(text[0])));
    check((1 == result) && ('A' == text[0]), "photo is *A*");
    reverse(g_photo, reversed, PHOTO_ELEMENTS);
    result = barcode_decode_frame(reversed, PHOTO_ELEMENTS, text, TEXT_SIZE);
    check((1 == result) && ('A' == text[0]), "photo read backwards is A");

    srand(1u);

    for (trial = 0u; trial < TRIALS; trial++)
    {
        for (cmd = 0u; cmd < sizeof(commands); cmd++)
        {
            /* +-20 % width noise and -1, 0 or +1 % speed drift per
               element; every other frame is read backwards.  Cast: the
               drift index is 0..2, exact in float32_t. */
            ramp =
                1.0f
                + (DRIFT_PER_EL * ((float32_t) (trial % DRIFT_KINDS) - 1.0f));
            count = make_frame(commands[cmd], widths, WIDTH_NOISE, ramp);

            if (0u != (trial % 2u))
            {
                reverse(widths, reversed, count);
                (void) memcpy(widths, reversed, count * sizeof(uint32_t));
            }

            result = barcode_decode_frame(widths, count, text, TEXT_SIZE);
            total++;
            decoded += ((1 == result) && (commands[cmd] == text[0])) ? 1u : 0u;
        }
    }

    (void) printf("barcode: %" PRIu32 "/%" PRIu32 " synthetic frames decoded "
                  "(20%% noise, speed drift, both directions)\n",
                  decoded, total);
    check(decoded == total, "all synthetic frames decode");
    check(NAV_LEFT == barcode_decode_to_command(BARCODE_LEFT_SYMBOL), "left");
    check(NAV_RIGHT == barcode_decode_to_command(BARCODE_RIGHT_SYMBOL),
          "right");
    check(NAV_STRAIGHT == barcode_decode_to_command(BARCODE_STRAIGHT_SYMBOL),
          "straight");
    check(NAV_UTURN == barcode_decode_to_command(BARCODE_UTURN_SYMBOL),
          "u-turn");
    check(NAV_NONE == barcode_decode_to_command('*'), "* is no command");
    barcode_decode_set_symbol(NAV_LEFT, 'K');
    check((NAV_LEFT == barcode_decode_to_command('K'))
              && (NAV_NONE == barcode_decode_to_command(BARCODE_LEFT_SYMBOL))
              && ('K' == barcode_decode_symbol(NAV_LEFT)),
          "map: demo letter K now turns left, the old one does nothing");
    barcode_decode_set_symbol(NAV_LEFT, BARCODE_LEFT_SYMBOL);

    for (count = 0u; count < PHOTO_ELEMENTS; count++)
    {
        flat[count] = FLAT_WIDTH;
    }

    check(barcode_decode_frame(flat, PHOTO_ELEMENTS, text, TEXT_SIZE) < 0,
          "flat widths rejected");
    check(barcode_decode_frame(g_photo, SHORT_ELEMENTS, text, TEXT_SIZE) < 0,
          "short frame rejected");
    test_barcode_bias();
}

/*!
 * @brief Barcode read by the car: bars thinner and spaces wider than
 *        printed.  The car trace, then A..Z with a 2 mm bias.
 */
static void
test_barcode_bias (void)
{
    char      text[TEXT_SIZE];
    uint32_t  widths[FRAME_MAX];
    uint32_t  reversed[FRAME_MAX];
    uint32_t  count   = 0u;
    uint32_t  idx     = 0u;
    uint32_t  trial   = 0u;
    uint32_t  decoded = 0u;
    uint32_t  total   = 0u;
    int32_t   result  = 0;
    char      letter  = 'A';

    result = barcode_decode_frame(g_car_trace, PHOTO_ELEMENTS, text, TEXT_SIZE);
    check((1 == result) && ('B' == text[0]), "car trace of the 14 cm *B*");
    reverse(g_car_trace, reversed, PHOTO_ELEMENTS);
    result = barcode_decode_frame(reversed, PHOTO_ELEMENTS, text, TEXT_SIZE);
    check((1 == result) && ('B' == text[0]), "car trace read backwards");

    for (letter = 'A'; letter <= 'Z'; letter++)
    {
        for (trial = 0u; trial < BIAS_TRIALS; trial++)
        {
            count = make_frame(letter, widths, BIAS_NOISE, 1.0f);

            /* Even elements are bars.  Casts: the bias is 10 000 us. */
            for (idx = 0u; idx < count; idx++)
            {
                widths[idx] = (0u == (idx % 2u))
                                  ? (widths[idx] - (uint32_t) (BIAS_MM * US_PER_MM))
                                  : (widths[idx] + (uint32_t) (BIAS_MM * US_PER_MM));
            }

            if (0u != (trial % 2u))
            {
                reverse(widths, reversed, count);
                (void) memcpy(widths, reversed, count * sizeof(uint32_t));
            }

            result = barcode_decode_frame(widths, count, text, TEXT_SIZE);
            total++;
            decoded += ((1 == result) && (letter == text[0])) ? 1u : 0u;
        }
    }

    (void) printf("barcode: %" PRIu32 "/%" PRIu32 " A-Z frames decoded "
                  "(bars -2 mm, spaces +2 mm, 10%% noise)\n",
                  decoded, total);
    check(decoded == total, "A-Z decode with thin bars and wide spaces");
    test_barcode_lost_start();
}

/*!
 * @brief Frames missing their first elements (MISSION, 24 Oct): the car's
 *        four frames, then A..Z with 1-10 leading elements dropped in both
 *        directions.  A frame cut at the end must never give a letter.
 */
static void
test_barcode_lost_start (void)
{
    char     text[TEXT_SIZE];
    uint32_t widths[FRAME_MAX];
    uint32_t reversed[FRAME_MAX];
    uint32_t count   = 0u;
    uint32_t drop    = 0u;
    uint32_t decoded = 0u;
    uint32_t total   = 0u;
    uint32_t wrong   = 0u;
    int32_t  result  = 0;
    char     letter  = 'A';

    result = barcode_decode_frame(g_drive_27a, (uint32_t) ARRAY_LEN(g_drive_27a),
                                  text, TEXT_SIZE);
    check((1 == result) && ('A' == text[0]), "drive frame 27a is A");
    result = barcode_decode_frame(g_drive_25, (uint32_t) ARRAY_LEN(g_drive_25),
                                  text, TEXT_SIZE);
    check((1 == result) && ('A' == text[0]), "drive frame 25 is A");
    result = barcode_decode_frame(g_drive_27b, (uint32_t) ARRAY_LEN(g_drive_27b),
                                  text, TEXT_SIZE);
    check((1 == result) && ('A' == text[0]), "drive frame 27b is A");
    result = barcode_decode_frame(g_drive_23, (uint32_t) ARRAY_LEN(g_drive_23),
                                  text, TEXT_SIZE);
    check((1 == result) && ('A' == text[0]), "drive frame 23 is A");
    result = barcode_decode_frame(g_drive_b25, (uint32_t) ARRAY_LEN(g_drive_b25),
                                  text, TEXT_SIZE);
    check((1 == result) && ('B' == text[0]), "drive frame with a broken end is B");
    reverse(g_drive_b25, reversed, (uint32_t) ARRAY_LEN(g_drive_b25));
    result = barcode_decode_frame(reversed, (uint32_t) ARRAY_LEN(g_drive_b25),
                                  text, TEXT_SIZE);
    check((1 == result) && ('B' == text[0]), "same frame crossed backwards is B");

    for (letter = 'A'; letter <= 'Z'; letter++)
    {
        for (drop = 1u; drop <= (BARCODE_ELEMENTS_PER_CHAR + 1u); drop++)
        {
            count = make_frame(letter, widths, BIAS_NOISE, 1.0f);

            /* Read forwards, then crossed the other way round; either way
               the first elements the car meets are lost. */
            result = barcode_decode_frame(&widths[drop], count - drop, text,
                                          TEXT_SIZE);
            total++;
            decoded += ((1 == result) && (letter == text[0])) ? 1u : 0u;
            reverse(widths, reversed, count);
            result = barcode_decode_frame(&reversed[drop], count - drop, text,
                                          TEXT_SIZE);
            total++;
            decoded += ((1 == result) && (letter == text[0])) ? 1u : 0u;

            /* Cut at the end instead: no letter, or the right one. */
            result = barcode_decode_frame(widths, count - drop, text, TEXT_SIZE);
            wrong += ((result > 0) && (letter != text[0])) ? 1u : 0u;
        }
    }

    (void) printf("barcode: %" PRIu32 "/%" PRIu32 " frames missing 1-10 first "
                  "elements decoded, %" PRIu32 " wrong when cut at the end\n",
                  decoded, total, wrong);
    check(decoded == total, "A-Z decode with the first bars lost");
    check(0u == wrong, "frames cut at the end never give a wrong letter");
}

/*!
 * @brief Drive over a raised-cosine hump with noisy pitch readings.
 *
 * @param[in]  height    Hump height in mm.
 * @param[in]  length    Hump length in mm.
 * @param[in]  noise_deg Pitch noise amplitude in degrees.
 * @param[out] p_humps   Humps counted.
 *
 * @return Highest estimated peak in mm.
 */
static float32_t
simulate_hump (float32_t height, float32_t length, float32_t noise_deg,
               uint32_t * p_humps)
{
    terrain_hump_t hump;
    float32_t      step_mm = HUMP_SPEED_MM_S * PID_DT_S; /* 50 Hz */
    float32_t      pos_mm  = -HUMP_LEAD_MM;
    float32_t      slope   = 0.0f;
    float32_t      theta   = 0.0f;
    float32_t      pitch   = 0.0f;

    terrain_hump_init(&hump);

    while (pos_mm < (length + HUMP_TAIL_MM))
    {
        slope = 0.0f;

        if ((pos_mm > 0.0f) && (pos_mm < length))
        {
            slope = ((height * PI_F) / length)
                    * sinf((2.0f * PI_F * pos_mm) / length);
        }

        theta = atanf(slope);
        pitch = (theta * DEG_PER_RAD) + (noise_deg * random_unit());

        /* The encoders measure the arc length. */
        terrain_hump_update(&hump, pitch, 0.0f, step_mm);
        pos_mm += step_mm * cosf(theta);
    }

    *p_humps = hump.humps;

    return (hump.max_peak_mm);
}

/*!
 * @brief Impact ignore time left after a duty jump and then count steps
 *        of steady duty.
 *
 * @param[in] count IMU steps after the jump.
 *
 * @return Time left, ms.
 */
static uint32_t
blank_steps (uint32_t count)
{
    uint32_t blank_ms = terrain_impact_blank(0u, 50.0f);
    uint32_t step     = 0u;

    for (step = 0u; step < count; step++)
    {
        blank_ms = terrain_impact_blank(blank_ms, 0.0f);
    }

    return (blank_ms);
}

/*!
 * @brief Drive one wheel over a raised-cosine hump (the other stays on
 *        the floor): the car rolls by atan(z / lever) and dips its nose
 *        by atan(z / 2 caster), with noisy readings.
 *
 * @param[in]  height    Hump height in mm.
 * @param[out] p_humps   Humps counted.
 * @param[out] p_changes CLIMBING / DESCENDING switches (1 = no flicker).
 *
 * @return Highest estimated peak in mm.
 */
static float32_t
simulate_one_wheel (float32_t height, uint32_t * p_humps, uint32_t * p_changes)
{
    terrain_hump_t       hump;
    terrain_hump_state_t last = HUMP_FLAT;
    float32_t      step_mm = HUMP_SPEED_MM_S * PID_DT_S; /* 50 Hz */
    float32_t      pos_mm  = -HUMP_LEAD_MM;
    float32_t      wheel_z = 0.0f;
    float32_t      roll    = 0.0f;
    float32_t      pitch   = 0.0f;

    terrain_hump_init(&hump);
    *p_changes = 0u;

    while (pos_mm < (HUMP_LENGTH_MM + HUMP_TAIL_MM))
    {
        wheel_z = 0.0f;

        if ((pos_mm > 0.0f) && (pos_mm < HUMP_LENGTH_MM))
        {
            wheel_z = 0.5f * height
                      * (1.0f - cosf((2.0f * PI_F * pos_mm) / HUMP_LENGTH_MM));
        }

        roll  = (atanf(wheel_z / HUMP_ROLL_LEVER_MM) * DEG_PER_RAD)
               + (ROLL_NOISE_DEG * random_unit());
        pitch = (-atanf(wheel_z / (2.0f * CASTER_MM)) * DEG_PER_RAD)
                + (PITCH_NOISE_DEG * random_unit());
        terrain_hump_update(&hump, pitch, roll, step_mm);
        *p_changes += ((HUMP_FLAT != last) && (HUMP_FLAT != hump.state)
                       && (last != hump.state))
                          ? 1u
                          : 0u;
        last    = hump.state;
        pos_mm += step_mm;
    }

    *p_humps = hump.humps;

    return (hump.max_peak_mm);
}

/*!
 * @brief Roll on the flat with a roll step of roll_deg for count samples
 *        (3 mm each), then level again.
 *
 * @param[in] roll_deg Roll during the step.
 * @param[in] count    Samples in the step.
 *
 * @return Humps counted.
 */
static uint32_t
roll_step_humps (float32_t roll_deg, uint32_t count)
{
    terrain_hump_t hump;
    uint32_t       idx = 0u;

    terrain_hump_init(&hump);

    for (idx = 0u; idx < LIFT_SAMPLES; idx++)
    {
        terrain_hump_update(&hump, 0.0f,
                            ((idx >= JOLT_FIRST) && (idx < (JOLT_FIRST + count)))
                                ? roll_deg
                                : 0.0f,
                            HUMP_SPEED_MM_S * PID_DT_S);
    }

    return (hump.humps);
}

/*!
 * @brief Stop on a one-wheel hump (5 deg roll) and tilt the car 15 deg
 *        sideways by hand: the peak must not change.
 *
 * @return true if the peak stayed at the rolling value.
 */
static bool
lift_on_hump_holds_peak (void)
{
    terrain_hump_t hump;
    float32_t      peak = 0.0f;
    uint32_t       idx  = 0u;

    terrain_hump_init(&hump);

    for (idx = 0u; idx < (LIFT_SAMPLES / 2u); idx++)
    {
        terrain_hump_update(&hump, 0.0f, 5.0f, HUMP_SPEED_MM_S * PID_DT_S);
    }

    peak = hump.peak_mm;

    for (idx = 0u; idx < (LIFT_SAMPLES / 2u); idx++)
    {
        terrain_hump_update(&hump, 0.0f, 15.0f, 0.0f);
    }

    return ((HUMP_FLAT != hump.state) && nearly(hump.peak_mm, peak, 0.01f));
}

/*!
 * @brief Lift the front 15 deg and the back 15 deg with the wheels still,
 *        one side tilted 10 deg, then roll on the flat: the tracker must
 *        never leave HUMP_FLAT.
 *
 * @return true if it stayed flat and counted nothing.
 */
static bool
lifted_stays_flat (void)
{
    terrain_hump_t hump;
    bool           b_flat = true;
    uint32_t       idx    = 0u;

    terrain_hump_init(&hump);

    for (idx = 0u; idx < LIFT_SAMPLES; idx++)
    {
        terrain_hump_update(&hump, (idx < (LIFT_SAMPLES / 2u)) ? 15.0f : -15.0f,
                            10.0f, 0.0f);
        b_flat = b_flat && (HUMP_FLAT == hump.state);
    }

    for (idx = 0u; idx < LIFT_SAMPLES; idx++)
    {
        terrain_hump_update(&hump, 0.0f, 0.0f, HUMP_SPEED_MM_S * PID_DT_S);
        b_flat = b_flat && (HUMP_FLAT == hump.state);
    }

    return (b_flat && (0u == hump.humps));
}

/*!
 * @brief Hump height estimation and motion-event classification.
 */
static void
test_terrain (void)
{
    float32_t const heights[HEIGHT_COUNT] = {20.0f, 30.0f, 40.0f, 60.0f};
    float32_t const wheel_heights[WHEEL_COUNT] = {10.0f, 20.0f};
    uint32_t        humps                 = 0u;
    uint32_t        changes               = 0u;
    uint32_t        idx                   = 0u;
    float32_t       estimate              = 0.0f;

    srand(2u);
    (void) printf("terrain: true height -> estimated peak "
                  "(1.0 deg pitch noise)\n");

    for (idx = 0u; idx < HEIGHT_COUNT; idx++)
    {
        estimate = simulate_hump(heights[idx], HUMP_LENGTH_MM, PITCH_NOISE_DEG,
                                 &humps);
        (void) printf("   %4.0f mm -> %5.1f mm (%+.1f%%), humps=%" PRIu32 "\n",
                      wide(heights[idx]), wide(estimate),
                      wide((100.0f * (estimate - heights[idx])) / heights[idx]),
                      humps);
        check(1u == humps, "one hump detected");
        check(nearly(estimate, heights[idx],
                     (HEIGHT_FRACTION * heights[idx]) + HEIGHT_SLACK_MM),
              "peak within 10 %");
    }

    (void) printf("terrain: one-wheel hump, true height -> estimated peak\n");

    for (idx = 0u; idx < WHEEL_COUNT; idx++)
    {
        estimate = simulate_one_wheel(wheel_heights[idx], &humps, &changes);
        (void) printf("   %4.0f mm -> %5.1f mm, humps=%" PRIu32
                      ", state changes=%" PRIu32 "\n",
                      wide(wheel_heights[idx]), wide(estimate), humps, changes);
        check(1u == humps, "one-wheel hump detected");
        check(1u == changes, "climbing then descending, no flicker");
        check(nearly(estimate, wheel_heights[idx],
                     (HEIGHT_FRACTION * wheel_heights[idx]) + HEIGHT_SLACK_MM),
              "one-wheel peak within 10 %");
    }

    (void) simulate_hump(0.0f, HUMP_LENGTH_MM, PITCH_NOISE_DEG, &humps);
    check(0u == humps, "flat floor with noise: no hump");
    (void) simulate_one_wheel(0.0f, &humps, &changes);
    check(0u == humps, "flat floor with roll noise: no hump");
    check(0u == roll_step_humps(7.0f, JOLT_SAMPLES),
          "15 mm roll jolt while pushing: no hump");
    check(1u == roll_step_humps(4.0f, LEAN_SAMPLES),
          "24 mm lean (1 cm one-wheel hump): one hump");
    check(lift_on_hump_holds_peak(), "lifted on a hump: peak held");
    check(lifted_stays_flat(), "tilted with the wheels still: no hump");
    check(EVT_IMPACT == terrain_classify(0.0f, 0.0f, 0.0f, HUMP_FLAT, true),
          "impact");
    check(EVT_TURNING
              == terrain_classify(150.0f, 0.0f, 90.0f, HUMP_FLAT, false),
          "turn");
    check(EVT_STATIONARY
              == terrain_classify(5.0f, 0.0f, 0.0f, HUMP_FLAT, false),
          "still");
    check(EVT_STATIONARY
              == terrain_classify(0.0f, 0.0f, 0.0f, HUMP_CLIMBING, false),
          "stopped with a hump open: still");
    check(EVT_CLIMBING
              == terrain_classify(150.0f, 0.0f, 0.0f, HUMP_CLIMBING, false),
          "rolling up a hump: climbing");
    check(EVT_TURNING
              == terrain_classify(0.0f, 0.0f, 90.0f, HUMP_DESCENDING, false),
          "pivoting with a hump open: turning");

    /* Impacts: TEST_CRASH jolts (mg) on the car's forward axis. */
    check(nearly(terrain_impact_jerk(-1014.0f, 300.0f, 200.0f, 1.0f),
                 1014.0f, 0.1f),
          "wall hit driving forward: counts");
    check(nearly(terrain_impact_jerk(1453.0f, 300.0f, 200.0f, 1.0f), 0.0f,
                 0.1f),
          "forward motor start: ignored");
    check(nearly(terrain_impact_jerk(-1797.0f, 300.0f, 200.0f, -1.0f),
                 0.0f, 0.1f),
          "reverse motor start: ignored");
    check(nearly(terrain_impact_jerk(1200.0f, 0.0f, 0.0f, -1.0f), 1200.0f,
                 0.1f),
          "hit while reversing: counts");
    check(nearly(terrain_impact_jerk(300.0f, 400.0f, 0.0f, 0.0f), 500.0f,
                 0.1f),
          "parked: any jolt counts");
    check(nearly(terrain_travel_sign(70.0f, 0.0f, 0.0f), 1.0f, 0.0f)
              && nearly(terrain_travel_sign(-50.0f, 0.0f, 1.0f), -1.0f, 0.0f),
          "driving: direction from the duty");
    check(nearly(terrain_travel_sign(0.0f, 400.0f, 1.0f), 1.0f, 0.0f)
              && nearly(terrain_travel_sign(0.0f, 5.0f, 1.0f), 0.0f, 0.0f),
          "motors off: coasting keeps the direction, stopped is parked");
    check(IMPACT_BLANK_MS == terrain_impact_blank(0u, -50.0f),
          "braking from 50 % duty: impacts ignored");
    check(IMPACT_BLANK_MS == terrain_impact_blank(0u, 70.0f),
          "motor start: impacts ignored");
    check(0u == terrain_impact_blank(0u, 5.0f),
          "steering correction: impacts still count");
    check(blank_steps(BLANK_STEADY_STEPS) > 0u,
          "just after the jump: still ignored");
    check(0u == blank_steps(BLANK_STEADY_STEPS + 1u),
          "300 ms of steady duty: impacts count again");
}

/*!
 * @brief Simulated servo scan of a box face, optionally with a wall on
 *        the left.
 *
 * @param[out] p_points  Scan points (at least SCAN_MAX_POINTS).
 * @param[in]  centre_x  Lateral centre of the box (positive = right).
 * @param[in]  range_y   Forward distance of the box face.
 * @param[in]  width     Box width.
 * @param[in]  wall_left Distance of a wall on the left, 0 for none.
 * @param[in]  first    First servo angle.
 * @param[in]  last     Last servo angle.
 *
 * @return Number of points.
 */
static uint32_t
scan_box (avoidance_point_t * p_points, float32_t centre_x, float32_t range_y,
          float32_t width, float32_t wall_left, int32_t first, int32_t last)
{
    uint32_t  count   = 0u;
    int32_t   angle   = first;
    float32_t cos_a   = 0.0f;
    float32_t sin_a   = 0.0f;
    float32_t best    = NO_ECHO_MM;
    float32_t lateral = 0.0f;
    float32_t smear   = range_y * tanf(OBST_BEAM_HALF_DEG * CAR_MATH_RAD_PER_DEG);

    for (angle = first; angle <= last; angle += SCAN_STEP_DEG)
    {
        /* Casts: angles are 10..170, exact in float32_t and int16_t; the
           distance is below US_MAX_MM or replaced by -1. */
        cos_a   = cosf((float32_t) angle * CAR_MATH_RAD_PER_DEG);
        sin_a   = sinf((float32_t) angle * CAR_MATH_RAD_PER_DEG);
        best    = NO_ECHO_MM;
        lateral = (range_y * cos_a) / sin_a;

        /* Front face of the box: y = range_y, x in centre +- width / 2,
           seen OBST_BEAM_HALF_DEG past each edge (the sonar cone). */
        if ((lateral >= (centre_x - (width / 2.0f) - smear))
            && (lateral <= (centre_x + (width / 2.0f) + smear)))
        {
            best = range_y / sin_a;
        }

        /* Wall at x = -wall_left. */
        if ((wall_left > 0.0f) && (cos_a < -0.01f)
            && ((wall_left / -cos_a) < best))
        {
            best = wall_left / -cos_a;
        }

        /* Casts as above. */
        p_points[count].angle_deg = (int16_t) angle;
        p_points[count].dist_mm =
            (best > (float32_t) US_MAX_MM) ? (int16_t) -1 : (int16_t) best;
        count++;
    }

    return (count);
}

/*!
 * @brief Obstacle profiling and avoidance planning.
 */
static void
test_obstacle (void)
{
    avoidance_point_t   points[SCAN_MAX_POINTS];
    avoidance_profile_t prof;
    avoidance_plan_t    plan;
    uint32_t            count = 0u;
    uint32_t            idx   = 0u;

    count = scan_box(points, 20.0f, BOX_RANGE_MM, BOX_WIDTH_MM, 0.0f, SCAN_FIRST_DEG,
                     SCAN_LAST_DEG);
    check(avoidance_profile(points, count, &prof), "box found");
    plan = avoidance_plan(&prof, 0u);
    (void) printf("obstacle: box w=100 at x=+20: est width %.0f, "
                  "centre %.0f, closest %" PRId32 " -> %s offset %.0f\n",
                  wide(prof.width_mm), wide(prof.centre_mm), prof.closest_mm,
                  avoidance_action_name(plan.action), wide(plan.offset_mm));
    check(nearly(prof.width_mm, BOX_WIDTH_MM, WIDTH_TOLERANCE),
          "width estimate");
    check(AVOID_TURN_LEFT == plan.action, "box right of centre: pass left");
    check(nearly(plan.pass_mm,
                 (float32_t) prof.closest_mm + OBST_DEPTH_MAX_MM
                     + CAR_LENGTH_MM + OBST_MARGIN_MM,
                 0.5f),
          "bypass: pass = closest + deepest box + car length + margin");

    count = scan_box(points, 20.0f, BOX_RANGE_MM, BOX_WIDTH_MM, 200.0f, SCAN_FIRST_DEG,
                     SCAN_LAST_DEG);
    (void) avoidance_profile(points, count, &prof);
    plan = avoidance_plan(&prof, 0u);
    (void) printf("obstacle: same box + wall 200 mm left: clear L=%" PRId32
                  " R=%" PRId32 " -> %s\n",
                  prof.clear_left_mm, prof.clear_right_mm,
                  avoidance_action_name(plan.action));
    check(AVOID_TURN_RIGHT == plan.action, "wall on the left: pass right");

    count = scan_box(points, 0.0f, 70.0f, BOX_WIDTH_MM, 0.0f, SCAN_FIRST_DEG,
                     SCAN_LAST_DEG);
    (void) avoidance_profile(points, count, &prof);
    check(AVOID_REVERSE == avoidance_plan(&prof, 0u).action,
          "too close: reverse");
    check(AVOID_STOP == avoidance_plan(&prof, OBST_MAX_ATTEMPTS).action,
          "then stop");

    /* After a REVERSE the box is often just past OBST_DETECT_MM (221 mm,
       25 Oct): still the obstacle, not a clear path. */
    count = scan_box(points, 0.0f, 221.0f, BOX_WIDTH_MM, 0.0f, SERVO_MIN_DEG,
                     SERVO_MAX_DEG);
    check(avoidance_profile(points, count, &prof)
              && (AVOID_CONTINUE != avoidance_plan(&prof, 0u).action),
          "box at 221 mm after a reverse: found, no continue");

    /* 25 Oct, box on the left: floor-range slant echoes (293) split it
       into pieces 37 mm apart; they are one box, open on the right. */
    {
        static int16_t const split[11] = {652, 679, 221, 221, 221, 293,
                                          199, 198, 293, 211, 224};

        for (idx = 0u; idx < 11u; idx++)
        {
            /* Cast: 65..115 deg fit int16_t. */
            points[idx].angle_deg = (int16_t) (SERVO_MIN_DEG + (5 * (int32_t) idx));
            points[idx].dist_mm   = split[idx];
        }

        (void) avoidance_profile(points, 11u, &prof);
        check(AVOID_TURN_RIGHT == avoidance_plan(&prof, 0u).action,
              "box split by floor echoes: one box, pass right");
    }

    /* 25 Oct, 8 cm box 60 mm left.  A flat face read 10 mm right of
       centre: side unsure, wide blind pass.  Seen only at 110..115 deg:
       pass right, but never shift less than OBST_MIN_OFFSET_MM. */
    {
        static int16_t const centred[11] = {430, 412, 293, 232, 224, 224,
                                            224, 424, 424, 424, 424};
        static int16_t const far_left[11] = {570, 584, 1186, 1182, 1181, 295,
                                             295, 295, 295, 227, 227};

        for (idx = 0u; idx < 11u; idx++)
        {
            /* Cast: 65..115 deg fit int16_t. */
            points[idx].angle_deg = (int16_t) (SERVO_MIN_DEG + (5 * (int32_t) idx));
            points[idx].dist_mm   = centred[idx];
        }

        (void) avoidance_profile(points, 11u, &prof);
        plan = avoidance_plan(&prof, 0u);
        check((AVOID_TURN_LEFT == plan.action)
                  && nearly(plan.offset_mm, OBST_BLIND_OFFSET_MM, 0.5f),
              "box leaning right near the centre: wide pass left");

        for (idx = 0u; idx < 11u; idx++)
        {
            points[idx].dist_mm = far_left[idx];
        }

        (void) avoidance_profile(points, 11u, &prof);
        plan = avoidance_plan(&prof, 0u);
        check((AVOID_TURN_RIGHT == plan.action)
                  && (plan.offset_mm >= OBST_MIN_OFFSET_MM),
              "box seen far left: pass right, at least the minimum shift");
    }

    count = scan_box(points, 400.0f, BOX_RANGE_MM, BOX_WIDTH_MM, 0.0f, SCAN_FIRST_DEG,
                     SCAN_LAST_DEG);
    (void) avoidance_profile(points, count, &prof);
    check(AVOID_CONTINUE == avoidance_plan(&prof, 0u).action,
          "outside the corridor: continue");

    /* S5-U07: a 235 mm box on the real mount (65..115 degrees). */
    count = scan_box(points, -WIDE_BOX_OFFSET_MM, WIDE_BOX_RANGE_MM, WIDE_BOX_MM,
                     0.0f, SERVO_MIN_DEG, SERVO_MAX_DEG);
    (void) avoidance_profile(points, count, &prof);
    plan = avoidance_plan(&prof, 0u);
    check(AVOID_TURN_RIGHT == plan.action,
          "wide box 60 mm left: pass right (left edge not seen)");
    check((plan.offset_mm - (CAR_WIDTH_MM / 2.0f))
              >= ((WIDE_BOX_MM / 2.0f) - WIDE_BOX_OFFSET_MM),
          "wide box 60 mm left: the car clears its right edge");
    count = scan_box(points, WIDE_BOX_OFFSET_MM, WIDE_BOX_RANGE_MM, WIDE_BOX_MM,
                     0.0f, SERVO_MIN_DEG, SERVO_MAX_DEG);
    (void) avoidance_profile(points, count, &prof);
    plan = avoidance_plan(&prof, 0u);
    check((AVOID_TURN_LEFT == plan.action)
              && ((plan.offset_mm - (CAR_WIDTH_MM / 2.0f))
                  >= ((WIDE_BOX_MM / 2.0f) - WIDE_BOX_OFFSET_MM)),
          "wide box 60 mm right: pass left, clear of its left edge");
    count = scan_box(points, 0.0f, NEAR_BOX_MM, WIDE_BOX_MM, 0.0f,
                     SERVO_MIN_DEG, SERVO_MAX_DEG);
    (void) avoidance_profile(points, count, &prof);
    plan = avoidance_plan(&prof, 0u);
    check((BLIND_ACTION == plan.action)
              && (plan.offset_mm >= OBST_BLIND_OFFSET_MM),
          "wide box past both sweep ends: blind pass on OBST_BLIND_LEFT's side");
    check((OBST_BLIND_OFFSET_MM - (CAR_WIDTH_MM / 2.0f))
              >= ((LARGEST_BOX_MM / 2.0f) + WIDE_BOX_OFFSET_MM),
          "blind pass clears a 150 mm box 60 mm off the line either way");
    count = scan_box(points, 0.0f, NEAR_BOX_MM, LARGEST_BOX_MM, 0.0f,
                     SCAN_FIRST_DEG, SCAN_LAST_DEG);
    check(avoidance_profile(points, count, &prof)
              && nearly(prof.width_mm, LARGEST_BOX_MM, WIDTH_TOLERANCE),
          "150 mm box centred at 150 mm: width estimate");

    /* Design obstacle: 100 mm box, 60 mm off the line, on the real mount:
       one edge in view, so the profile picks the free side. */
    count = scan_box(points, -WIDE_BOX_OFFSET_MM, NEAR_BOX_MM, BOX_WIDTH_MM,
                     0.0f, SERVO_MIN_DEG, SERVO_MAX_DEG);
    (void) avoidance_profile(points, count, &prof);
    plan = avoidance_plan(&prof, 0u);
    check((AVOID_TURN_RIGHT == plan.action)
              && ((plan.offset_mm - (CAR_WIDTH_MM / 2.0f))
                  >= ((BOX_WIDTH_MM / 2.0f) - WIDE_BOX_OFFSET_MM)),
          "100 mm box 60 mm left: pass right, clear of it");
    count = scan_box(points, WIDE_BOX_OFFSET_MM, NEAR_BOX_MM, BOX_WIDTH_MM,
                     0.0f, SERVO_MIN_DEG, SERVO_MAX_DEG);
    (void) avoidance_profile(points, count, &prof);
    plan = avoidance_plan(&prof, 0u);
    check((AVOID_TURN_LEFT == plan.action)
              && ((plan.offset_mm - (CAR_WIDTH_MM / 2.0f))
                  >= ((BOX_WIDTH_MM / 2.0f) - WIDE_BOX_OFFSET_MM)),
          "100 mm box 60 mm right: pass left, clear of it");

    /* Car scan of 24 Oct, box right of the line: the floor echoed at
       301-369 mm on the left and planned REVERSE.  Ignored, the left
       is free. */
    count = 0u;
    for (idx = 0u; idx < FLOOR_SCAN_POINTS; idx++)
    {
        points[count].angle_deg = (int16_t) (SERVO_MIN_DEG
                                             + ((int32_t) idx * OBST_FINE_STEP_DEG));
        points[count].dist_mm   = g_floor_scan_mm[idx];
        count++;
    }
    (void) avoidance_profile(points, count, &prof);
    check(AVOID_TURN_LEFT == avoidance_plan(&prof, 0u).action,
          "box right, floor echo on the left ignored: pass left");

    /* Nothing found: no stale edges or clearances (scans printed
       cl=70647).  Fill the profile with junk first. */
    prof.clear_left_mm  = JUNK_MM;
    prof.clear_right_mm = JUNK_MM;
    prof.closest_angle  = JUNK_MM;
    prof.left_edge_mm   = (float32_t) JUNK_MM;
    prof.right_edge_mm  = (float32_t) JUNK_MM;
    prof.width_mm       = (float32_t) JUNK_MM;
    prof.centre_mm      = (float32_t) JUNK_MM;

    for (idx = 0u; idx < NO_ECHO_POINTS; idx++)
    {
        points[idx].angle_deg = (int16_t) (SERVO_CENTRE_DEG + (int32_t) idx);
        points[idx].dist_mm   = (int16_t) -1;
    }

    check(!avoidance_profile(points, NO_ECHO_POINTS, &prof),
          "no echoes: nothing found");
    check((0 == prof.clear_left_mm) && (0 == prof.clear_right_mm)
              && (0 == prof.closest_angle)
              && nearly(prof.left_edge_mm, 0.0f, 0.0f)
              && nearly(prof.right_edge_mm, 0.0f, 0.0f)
              && nearly(prof.width_mm, 0.0f, 0.0f)
              && nearly(prof.centre_mm, 0.0f, 0.0f),
          "nothing found: edges and clearances zero");
}

/*!
 * @brief Publish and command rings between the tasks and the MQTT client.
 */
static void
test_bridge (void)
{
    static uint8_t const frame[]   = {0x14u, 0x21u, 0x00u}; /* any bytes */
    static uint8_t const command[] = {0x1Cu, 0x05u, 0x96u, 0x00u, 0x48u};
    uint8_t              big[MQTT_BRIDGE_PAYLOAD_MAX + 1u] = {0u};
    uint8_t              cmd[MQTT_BRIDGE_CMD_MAX];
    char                 data[MQTT_BRIDGE_PAYLOAD_MAX];
    mqtt_bridge_topic_t  topic  = TOPIC_TELEMETRY;
    uint32_t             len    = 0u;
    uint32_t             queued = 0u;
    uint32_t             idx    = 0u;

    check(!mqtt_bridge_publish(TOPIC_TELEMETRY, frame, sizeof(frame)),
          "dropped when offline");
    mqtt_bridge_set_link(LINK_CONNECTED);
    check(!mqtt_bridge_publish(TOPIC_TELEMETRY, big, sizeof(big)),
          "a frame longer than a slot is dropped");

    for (idx = 0u; idx < PUBLISHES; idx++)
    {
        queued += mqtt_bridge_publish(TOPIC_EVENT, frame, sizeof(frame)) ? 1u
                                                                         : 0u;
    }

    check(5u == queued, "ring holds 5 (6 slots - 1)");
    check((mqtt_bridge_peek_tx(&topic, data, sizeof(data), &len))
              && (TOPIC_EVENT == topic) && (3u == len)
              && (0x21 == (uint8_t) data[1]) && (0 == data[2]),
          "peek: topic, length and binary bytes (zeros included)");
    mqtt_bridge_drop_tx();
    mqtt_bridge_put_command(command, sizeof(command));
    check((mqtt_bridge_get_command(cmd, sizeof(cmd), &len))
              && (sizeof(command) == len) && (0 == memcmp(cmd, command, len)),
          "command frame passes through unchanged");
    check(!mqtt_bridge_get_command(cmd, sizeof(cmd), &len),
          "command ring empty");
    mqtt_bridge_put_command(big, MQTT_BRIDGE_CMD_MAX + 1u);
    check(!mqtt_bridge_get_command(cmd, sizeof(cmd), &len),
          "a command longer than a slot is dropped");
}

/*!
 * @brief MQTT connection manager (mqtt_lwip.c) against a fake lwIP: retry
 *        timing, "online" and the command subscription on connect, queue
 *        flushing and command routing.
 */
static void
test_mqtt_client (void)
{
    static uint8_t const stop[]  = {0x1Cu, 0x01u, 0x00u, 0x00u};
    static uint8_t const start[] = {0x1Cu, 0x00u, 0x00u, 0x00u};
    static uint8_t const tel[]   = {0x11u, 0x01u};
    struct netif         netif   = {0u, {0u}};
    mqtt_bridge_stats_t  stats;
    uint8_t              cmd[MQTT_BRIDGE_CMD_MAX];
    char                 data[MQTT_BRIDGE_PAYLOAD_MAX];
    mqtt_bridge_topic_t  topic = TOPIC_TELEMETRY;
    uint32_t             len   = 0u;

    /* Start from an empty publish ring (test_bridge() left messages). */
    while (mqtt_bridge_peek_tx(&topic, data, sizeof(data), &len))
    {
        mqtt_bridge_drop_tx();
    }

    g_fake_now_ms = NET_UP_MS;
    cyw43_utk_app_poll(&netif);
    check(0u == g_fake_connects, "no attempt without a network");
    mqtt_bridge_get_stats(&stats);
    check((0 == strcmp(stats.p_wifi, "joined")) && (1u == stats.wifi_code),
          "the WiFi state is reported (3-bit code)");
    g_fake_now_ms += RESTART_MS - 1u;
    cyw43_utk_app_poll(&netif);
    check((0u == rejoins()) && (0u == restarts()),
          "the first join at power-up is left to run");
    g_fake_now_ms++;
    cyw43_utk_app_poll(&netif);
    check((0u == rejoins()) && (1u == restarts()),
          "no link 10 s after power-up: WiFi chip restarted");
    netif.b_up    = 1u;
    netif.ip.addr = 1u;
    cyw43_utk_app_poll(&netif);
    cyw43_utk_app_poll(&netif);
    check(1u == g_fake_connects, "one attempt once the network is up");
    mqtt_bridge_get_stats(&stats);
    check((1u == stats.attempts) && (1u == stats.address),
          "attempt counted, own address recorded");
    g_fake_now_ms += FIRST_WINDOW_MS - 1u;
    cyw43_utk_app_poll(&netif);
    check(1u == g_fake_connects, "the attempt gets 2 s");
    mqtt_bridge_get_stats(&stats);
    check(0 == strcmp(stats.p_why, "-"), "no reason before an answer");
    g_fake_now_ms++;
    cyw43_utk_app_poll(&netif);
    check(2u == g_fake_connects, "then a new attempt");
    mqtt_bridge_get_stats(&stats);
    check(0 == strncmp(stats.p_why, "no answer", 9u),
          "an unanswered attempt is reported");
    fake_mqtt_connection(MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_);
    mqtt_bridge_get_stats(&stats);
    check((0 == strncmp(stats.p_why, "refused: not authorised", 23u))
              && (LINK_BACKOFF == stats.link),
          "a refusal is reported in words");
    g_fake_now_ms += FIRST_WINDOW_MS * 2u;
    cyw43_utk_app_poll(&netif);
    check(3u == g_fake_connects, "a refused attempt is retried");

    fake_mqtt_connection(MQTT_CONNECT_ACCEPTED);
    mqtt_bridge_get_stats(&stats);
    check(0 == strcmp(stats.p_why, "connected"), "success is reported");
    check((0 == strcmp(g_fake_topic, "picocar/t/status"))
              && (1u == g_fake_retain)
              && (0 == strcmp(g_fake_subscribed, "picocar/t/cmd")),
          "connected: online (retained) and cmd subscribed");
    check((2u == g_fake_payload_len) && (0x1Bu == g_fake_payload[0])
              && (0x01u == g_fake_payload[1]),
          "online is a 2-byte status frame");

    (void) mqtt_bridge_publish(TOPIC_TELEMETRY, tel, sizeof(tel));
    g_fake_publish_err = ERR_MEM;
    cyw43_utk_app_poll(&netif);
    check(mqtt_bridge_peek_tx(&topic, data, sizeof(data), &len),
          "kept while the client is full");
    g_fake_publish_err = ERR_OK;
    cyw43_utk_app_poll(&netif);
    check((!mqtt_bridge_peek_tx(&topic, data, sizeof(data), &len))
              && (0 == strcmp(g_fake_topic, "picocar/t/telemetry")),
          "sent once the client has room");

    fake_mqtt_incoming_bytes("picocar/t/other", start, sizeof(start));
    fake_mqtt_incoming_bytes("picocar/t/cmd", stop, sizeof(stop));
    check((mqtt_bridge_get_command(cmd, sizeof(cmd), &len))
              && (sizeof(stop) == len) && (0 == memcmp(cmd, stop, len))
              && (!mqtt_bridge_get_command(cmd, sizeof(cmd), &len)),
          "only the cmd topic gives commands");

    g_fake_now_ms += SESSION_MS;
    fake_mqtt_connection(MQTT_CONNECT_DISCONNECTED);
    cyw43_utk_app_poll(&netif);
    check(4u == g_fake_connects, "a lost session reconnects at once");
    fake_mqtt_connection(MQTT_CONNECT_ACCEPTED);
    mqtt_bridge_get_stats(&stats);
    check((LINK_CONNECTED == stats.link) && (1u == stats.reconnects),
          "reconnect counted");
    test_rejoin(&netif);
}

/*!
 * @brief WiFi re-join: 3 s after the link drops, then every 5 s; a chip
 *        restart 10 s into the outage and every 10 s after it, but never
 *        while the car is driving; a new outage starts again at 3 s. One
 *        chip restart was already counted at power-up.
 *
 * @param[in,out] p_netif The fake network interface (link up on entry).
 */
static void
test_rejoin (struct netif * p_netif)
{
    check((0u == rejoins()) && (1u == restarts()), "no re-join while linked");
    p_netif->b_up = 0u;
    fake_mqtt_connection(MQTT_CONNECT_DISCONNECTED);
    cyw43_utk_app_poll(p_netif);
    g_fake_now_ms += REJOIN_AFTER_MS - 1u;
    cyw43_utk_app_poll(p_netif);
    check(0u == rejoins(), "no re-join before 3 s");
    g_fake_now_ms++;
    cyw43_utk_app_poll(p_netif);
    check(1u == rejoins(), "re-join after 3 s");
    g_fake_now_ms += REJOIN_EVERY_MS - 1u;
    cyw43_utk_app_poll(p_netif);
    check(1u == rejoins(), "the next re-join waits 5 s");
    g_fake_now_ms++;
    cyw43_utk_app_poll(p_netif);
    check(2u == rejoins(), "re-join again after 5 s");

    /* 8 s into the outage: the chip restart is due at 10 s. */
    g_fake_now_ms += (RESTART_MS - REJOIN_AFTER_MS - REJOIN_EVERY_MS) - 1u;
    cyw43_utk_app_poll(p_netif);
    check(1u == restarts(), "no chip restart before 10 s");
    g_fake_now_ms++;
    cyw43_utk_app_poll(p_netif);
    check((2u == restarts()) && (2u == rejoins()),
          "still down after 10 s: WiFi chip restarted");
    g_fake_now_ms += REJOIN_EVERY_MS;
    cyw43_utk_app_poll(p_netif);
    check(3u == rejoins(), "re-join 5 s after the restart");

    mqtt_bridge_set_restart_ok(false);
    g_fake_now_ms += REJOIN_EVERY_MS;
    cyw43_utk_app_poll(p_netif);
    check((2u == restarts()) && (4u == rejoins()),
          "driving: no chip restart, re-joins go on");
    mqtt_bridge_set_restart_ok(true);

    p_netif->b_up = 1u;
    cyw43_utk_app_poll(p_netif);
    p_netif->b_up = 0u;
    cyw43_utk_app_poll(p_netif);
    g_fake_now_ms += REJOIN_AFTER_MS;
    cyw43_utk_app_poll(p_netif);
    check(5u == rejoins(), "a new outage starts again at 3 s");
    test_noip_rejoin(p_netif);
}

/*!
 * @brief Joined but no IP address (DHCP silent): DHCP restart and re-join
 *        every 15 s until an address arrives, then quiet again.
 *
 * @param[in,out] p_netif The fake network interface (link down on entry).
 */
static void
test_noip_rejoin (struct netif * p_netif)
{
    uint32_t before = 0u;

    p_netif->b_up    = 1u;
    p_netif->ip.addr = 1u;
    cyw43_utk_app_poll(p_netif);
    before = rejoins();
    check(0u == g_fake_dhcp_starts, "a link-down outage does not restart DHCP");

    p_netif->ip.addr = 0u;
    cyw43_utk_app_poll(p_netif);
    g_fake_now_ms += NOIP_REJOIN_MS - 1u;
    cyw43_utk_app_poll(p_netif);
    check(before == rejoins(), "joined without an IP: DHCP gets 15 s");
    g_fake_now_ms++;
    cyw43_utk_app_poll(p_netif);
    check(((before + 1u) == rejoins()) && (1u == g_fake_dhcp_starts),
          "still no IP after 15 s: DHCP restarted and re-joined");
    g_fake_now_ms += NOIP_REJOIN_MS;
    cyw43_utk_app_poll(p_netif);
    check(((before + 2u) == rejoins()) && (2u == g_fake_dhcp_starts),
          "and again every 15 s");

    p_netif->ip.addr = 1u;
    cyw43_utk_app_poll(p_netif);
    g_fake_now_ms += NOIP_REJOIN_MS;
    cyw43_utk_app_poll(p_netif);
    check((before + 2u) == rejoins(), "an address ends the retries");
    test_failed_join(p_netif);
}

/*!
 * @brief A join that has failed (hotspot not found) is retried every 5 s
 *        from the start, without the 3 s first wait (chip restarts are
 *        held off here to see the re-joins alone).
 *
 * @param[in,out] p_netif The fake network interface (linked on entry).
 */
static void
test_failed_join (struct netif * p_netif)
{
    uint32_t before  = rejoins();
    uint32_t idx     = 0u;
    bool     b_early = false;

    mqtt_bridge_set_restart_ok(false);
    p_netif->b_up       = 0u;
    gb_stub_join_failed = true;
    cyw43_utk_app_poll(p_netif);

    for (idx = 0u; idx < FAILED_STEPS; idx++)
    {
        g_fake_now_ms += REJOIN_EVERY_MS - 1u;
        cyw43_utk_app_poll(p_netif);
        b_early = b_early || ((before + idx) != rejoins());
        g_fake_now_ms++;
        cyw43_utk_app_poll(p_netif);
    }

    check((!b_early) && ((before + FAILED_STEPS) == rejoins()),
          "a failed join is retried every 5 s");
    gb_stub_join_failed = false;
    p_netif->b_up       = 1u;
    cyw43_utk_app_poll(p_netif);
    mqtt_bridge_set_restart_ok(true);
}

/*!
 * @brief WiFi re-join attempts counted by the bridge.
 *
 * @return The count.
 */
static uint32_t
rejoins (void)
{
    mqtt_bridge_stats_t stats;

    mqtt_bridge_get_stats(&stats);

    return (stats.rejoins);
}

/*!
 * @brief WiFi chip restarts counted by the bridge.
 *
 * @return The count.
 */
static uint32_t
restarts (void)
{
    mqtt_bridge_stats_t stats;

    mqtt_bridge_get_stats(&stats);

    return (stats.wifi_restarts);
}

/*!
 * @brief Feed a string to a line assembler byte by byte.
 *
 * @param[in,out] p_line Line assembler.
 * @param[in]     p_text Bytes to feed.
 * @param[out]    p_out  Completed line (COMMAND_LINE_MAX bytes).
 *
 * @return Number of complete lines produced.
 */
static uint32_t
feed_text (command_line_t * p_line, char const * p_text, char * p_out)
{
    uint32_t     lines    = 0u;
    char const * p_cursor = p_text;

    while ('\0' != *p_cursor)
    {
        /* Cast: test input is ASCII, identical as uint8_t. */
        lines += command_line_feed(p_line, (uint8_t) *p_cursor, p_out,
                                   COMMAND_LINE_MAX)
                     ? 1u
                     : 0u;
        p_cursor++;
    }

    return (lines);
}

/*!
 * @brief Command parser, line assembler and fixed-point formatter.
 */
static void
test_command (void)
{
    command_t      cmd;
    command_line_t line;
    char           out[COMMAND_LINE_MAX];
    char           text[COMMAND_DECIMAL_MAX];
    uint32_t       idx = 0u;

    check((command_parse("start", &cmd)) && (COMMAND_START == cmd.kind),
          "start");
    check((command_parse("  STOP \r\n", &cmd)) && (COMMAND_STOP == cmd.kind),
          "case and spaces");
    check((!command_parse("starter", &cmd)) && (COMMAND_INVALID == cmd.kind),
          "no prefix match");
    check((command_parse("pid=0.08,0.6,0", &cmd)) && (COMMAND_PID == cmd.kind)
              && (3u == cmd.n_args)
              && (nearly(cmd.arg[0], 0.08f, GAIN_TOLERANCE))
              && (nearly(cmd.arg[1], 0.6f, GAIN_TOLERANCE))
              && (nearly(cmd.arg[2], 0.0f, GAIN_TOLERANCE)),
          "pid values");
    check((command_parse("line = 160 , 0 , 12.5", &cmd))
              && (COMMAND_LINE == cmd.kind)
              && (nearly(cmd.arg[0], 160.0f, GAIN_TOLERANCE))
              && (nearly(cmd.arg[2], 12.5f, GAIN_TOLERANCE)),
          "spaces around = and ,");
    check((command_parse("ff=.1,18", &cmd))
              && (nearly(cmd.arg[0], 0.1f, GAIN_TOLERANCE)),
          "leading-dot decimal");
    check((command_parse("speed=150", &cmd)) && (COMMAND_SPEED == cmd.kind)
              && (nearly(cmd.arg[0], 150.0f, GAIN_TOLERANCE)),
          "speed");
    check((command_parse("rate=0", &cmd)) && (COMMAND_RATE == cmd.kind),
          "rate off");
    check(!command_parse("rate=10", &cmd), "rate too fast rejected");
    check((command_parse("nav=4", &cmd)) && (COMMAND_NAV == cmd.kind)
              && (nearly(cmd.arg[0], 4.0f, GAIN_TOLERANCE))
              && (command_allowed(COMMAND_NAV, false)),
          "nav: U-turn, accepted while driving");
    check((!command_parse("nav=0", &cmd)) && (!command_parse("nav=5", &cmd)),
          "nav: only 1..4");
    check((command_parse("map=1,20", &cmd)) && (COMMAND_MAP == cmd.kind)
              && (nearly(cmd.arg[1], 20.0f, GAIN_TOLERANCE))
              && (!command_allowed(COMMAND_MAP, false)),
          "map: left = K (index 20), only while stopped");
    check((!command_parse("map=5,10", &cmd)) && (!command_parse("map=1,43", &cmd)),
          "map: 1..4 and index 0..42 only");
    check(!command_parse("pid=1,2", &cmd), "pid needs 3 values");
    check(!command_parse("pid=1,2,3,4", &cmd), "too many values");
    check(!command_parse("pid=-1,0,0", &cmd), "negative gain rejected");
    check(!command_parse("pid=a,b,c", &cmd), "letters rejected");
    check(!command_parse("ff=0.1,150", &cmd), "offset > 100 rejected");
    check(!command_parse("stop now", &cmd), "trailing text rejected");
    check(!command_parse("gains=1", &cmd), "gains takes no value");
    check(!command_parse("", &cmd), "empty rejected");
    check((command_parse("#17 gains", &cmd)) && (17u == cmd.req_id)
              && (COMMAND_GAINS == cmd.kind),
          "request id");
    check((command_parse("speed=150", &cmd)) && (0u == cmd.req_id),
          "no request id is 0");
    check((!command_parse("#12gains", &cmd))
              && (!command_parse("#0 gains", &cmd))
              && (!command_parse("# gains", &cmd)),
          "malformed request ids rejected");
    check((!command_parse("#5 bogus", &cmd)) && (5u == cmd.req_id),
          "the id survives a bad command (for the reply)");

    /* Line assembler: CRLF, backspace, control bytes and overflow. */
    command_line_init(&line);
    check((1u == feed_text(&line, "pix\bd=1,2,3\r\n", out))
              && (0 == strcmp(out, "pid=1,2,3")),
          "backspace edit, CRLF = 1 line");
    (void) command_line_feed(&line, 0x00u, out, COMMAND_LINE_MAX);
    (void) command_line_feed(&line, 's', out, COMMAND_LINE_MAX);
    check(!command_line_feed(&line, 0x1Bu, out, COMMAND_LINE_MAX),
          "control byte ignored");
    (void) feed_text(&line, "top\n", out);
    check(0 == strcmp(out, "stop"), "NUL and ESC ignored");

    for (idx = 0u; idx < OVERFLOW_BYTES; idx++)
    {
        (void) command_line_feed(&line, 'x', out, COMMAND_LINE_MAX);
    }

    check(!command_line_feed(&line, '\n', out, COMMAND_LINE_MAX),
          "overlong line dropped");
    (void) feed_text(&line, "gains\n", out);
    check(0 == strcmp(out, "gains"), "recovers after an overflow");

    /* Fixed-point formatter (tm_sprintf() has no %f). */
    command_format_decimal(text, 0.08f);
    check(0 == strcmp(text, "0.080"), "0.080");
    command_format_decimal(text, 160.0f);
    check(0 == strcmp(text, "160.000"), "160.000");
    command_format_decimal(text, -1.2345f);
    check(0 == strcmp(text, "-1.235"), "-1.235");
    command_format_decimal(text, 0.0f);
    check(0 == strcmp(text, "0.000"), "0.000");
    command_format_decimal(text, -0.0004f);
    check(0 == strcmp(text, "0.000"), "-0.0004 gives 0.000");
    command_format_decimal(text, sqrtf(-1.0f));
    check(0 == strcmp(text, "0.000"), "NaN gives 0.000");
}

/*!
 * @brief Command limits from the CSC05 review: speed= 60..300 (NFR2),
 *        range-checked rate=, the timing command, and tuning only
 *        while the car is stationary.
 */
static void
test_command_limits (void)
{
    command_t cmd;

    check((command_parse("speed=300", &cmd))
              && (command_parse("speed=60", &cmd))
              && (!command_parse("speed=301", &cmd))
              && (!command_parse("speed=59", &cmd))
              && (!command_parse("speed=99999999999", &cmd)),
          "speed 60..300 only (NFR2)");
    check((!command_parse("rate=99999999999", &cmd))
              && (!command_parse("rate=-5", &cmd)),
          "rate: huge or negative values rejected");
    check((command_parse("timing", &cmd)) && (COMMAND_TIMING == cmd.kind),
          "timing command");
    check((!command_allowed(COMMAND_SPEED, false))
              && (!command_allowed(COMMAND_PID, false))
              && (!command_allowed(COMMAND_FF, false))
              && (!command_allowed(COMMAND_LINE, false)),
          "tuning refused while driving");
    check((command_allowed(COMMAND_PID, true))
              && (command_allowed(COMMAND_STOP, false))
              && (command_allowed(COMMAND_START, false))
              && (command_allowed(COMMAND_RATE, false))
              && (command_allowed(COMMAND_GAINS, false))
              && (command_allowed(COMMAND_TIMING, false)),
          "tuning when stationary; other commands always");
}

/*!
 * @brief Live line-following gains survive re-initialisation.
 */
static void
test_line_gains (void)
{
    line_follow_t follower;
    float32_t     gain_p = 0.0f;
    float32_t     gain_i = 0.0f;
    float32_t     gain_d = 0.0f;

    line_follow_init(&follower);
    line_follow_set_gains(&follower, 90.0f, 1.0f, 5.0f);
    check((nearly(follower.pid.gain_p, 90.0f, GAIN_TOLERANCE))
              && (nearly(follower.pid.gain_d, 5.0f, GAIN_TOLERANCE)),
          "running PID updated");
    line_follow_init(&follower);
    line_follow_get_gains(&gain_p, &gain_i, &gain_d);
    check((nearly(follower.pid.gain_p, 90.0f, GAIN_TOLERANCE))
              && (nearly(gain_p, 90.0f, GAIN_TOLERANCE))
              && (nearly(gain_i, 1.0f, GAIN_TOLERANCE)),
          "kept after init");
    line_follow_set_gains(NULL, LINE_KP, LINE_KI, LINE_KD);
    (void) printf("command: parser, line assembler, formatter and live gains "
                  "OK\n");
}

/*!
 * @brief Snapshot of the simulated vehicle's status.
 *
 * @return Status.
 */
static vehicle_status_t
vehicle_status (void)
{
    vehicle_status_t status;

    vehicle_get_status(&status);

    return (status);
}

/*!
 * @brief Whether the simulated vehicle is in a state, for a reason.
 *
 * @param[in] state Expected state.
 * @param[in] p_why Expected reason, or NULL for any.
 *
 * @return true if both match.
 */
static bool
vehicle_is (vehicle_state_t state, char const * p_why)
{
    vehicle_status_t status = vehicle_status();

    return ((state == status.state)
            && ((NULL == p_why) || (0 == strcmp(status.p_reason, p_why))));
}

/*!
 * @brief Tap START (a run starts on release), then drive on the line.
 *
 * @param[in] drive_ms Simulated time to run for.
 */
static void
vehicle_start (uint32_t drive_ms)
{
    vehicle_sim_line(true);
    vehicle_sim_press(PIN_BTN_START, TAP_AT_MS, TAP_MS);
    vehicle_sim_run(drive_ms);
}

/*!
 * @brief Two front readings in a row closer than OBST_DETECT_MM.
 */
static void
vehicle_close_echo (void)
{
    vehicle_sim_front(NEAR_MM);
    vehicle_sim_front(NEARER_MM);
}

/*!
 * @brief Servo: angles past the mount's limits are held at the limit, so
 *        the servo never pushes the sensor against the car.
 */
static void
test_servo_limits (void)
{
    uint32_t right_us = 0u;
    uint32_t left_us  = 0u;

    servo_set_angle(SERVO_MIN_DEG);
    right_us = vehicle_sim_servo_us();
    servo_set_angle(SERVO_MAX_DEG);
    left_us = vehicle_sim_servo_us();
    servo_set_angle(0);
    check((SERVO_MIN_DEG == servo_get_angle())
              && (right_us == vehicle_sim_servo_us()),
          "servo: 0 deg is held at SERVO_MIN_DEG");
    servo_set_angle(180);
    check((SERVO_MAX_DEG == servo_get_angle())
              && (left_us == vehicle_sim_servo_us()),
          "servo: 180 deg is held at SERVO_MAX_DEG");
    servo_set_angle(SERVO_CENTRE_DEG);
}

/*!
 * @brief Vehicle task: START and STOP buttons and the speed limit.
 */
static void
test_vehicle_buttons (void)
{
    vehicle_sim_init();
    check((vehicle_is(VS_IDLE, "boot")) && (!vehicle_sim_watchdog_armed()),
          "vehicle: boots IDLE, watchdog off");
    check(CENTRE_US == vehicle_sim_servo_us(), "vehicle: sonar centred");
    test_servo_limits();

    vehicle_sim_press(PIN_BTN_STOP, 50u, 200u);
    vehicle_sim_run(600u);
    check((vehicle_is(VS_IDLE, NULL)) && (0u == vehicle_sim_count("L360")),
          "vehicle: STOP while idle never moves the car");

    vehicle_sim_press(PIN_BTN_START, 50u, 300u);
    vehicle_sim_run(300u);
    check(vehicle_is(VS_IDLE, NULL), "vehicle: START held: no run yet");
    vehicle_sim_run(200u);
    check((vehicle_is(VS_LINE_FOLLOW, "start"))
              && (vehicle_sim_watchdog_armed()),
          "vehicle: START released: run, watchdog armed");

    /* 6 ms, between two 20 ms polls: only the interrupt latch sees it. */
    vehicle_sim_press(PIN_BTN_STOP, 3u, 6u);
    vehicle_sim_run(60u);
    check((vehicle_is(VS_STOPPED, "stop command"))
              && (!vehicle_sim_watchdog_armed()),
          "vehicle: short STOP press latched, watchdog off");

    vehicle_sim_clear_log();
    vehicle_sim_press(PIN_BTN_START, 50u, 2500u);
    vehicle_sim_run(2500u);
    check(0u == vehicle_sim_count("L360"), "vehicle: no spin while held");
    vehicle_sim_run(1500u);
    check((1u == vehicle_sim_count("L360"))
              && (vehicle_is(VS_IDLE, "calibrated")),
          "vehicle: START held 2 s calibrates on release");

    vehicle_start(450u);
    vehicle_sim_press(PIN_BTN_START, 50u, 350u);
    vehicle_sim_press(PIN_BTN_STOP, 100u, 50u);
    vehicle_sim_run(600u);
    check(vehicle_is(VS_STOPPED, "stop command"),
          "vehicle: START pressed during a run does not restart it");

    (void) vehicle_command(VCMD_SET_SPEED, 400);
    vehicle_sim_step();
    check(300 == vehicle_status().cruise_mm_s, "vehicle: speed=400 -> 300");
    (void) vehicle_command(VCMD_SET_SPEED, 10);
    vehicle_sim_step();
    check(60 == vehicle_status().cruise_mm_s, "vehicle: speed=10 -> 60");
    (void) vehicle_command(VCMD_SET_SPEED, 180);
    vehicle_sim_step();
}

/*!
 * @brief Vehicle task: obstacle triggers, scan, bypass, impact reverse and
 *        the hump guard.
 */
static void
test_vehicle_obstacle (void)
{
    avoidance_point_t points[SCAN_MAX_POINTS];
    uint32_t          pings = 0u;

    vehicle_start(DRIVE_MS);
    vehicle_sim_box(true);
    vehicle_sim_clear_log();
    vehicle_sim_front(NEAR_MM);
    vehicle_sim_run(40u);
    check(0u == vehicle_sim_count("-> OBSTACLE"),
          "vehicle: one close echo is ignored");
    vehicle_close_echo();
    vehicle_sim_step();
    check(vehicle_is(VS_OBSTACLE, NULL), "vehicle: two close echoes stop");

    vehicle_sim_clear_log();
    pings = vehicle_sim_pings();
    vehicle_sim_step();
    check(vehicle_sim_log_starts("u"), "vehicle: front trigger: no reverse");
    /* 11 fine looks x 3 pings (the monitor was the coarse stage); the
       pass needs no side looks. */
    check(((vehicle_sim_pings() - pings) == 33u)
              && (1u == vehicle_sim_count("{[scan] pts=11 found=1")),
          "vehicle: fine scan, inside the mount's limits");
    check((11u == obstacle_get_points(points, SCAN_MAX_POINTS))
              && (65 == points[0].angle_deg) && (150 == points[2].dist_mm),
          "vehicle: last scan points kept (65..115 deg fine scan)");
    check((2u == vehicle_sim_count("L45")) && (2u == vehicle_sim_count("R45"))
              && (0u == vehicle_sim_count("L90")) && (0u == vehicle_sim_count("R90"))
              && (vehicle_is(VS_LINE_FOLLOW, "obstacle bypassed")),
          "vehicle: diagonal bypass, line rejoined");
    /* Box seen 75..105 deg, centred: side unsure, blind offset 280 mm.
       45 deg leg 280 x 1.414 = 396.  Box edge 280 - 80 - 60 = 140, near
       face 150 + sonar 70 = 220, so back off 140 + 0.707 x 160 + 1.414 x
       30 - 220 = 76; return at 220 + 76 + deepest 150 + 140 - 280 + 156 =
       461, less the 280 the leg covered. */
    check((1u == vehicle_sim_count("back76;"))
              && (2u == vehicle_sim_count("fwd396;"))
              && (1u == vehicle_sim_count("fwd181;")),
          "vehicle: diagonal leg and the rest of the pass");
    vehicle_sim_box(false);

    vehicle_sim_impact();
    vehicle_sim_step();
#if OBST_ON_IMPACT
    check(vehicle_is(VS_OBSTACLE, NULL), "vehicle: impact stops the car");
    vehicle_sim_clear_log();
    vehicle_sim_step();
    check(vehicle_sim_log_starts("back120;u"),
          "vehicle: impact: reverse 120 mm, then scan");
    check(vehicle_is(VS_LINE_FOLLOW, "path clear"),
          "vehicle: nothing found: continue");
#else
    /* car_config.h OBST_ON_IMPACT 0 (line-following tuning): impacts are
       counted and reported but do not start the obstacle routine. */
    check(vehicle_is(VS_LINE_FOLLOW, NULL),
          "vehicle: impact ignored while OBST_ON_IMPACT is 0");
#endif

    vehicle_sim_run(DRIVE_MS);
    vehicle_sim_hump(HUMP_DESCENDING);
    vehicle_sim_clear_log();
    vehicle_close_echo();
    vehicle_sim_impact();
    vehicle_sim_run(40u);
    check(0u == vehicle_sim_count("-> OBSTACLE"),
          "vehicle: on a hump, sonar and impact are ignored");
    vehicle_sim_hump(HUMP_FLAT);
    vehicle_sim_front(FAR_MM);
    vehicle_sim_run(200u);
    check(0u == vehicle_sim_count("-> OBSTACLE"),
          "vehicle: an impact on the hump is not replayed");
}

/*!
 * @brief Vehicle task: STOP button and stop command during a scan.
 */
static void
test_vehicle_scan_stop (void)
{
    uint32_t pings = 0u;

    vehicle_close_echo();
    vehicle_sim_step();
    check(vehicle_is(VS_OBSTACLE, NULL), "vehicle: obstacle (scan stop)");
    vehicle_sim_clear_log();
    pings = vehicle_sim_pings();
    vehicle_sim_press(PIN_BTN_STOP, 420u, 30u); /* in the first servo step */
    vehicle_sim_step();
    check((0u == (vehicle_sim_pings() - pings))
              && (1u == vehicle_sim_count("{[scan] stopped}")),
          "vehicle: STOP while the servo moves ends the scan, no more pings");
    check((0u == vehicle_sim_count("L45")) && (0u == vehicle_sim_count("fwd")),
          "vehicle: no move after a stopped scan");
    check(CENTRE_US == vehicle_sim_servo_us(), "vehicle: sonar re-centred");
    vehicle_sim_step();
    check(vehicle_is(VS_STOPPED, "stop command"),
          "vehicle: scan stopped -> STOPPED (stop command)");

    vehicle_start(DRIVE_MS);
    vehicle_close_echo();
    vehicle_sim_step();
    pings = vehicle_sim_pings();
    vehicle_sim_stop_command_at(800u); /* during the second servo step */
    vehicle_sim_step();
    vehicle_sim_step();
    check((3u == (vehicle_sim_pings() - pings))
              && (vehicle_is(VS_STOPPED, "stop command")),
          "vehicle: stop command ends the scan within 20 ms");
}

/*!
 * @brief Vehicle task: a STOP during a line search, a barcode U-turn or a
 *        bypass gives one STOPPED (stop command) and no failure state.
 */
static void
test_vehicle_manoeuvre_stop (void)
{
    vehicle_start(200u);
    vehicle_sim_clear_log();
    vehicle_sim_line(false);
    vehicle_sim_press(PIN_BTN_STOP, 1200u, 60u); /* during the sweeps */
    vehicle_sim_run(8000u);
    check(1u == vehicle_sim_count("-> LINE_SEARCH (line lost)"),
          "vehicle: line lost -> LINE_SEARCH");
    check((vehicle_is(VS_STOPPED, "stop command"))
              && (0u == vehicle_sim_count("line not found")),
          "vehicle: STOP in a line search reported once");

    vehicle_start(200u);
    vehicle_sim_barcode(NAV_UTURN);
    vehicle_sim_step();
    check(vehicle_is(VS_NAV_TURN, NULL), "vehicle: U-turn barcode");
    vehicle_sim_clear_log();
    vehicle_sim_press(PIN_BTN_STOP, 100u, 30u); /* during the 180 turn */
    vehicle_sim_run(600u);
    check((vehicle_is(VS_STOPPED, "stop command"))
              && (0u == vehicle_sim_count("LINE_SEARCH")),
          "vehicle: STOP in a U-turn reported once");

    vehicle_start(DRIVE_MS);
    vehicle_sim_box(true);
    vehicle_close_echo();
    vehicle_sim_step();
    vehicle_sim_clear_log();
    vehicle_sim_stop_on_next_turn();
    vehicle_sim_step();
    vehicle_sim_step();
    check((1u == vehicle_sim_count("L45")) && (0u == vehicle_sim_count("fwd"))
              && (vehicle_is(VS_STOPPED, "stop command"))
              && (0u == vehicle_sim_count("LINE_SEARCH")),
          "vehicle: STOP in a bypass reported once");

    /* Something ahead during the pass: stop beside the obstacle. */
    vehicle_start(DRIVE_MS);
    vehicle_close_echo();
    vehicle_sim_step();
    vehicle_sim_clear_log();
    vehicle_sim_front_ambient(ULTRASONIC_VALID, NEAR_MM);
    vehicle_sim_step();
    vehicle_sim_front_ambient(ULTRASONIC_OUT_OF_RANGE, ULTRASONIC_NO_ECHO);
    check((1u == vehicle_sim_count("fwd181;"))
              && (1u == vehicle_sim_count("L45")) && (1u == vehicle_sim_count("R45"))
              && (vehicle_is(VS_STOPPED, "path blocked"))
              && (0u == vehicle_sim_count("LINE_SEARCH")),
          "vehicle: obstacle ahead on the pass: STOPPED (path blocked)");
    vehicle_sim_box(false);
}

/*!
 * @brief Vehicle task: a failing or silent front sensor stops the car
 *        (Week 6 obstacle statechart).
 */
static void
test_vehicle_sonar_health (void)
{
    vehicle_start(DRIVE_MS);
    vehicle_sim_front_fault();
    vehicle_sim_step();
    check(vehicle_is(VS_LINE_FOLLOW, NULL),
          "vehicle: one FAULT ping is tolerated");
    vehicle_sim_front_fault();
    vehicle_sim_front_fault();
    vehicle_sim_step();
    check(vehicle_is(VS_STOPPED, "fault: sonar no echo"),
          "vehicle: two FAULT pings in a row stop the car");

    vehicle_start(DRIVE_MS);
    vehicle_sim_monitor_starve(true);
    vehicle_sim_run(OBST_STALE_MS + 100u);
    check(vehicle_is(VS_STOPPED, "sonar stale"),
          "vehicle: no front reading for 300 ms stops the car");
    vehicle_sim_monitor_starve(false);
}

/*!
 * @brief Vehicle task: stale barcodes at START and the watchdog feed gap.
 */
static void
test_vehicle_barcode_watchdog (void)
{
    uint32_t gap = 0u;

    vehicle_sim_barcode(NAV_LEFT);
    vehicle_sim_barcode(NAV_RIGHT);
    vehicle_sim_clear_log();
    vehicle_start(200u);
    check((NAV_NONE == vehicle_status().pending_nav)
              && (0u == vehicle_sim_queued_barcodes())
              && (1u == vehicle_sim_count("2 stale barcode(s) discarded")),
          "vehicle: barcodes read while stopped are discarded");
    vehicle_sim_barcode(NAV_RIGHT);
    vehicle_sim_step(); /* taken in this loop ...               */
    vehicle_sim_step(); /* ... and published at the next one */
    check(NAV_RIGHT == vehicle_status().pending_nav,
          "vehicle: a barcode read during the run is kept");

    /* Turn once the barcode sensor is past the junction line, not on a
       barcode-sized bar. */
    vehicle_sim_press(PIN_BTN_STOP, 0u, 30u);
    vehicle_sim_run(200u);
    vehicle_start(1000u);
    vehicle_sim_barcode(NAV_LEFT);
    vehicle_sim_run(100u);
    vehicle_sim_bar_black(true);
    vehicle_sim_run(BAR_NARROW_MS);
    vehicle_sim_bar_black(false);
    vehicle_sim_run(100u);
    check((vehicle_is(VS_LINE_FOLLOW, NULL))
              && (NAV_LEFT == vehicle_status().pending_nav),
          "vehicle: no turn on a barcode-sized bar");
    vehicle_sim_clear_log();
    vehicle_sim_bar_black(true);
    vehicle_sim_run(BAR_JUNCTION_MS);
    check(0u == vehicle_sim_count("NAV_TURN"),
          "vehicle: no turn while on the junction line");
    vehicle_sim_bar_black(false);
    vehicle_sim_run(60u);
    check((1u == vehicle_sim_count("junction line"))
              && (1u == vehicle_sim_count("NAV_TURN")),
          "vehicle: turn once past the junction line");

    gap = vehicle_sim_watchdog_gap_ms();
    check((WATCHDOG_TIMEOUT_MS == vehicle_sim_watchdog_timeout_ms())
              && (gap < (WATCHDOG_TIMEOUT_MS / 2u)),
          "vehicle: watchdog fed within half its timeout");
    (void) printf("vehicle: buttons, scan, bypass, impact, hump, stop and "
                  "barcode scenarios; longest watchdog gap %" PRIu32 " ms\n",
                  gap);
}

/*** end of file ***/
