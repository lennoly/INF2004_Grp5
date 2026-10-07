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
#include "line_follow.h"
#include "lwip/apps/mqtt.h"
#include "mqtt_bridge.h"
#include "mqtt_lwip.h"
#include "obstacle.h"
#include "pid.h"
#include "sim/vehicle_sim.h"
#include "terrain.h"
#include "vehicle.h"
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

/* Terrain. */
#define HUMP_SPEED_MM_S (150.0f)
#define HUMP_LENGTH_MM  (250.0f)
#define HUMP_LEAD_MM    (200.0f)
#define HUMP_TAIL_MM    (300.0f)
#define PITCH_NOISE_DEG (1.0f)
#define HEIGHT_COUNT    (4u)
#define HEIGHT_FRACTION (0.1f)
#define HEIGHT_SLACK_MM (2.0f)
#define PI_F            (3.14159f)

/* Obstacle scan. */
#define SCAN_FIRST_DEG  (10)
#define SCAN_LAST_DEG   (170)
#define SCAN_STEP_DEG   (5)
#define NO_ECHO_MM      (9999.0f)
#define BOX_WIDTH_MM    (100.0f)
#define BOX_RANGE_MM    (250.0f)
#define WIDTH_TOLERANCE (45.0f)

/* Command parser. */
#define GAIN_TOLERANCE (1e-6f)
#define OVERFLOW_BYTES (100u)

/* MQTT. */
#define PUBLISHES       (10u)
#define NET_UP_MS       (5000u)  /* the network comes up after boot  */
#define FIRST_WINDOW_MS (2000u)  /* the first attempt's time limit   */
#define SESSION_MS      (60000u) /* a session that ran for a while   */

/* Vehicle simulation (sim/): one press of START, a 1.5 s drive (past the
   150 mm front-ignore window after "path clear") and a close echo pair. */
#define TAP_AT_MS (20u)
#define TAP_MS    (40u)
#define DRIVE_MS  (1500u)
#define NEAR_MM   (250)
#define NEARER_MM (240)
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
                                      {'*', "010010100"}};

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
static uint32_t     make_frame(char data, uint32_t * p_widths, float32_t noise,
                               float32_t ramp);
static void reverse(uint32_t const * p_src, uint32_t * p_dst, uint32_t count);
static void test_barcode(void);
static float32_t simulate_hump(float32_t height, float32_t length,
                               float32_t noise_deg, uint32_t * p_humps);
static void      test_terrain(void);
static uint32_t  scan_box(avoidance_point_t * p_points, float32_t centre_x,
                          float32_t range_y, float32_t width,
                          float32_t wall_left);
static void      test_obstacle(void);
static void      test_bridge(void);
static void      test_mqtt_client(void);
static uint32_t  feed_text(command_line_t * p_line, char const * p_text,
                           char * p_out);
static void      test_command(void);
static void      test_line_gains(void);
static vehicle_status_t vehicle_status(void);
static bool             vehicle_is(vehicle_state_t state, char const * p_why);
static void             vehicle_start(uint32_t drive_ms);
static void             vehicle_close_echo(void);
static void             test_vehicle_buttons(void);
static void             test_vehicle_obstacle(void);
static void             test_vehicle_scan_stop(void);
static void             test_vehicle_manoeuvre_stop(void);
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
    test_vehicle_barcode_watchdog();
    test_bridge();
    test_mqtt_client();
    test_command();
    test_line_gains();
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
}

/*!
 * @brief Code 39 bar pattern of a symbol used by the tests.
 *
 * @param[in] symbol 'A'..'D' or '*'.
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
    check(NAV_RIGHT == barcode_decode_to_command('B'), "B = right");
    check(NAV_STRAIGHT == barcode_decode_to_command('C'), "C = straight");
    check(NAV_UTURN == barcode_decode_to_command('D'), "D = u-turn");

    for (count = 0u; count < PHOTO_ELEMENTS; count++)
    {
        flat[count] = FLAT_WIDTH;
    }

    check(barcode_decode_frame(flat, PHOTO_ELEMENTS, text, TEXT_SIZE) < 0,
          "flat widths rejected");
    check(barcode_decode_frame(g_photo, SHORT_ELEMENTS, text, TEXT_SIZE) < 0,
          "short frame rejected");
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
        terrain_hump_update(&hump, pitch, step_mm);
        pos_mm += step_mm * cosf(theta);
    }

    *p_humps = hump.humps;

    return (hump.max_peak_mm);
}

/*!
 * @brief Hump height estimation and motion-event classification.
 */
static void
test_terrain (void)
{
    float32_t const heights[HEIGHT_COUNT] = {20.0f, 30.0f, 40.0f, 60.0f};
    uint32_t        humps                 = 0u;
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

    (void) simulate_hump(0.0f, HUMP_LENGTH_MM, PITCH_NOISE_DEG, &humps);
    check(0u == humps, "flat floor with noise: no hump");
    check(EVT_IMPACT == terrain_classify(0.0f, 0.0f, 0.0f, HUMP_FLAT, true),
          "impact");
    check(EVT_TURNING
              == terrain_classify(150.0f, 0.0f, 90.0f, HUMP_FLAT, false),
          "turn");
    check(EVT_STATIONARY
              == terrain_classify(5.0f, 0.0f, 0.0f, HUMP_FLAT, false),
          "still");
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
 *
 * @return Number of points.
 */
static uint32_t
scan_box (avoidance_point_t * p_points, float32_t centre_x, float32_t range_y,
          float32_t width, float32_t wall_left)
{
    uint32_t  count   = 0u;
    int32_t   angle   = SCAN_FIRST_DEG;
    float32_t cos_a   = 0.0f;
    float32_t sin_a   = 0.0f;
    float32_t best    = NO_ECHO_MM;
    float32_t lateral = 0.0f;

    for (angle = SCAN_FIRST_DEG; angle <= SCAN_LAST_DEG; angle += SCAN_STEP_DEG)
    {
        /* Casts: angles are 10..170, exact in float32_t and int16_t; the
           distance is below US_MAX_MM or replaced by -1. */
        cos_a   = cosf((float32_t) angle * CAR_MATH_RAD_PER_DEG);
        sin_a   = sinf((float32_t) angle * CAR_MATH_RAD_PER_DEG);
        best    = NO_ECHO_MM;
        lateral = (range_y * cos_a) / sin_a;

        /* Front face of the box: y = range_y, x in centre +- width / 2. */
        if ((lateral >= (centre_x - (width / 2.0f)))
            && (lateral <= (centre_x + (width / 2.0f))))
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

    count = scan_box(points, 20.0f, BOX_RANGE_MM, BOX_WIDTH_MM, 0.0f);
    check(avoidance_profile(points, count, &prof), "box found");
    plan = avoidance_plan(&prof, 0u);
    (void) printf("obstacle: box w=100 at x=+20: est width %.0f, "
                  "centre %.0f, closest %" PRId32 " -> %s offset %.0f\n",
                  wide(prof.width_mm), wide(prof.centre_mm), prof.closest_mm,
                  avoidance_action_name(plan.action), wide(plan.offset_mm));
    check(nearly(prof.width_mm, BOX_WIDTH_MM, WIDTH_TOLERANCE),
          "width estimate");
    check(AVOID_TURN_LEFT == plan.action, "box right of centre: pass left");

    count = scan_box(points, 20.0f, BOX_RANGE_MM, BOX_WIDTH_MM, 200.0f);
    (void) avoidance_profile(points, count, &prof);
    plan = avoidance_plan(&prof, 0u);
    (void) printf("obstacle: same box + wall 200 mm left: clear L=%" PRId32
                  " R=%" PRId32 " -> %s\n",
                  prof.clear_left_mm, prof.clear_right_mm,
                  avoidance_action_name(plan.action));
    check(AVOID_TURN_RIGHT == plan.action, "wall on the left: pass right");

    count = scan_box(points, 0.0f, 70.0f, BOX_WIDTH_MM, 0.0f);
    (void) avoidance_profile(points, count, &prof);
    check(AVOID_REVERSE == avoidance_plan(&prof, 0u).action,
          "too close: reverse");
    check(AVOID_STOP == avoidance_plan(&prof, OBST_MAX_ATTEMPTS).action,
          "then stop");

    count = scan_box(points, 400.0f, BOX_RANGE_MM, BOX_WIDTH_MM, 0.0f);
    (void) avoidance_profile(points, count, &prof);
    check(AVOID_CONTINUE == avoidance_plan(&prof, 0u).action,
          "outside the corridor: continue");
}

/*!
 * @brief Publish and command rings between the tasks and the MQTT client.
 */
static void
test_bridge (void)
{
    static uint8_t const command[] = {'s', 'p', 'e', 'e', 'd',
                                      '=', '1', '5', '0'};
    char                 cmd[COMMAND_LINE_MAX];
    char                 data[MQTT_BRIDGE_PAYLOAD_MAX];
    mqtt_bridge_topic_t  topic  = TOPIC_TELEMETRY;
    uint32_t             len    = 0u;
    uint32_t             queued = 0u;
    uint32_t             idx    = 0u;

    check(!mqtt_bridge_publish(TOPIC_TELEMETRY, "{}"), "dropped when offline");
    mqtt_bridge_set_link(LINK_CONNECTED);

    for (idx = 0u; idx < PUBLISHES; idx++)
    {
        queued += mqtt_bridge_publish(TOPIC_EVENT, "{\"x\":1}") ? 1u : 0u;
    }

    check(5u == queued, "ring holds 5 (6 slots - 1)");
    check((mqtt_bridge_peek_tx(&topic, data, sizeof(data), &len))
              && (TOPIC_EVENT == topic) && (7u == len),
          "peek");
    mqtt_bridge_drop_tx();
    mqtt_bridge_put_command(command, sizeof(command));
    check((mqtt_bridge_get_command(cmd, sizeof(cmd)))
              && (0 == strcmp(cmd, "speed=150")),
          "command");
    check(!mqtt_bridge_get_command(cmd, sizeof(cmd)), "command ring empty");
}

/*!
 * @brief MQTT connection manager (mqtt_lwip.c) against a fake lwIP: retry
 *        timing, "online" and the command subscription on connect, queue
 *        flushing and command routing.
 */
static void
test_mqtt_client (void)
{
    struct netif        netif = {0u, {0u}};
    mqtt_bridge_stats_t stats;
    char                cmd[COMMAND_LINE_MAX];
    char                data[MQTT_BRIDGE_PAYLOAD_MAX];
    mqtt_bridge_topic_t topic = TOPIC_TELEMETRY;
    uint32_t            len   = 0u;

    /* Start from an empty publish ring (test_bridge() left messages). */
    while (mqtt_bridge_peek_tx(&topic, data, sizeof(data), &len))
    {
        mqtt_bridge_drop_tx();
    }

    g_fake_now_ms = NET_UP_MS;
    cyw43_utk_app_poll(&netif);
    check(0u == g_fake_connects, "no attempt without a network");
    netif.b_up    = 1u;
    netif.ip.addr = 1u;
    cyw43_utk_app_poll(&netif);
    cyw43_utk_app_poll(&netif);
    check(1u == g_fake_connects, "one attempt once the network is up");
    g_fake_now_ms += FIRST_WINDOW_MS - 1u;
    cyw43_utk_app_poll(&netif);
    check(1u == g_fake_connects, "the attempt gets 2 s");
    g_fake_now_ms++;
    cyw43_utk_app_poll(&netif);
    check(2u == g_fake_connects, "then a new attempt");

    fake_mqtt_connection(MQTT_CONNECT_ACCEPTED);
    check((0 == strcmp(g_fake_topic, "picocar/t/status"))
              && (1u == g_fake_retain)
              && (0 == strcmp(g_fake_subscribed, "picocar/t/cmd")),
          "connected: online (retained) and cmd subscribed");

    (void) mqtt_bridge_publish(TOPIC_TELEMETRY, "{\"t\":1}");
    g_fake_publish_err = ERR_MEM;
    cyw43_utk_app_poll(&netif);
    check(mqtt_bridge_peek_tx(&topic, data, sizeof(data), &len),
          "kept while the client is full");
    g_fake_publish_err = ERR_OK;
    cyw43_utk_app_poll(&netif);
    check((!mqtt_bridge_peek_tx(&topic, data, sizeof(data), &len))
              && (0 == strcmp(g_fake_topic, "picocar/t/telemetry")),
          "sent once the client has room");

    fake_mqtt_incoming("picocar/t/other", "start");
    fake_mqtt_incoming("picocar/t/cmd", "stop");
    check((mqtt_bridge_get_command(cmd, sizeof(cmd)))
              && (0 == strcmp(cmd, "stop"))
              && (!mqtt_bridge_get_command(cmd, sizeof(cmd))),
          "only the cmd topic gives commands");

    g_fake_now_ms += SESSION_MS;
    fake_mqtt_connection(MQTT_CONNECT_DISCONNECTED);
    cyw43_utk_app_poll(&netif);
    check(3u == g_fake_connects, "a lost session reconnects at once");
    fake_mqtt_connection(MQTT_CONNECT_ACCEPTED);
    mqtt_bridge_get_stats(&stats);
    check((LINK_CONNECTED == stats.link) && (1u == stats.reconnects),
          "reconnect counted");
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
    check(!command_parse("pid=1,2", &cmd), "pid needs 3 values");
    check(!command_parse("pid=1,2,3,4", &cmd), "too many values");
    check(!command_parse("pid=-1,0,0", &cmd), "negative gain rejected");
    check(!command_parse("pid=a,b,c", &cmd), "letters rejected");
    check(!command_parse("ff=0.1,150", &cmd), "offset > 100 rejected");
    check(!command_parse("stop now", &cmd), "trailing text rejected");
    check(!command_parse("gains=1", &cmd), "gains takes no value");
    check(!command_parse("", &cmd), "empty rejected");

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
 * @brief Vehicle task: START and STOP buttons and the speed limit.
 */
static void
test_vehicle_buttons (void)
{
    vehicle_sim_init();
    check(vehicle_is(VS_IDLE, "boot"), "vehicle: boots IDLE");
    check(CENTRE_US == vehicle_sim_servo_us(), "vehicle: sonar centred");

    vehicle_sim_press(PIN_BTN_STOP, 50u, 200u);
    vehicle_sim_run(600u);
    check((vehicle_is(VS_IDLE, NULL)) && (0u == vehicle_sim_count("L360")),
          "vehicle: STOP while idle never moves the car");

    vehicle_sim_press(PIN_BTN_START, 50u, 300u);
    vehicle_sim_run(300u);
    check(vehicle_is(VS_IDLE, NULL), "vehicle: START held: no run yet");
    vehicle_sim_run(200u);
    check(vehicle_is(VS_LINE_FOLLOW, "start"), "vehicle: START released: run");

    /* 6 ms, between two 20 ms polls: only the interrupt latch sees it. */
    vehicle_sim_press(PIN_BTN_STOP, 3u, 6u);
    vehicle_sim_run(60u);
    check(vehicle_is(VS_STOPPED, "stop command"),
          "vehicle: short STOP press latched");

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
    vehicle_sim_front(NEARER_MM);
    vehicle_sim_step();
    check(vehicle_is(VS_OBSTACLE, NULL), "vehicle: two close echoes stop");

    vehicle_sim_clear_log();
    pings = vehicle_sim_pings();
    vehicle_sim_step();
    check(vehicle_sim_log_starts("u"), "vehicle: front trigger: no reverse");
    check((vehicle_sim_pings() - pings) >= 12u,
          "vehicle: coarse then fine scan");
    check((9u == obstacle_get_points(points, SCAN_MAX_POINTS))
              && (70 == points[0].angle_deg) && (200 == points[1].dist_mm),
          "vehicle: last scan points kept (70..110 deg fine scan)");
    check((2u == vehicle_sim_count("L90")) && (2u == vehicle_sim_count("R90"))
              && (vehicle_is(VS_LINE_FOLLOW, "obstacle bypassed")),
          "vehicle: box-shaped bypass, line rejoined");
    vehicle_sim_box(false);

    vehicle_sim_impact();
    vehicle_sim_step();
    check(vehicle_is(VS_OBSTACLE, NULL), "vehicle: impact stops the car");
    vehicle_sim_clear_log();
    vehicle_sim_step();
    check(vehicle_sim_log_starts("back120;u"),
          "vehicle: impact: reverse 120 mm, then scan");
    check(vehicle_is(VS_LINE_FOLLOW, "path clear"),
          "vehicle: nothing found: continue");

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
    check((1u == (vehicle_sim_pings() - pings))
              && (1u == vehicle_sim_count("{[scan] stopped}")),
          "vehicle: STOP ends the scan at the next step");
    check((0u == vehicle_sim_count("L90")) && (0u == vehicle_sim_count("fwd")),
          "vehicle: no move after a stopped scan");
    check(CENTRE_US == vehicle_sim_servo_us(), "vehicle: sonar re-centred");
    vehicle_sim_step();
    check(vehicle_is(VS_STOPPED, "stop command"),
          "vehicle: scan stopped -> STOPPED (stop command)");

    vehicle_start(DRIVE_MS);
    vehicle_close_echo();
    vehicle_sim_step();
    pings = vehicle_sim_pings();
    vehicle_sim_stop_command_at(900u); /* during the second servo step */
    vehicle_sim_step();
    vehicle_sim_step();
    check((2u == (vehicle_sim_pings() - pings))
              && (vehicle_is(VS_STOPPED, "stop command")),
          "vehicle: stop command ends the scan at the next step");
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
    check((1u == vehicle_sim_count("L90")) && (0u == vehicle_sim_count("fwd"))
              && (vehicle_is(VS_STOPPED, "stop command"))
              && (0u == vehicle_sim_count("LINE_SEARCH")),
          "vehicle: STOP in a bypass reported once");
    vehicle_sim_box(false);
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
    vehicle_sim_step();
    check(NAV_RIGHT == vehicle_status().pending_nav,
          "vehicle: a barcode read during the run is kept");

    gap = vehicle_sim_watchdog_gap_ms();
    check((WATCHDOG_TIMEOUT_MS == vehicle_sim_watchdog_timeout_ms())
              && (gap < (WATCHDOG_TIMEOUT_MS / 2u)),
          "vehicle: watchdog fed within half its timeout");
    (void) printf("vehicle: buttons, scan, bypass, impact, hump, stop and "
                  "barcode scenarios; longest watchdog gap %" PRIu32 " ms\n",
                  gap);
}

/*** end of file ***/
