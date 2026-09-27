/** @file obstacle.c
 *
 * @brief Buddy 5 - scanning task and coarse-to-fine scan (see obstacle.h).
 *
 * Stage 1 coarse : 30, 60, 90, 120 and 150 degrees (median of 3 each).
 * Stage 2 fine   : +-OBST_FINE_PAD_DEG around the detected region in
 *                  OBST_FINE_STEP_DEG steps.
 * Stage 3 profile: avoidance_profile() on the fine points.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "car_math.h"
#include "servo.h"
#include "ultrasonic.h"
#include "obstacle.h"

#define TASK_PRIORITY     (9)
#define TASK_STACK        (2048)
#define MONITOR_PERIOD_MS (70u)
#define COARSE_START      (30)
#define COARSE_STEP       (30)
#define COARSE_END        (150)
#define FINE_MIN_DEG      (10)
#define FINE_MAX_DEG      (170)
#define MEDIAN_SAMPLES    (3u)
#define SWING_MS_PER_DEG  (3u) /* about 0.18 s per 60 degrees */

/* Written by the monitor and vehicle tasks, read by the vehicle and
   telemetry tasks: volatile (Rule 1.8.c). */
static volatile int32_t             g_front_mm = ULTRASONIC_NO_ECHO;
static volatile bool                gb_monitor = true;
static volatile avoidance_profile_t g_last;
static volatile uint32_t            g_scans = 0u;

static void     servo_go(int32_t angle_deg);
static uint32_t coarse_scan(avoidance_point_t * p_points, int32_t * p_low,
                            int32_t * p_high);
static uint32_t fine_scan(avoidance_point_t * p_points, int32_t low,
                          int32_t high);
static void     monitor_task(INT stacd, void * p_exinf);

/*!
 * @brief Initialise the servo and the ultrasonic sensor and start the
 *        front monitor task.
 *
 * @return E_OK, an ultrasonic_init() error or a tk_cre_tsk() error.
 */
int32_t
obstacle_init (void)
{
    T_CTSK  ctsk   = {0};
    ID      h_task = 0;
    int32_t ercd   = E_OK;

    servo_init();
    ercd = ultrasonic_init();

    ctsk.tskatr = TA_HLNG | TA_RNG3;

    /* Cast: the kernel stores every task entry as the generic FP type and
       calls a TA_HLNG task as (INT stacd, void * exinf), matching
       monitor_task(). */
    ctsk.task    = (FP) monitor_task;
    ctsk.itskpri = TASK_PRIORITY;
    ctsk.stksz   = TASK_STACK;
    h_task       = tk_cre_tsk(&ctsk);

    if (h_task >= E_OK)
    {
        (void) tk_sta_tsk(h_task, 0);
    }
    else
    {
        ercd = h_task;
    }

    return (ercd);
}

/*!
 * @brief Latest front distance from the monitor task.
 *
 * @return Distance in mm, or ULTRASONIC_NO_ECHO.
 */
int32_t
obstacle_front_mm (void)
{
    return (g_front_mm);
}

/*!
 * @brief Pause or resume the front monitor; clears the last reading.
 *
 * @param[in] b_on true to measure, false to pause.
 */
void
obstacle_monitor_enable (bool b_on)
{
    gb_monitor = b_on;
    g_front_mm = ULTRASONIC_NO_ECHO;
}

/*!
 * @brief Point the sensor and take a median reading.
 *
 * @param[in] servo_deg Servo angle, 0 (right) .. 180 (left).
 *
 * @return Distance in mm, or ULTRASONIC_NO_ECHO.
 */
int32_t
obstacle_look_mm (int32_t servo_deg)
{
    servo_go(servo_deg);

    return (ultrasonic_median_mm(MEDIAN_SAMPLES));
}

/*!
 * @brief Full adaptive scan (the car should be stopped).  Leaves the
 *        servo centred.
 *
 * @param[out] p_prof Obstacle profile.
 *
 * @return true if an obstacle was profiled.
 */
bool
obstacle_scan (avoidance_profile_t * p_prof)
{
    avoidance_point_t points[SCAN_MAX_POINTS];
    uint32_t          count   = 0u;
    int32_t           low     = FINE_MAX_DEG;
    int32_t           high    = FINE_MIN_DEG;
    bool              b_found = false;
    UINT              imask   = 0u;

    gb_monitor = false;
    count      = coarse_scan(points, &low, &high);

    /* Something in range: refine around it. */
    if (low <= high)
    {
        count = fine_scan(points, low, high);
    }

    b_found = avoidance_profile(points, count, p_prof);
    servo_go(SERVO_CENTRE_DEG);
    gb_monitor = true;

    DI(imask);
    g_last = *p_prof;
    g_scans++;
    EI(imask);

    /* Casts: tm_printf() takes the kernel's UB string type (the literal is
       ASCII); the values are small counts and distances in mm, so they fit
       in INT (the width is rounded to whole mm first). */
    (void) tm_printf((UB const *) "[scan] pts=%d found=%d d=%d w=%d cl=%d "
                                  "cr=%d\n",
                     (INT) count, b_found ? 1 : 0, (INT) p_prof->closest_mm,
                     (INT) car_math_round(p_prof->width_mm),
                     (INT) p_prof->clear_left_mm, (INT) p_prof->clear_right_mm);

    return (b_found);
}

/*!
 * @brief Consistent copy of the last scan result.
 *
 * @param[out] p_prof  Last obstacle profile.
 * @param[out] p_scans Number of scans since boot.
 */
void
obstacle_get_last (avoidance_profile_t * p_prof, uint32_t * p_scans)
{
    UINT imask = 0u;

    DI(imask);
    *p_prof  = g_last;
    *p_scans = g_scans;
    EI(imask);
}

/*!
 * @brief Move the servo and wait long enough for it to arrive.
 *
 * @param[in] angle_deg Servo angle, 0 (right) .. 180 (left).
 */
static void
servo_go (int32_t angle_deg)
{
    int32_t  delta     = angle_deg - servo_get_angle();
    uint32_t swing_deg = 0u;

    /* Cast: |delta| <= 180, so the non-negative value fits uint32_t. */
    swing_deg = (uint32_t) ((delta < 0) ? -delta : delta);
    servo_set_angle(angle_deg);
    (void) tk_dly_tsk(SERVO_SETTLE_MS + (swing_deg * SWING_MS_PER_DEG));
}

/*!
 * @brief Stage 1: coarse scan at five fixed angles.
 *
 * @param[out] p_points Scan points.
 * @param[out] p_low    Lowest angle with an echo closer than
 *                      OBST_DETECT_MM (unchanged if none).
 * @param[out] p_high   Highest such angle (unchanged if none).
 *
 * @return Number of points.
 */
static uint32_t
coarse_scan (avoidance_point_t * p_points, int32_t * p_low, int32_t * p_high)
{
    uint32_t count   = 0u;
    int32_t  angle   = 0;
    int32_t  dist_mm = ULTRASONIC_NO_ECHO;

    for (angle = COARSE_START; angle <= COARSE_END; angle += COARSE_STEP)
    {
        dist_mm = obstacle_look_mm(angle);

        if ((ULTRASONIC_NO_ECHO != dist_mm) && (dist_mm < OBST_DETECT_MM))
        {
            *p_low  = (angle < *p_low) ? angle : *p_low;
            *p_high = (angle > *p_high) ? angle : *p_high;
        }

        /* Casts: angles are 0..180 and distances at most US_MAX_MM (or
           -1), so both fit int16_t. */
        p_points[count].angle_deg = (int16_t) angle;
        p_points[count].dist_mm   = (int16_t) dist_mm;
        count++;
    }

    return (count);
}

/*!
 * @brief Stage 2: fine scan around the region found by the coarse scan.
 *
 * @param[out] p_points Scan points.
 * @param[in]  low      Lowest coarse angle with a close echo.
 * @param[in]  high     Highest coarse angle with a close echo.
 *
 * @return Number of points.
 */
static uint32_t
fine_scan (avoidance_point_t * p_points, int32_t low, int32_t high)
{
    uint32_t count = 0u;
    int32_t  first = low - OBST_FINE_PAD_DEG;
    int32_t  last  = high + OBST_FINE_PAD_DEG;
    int32_t  angle = 0;

    first = (first < FINE_MIN_DEG) ? FINE_MIN_DEG : first;
    last  = (last > FINE_MAX_DEG) ? FINE_MAX_DEG : last;

    for (angle = first; (angle <= last) && (count < SCAN_MAX_POINTS);
         angle += OBST_FINE_STEP_DEG)
    {
        /* Casts: as in coarse_scan(), both values fit int16_t. */
        p_points[count].angle_deg = (int16_t) angle;
        p_points[count].dist_mm   = (int16_t) obstacle_look_mm(angle);
        count++;
    }

    return (count);
}

/*!
 * @brief Background front-distance monitor (only while the servo is
 *        centred).
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
monitor_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        if (gb_monitor && (SERVO_CENTRE_DEG == servo_get_angle()))
        {
            g_front_mm = ultrasonic_measure_mm();
        }

        (void) tk_dly_tsk(MONITOR_PERIOD_MS);
    }
}

/*** end of file ***/
