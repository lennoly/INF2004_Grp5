/** @file obstacle.c
 *
 * @brief Buddy 5 - scanning task and coarse-to-fine scan (see obstacle.h).
 *
 * Stage 1 coarse : 30, 60, 90, 120 and 150 degrees (median of 3 each).
 * Stage 2 fine   : +-OBST_FINE_PAD_DEG around the detected region in
 *                  OBST_FINE_STEP_DEG steps.
 * Stage 3 profile: avoidance_profile() on the fine points.
 *
 * The caller's stop callback is polled before every servo step, so a STOP
 * ends a scan within one step.  The front monitor reports an obstacle only
 * after OBST_CONFIRM_READINGS close readings in a row, so one spurious
 * echo does not stop the car.
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
static volatile int32_t             g_front_mm    = ULTRASONIC_NO_ECHO;
static volatile uint32_t            g_close_count = 0u;
static volatile bool                gb_monitor    = true;
static volatile avoidance_profile_t g_last;
static volatile uint32_t            g_scans = 0u;

static void servo_go(int32_t angle_deg);
static bool keep_scanning(obstacle_stop_cb_t p_should_stop);
static bool coarse_scan(avoidance_point_t * p_points, uint32_t * p_count,
                        int32_t * p_low, int32_t * p_high,
                        obstacle_stop_cb_t p_should_stop);
static bool fine_scan(avoidance_point_t * p_points, uint32_t * p_count,
                      int32_t low, int32_t high,
                      obstacle_stop_cb_t p_should_stop);
static void monitor_task(INT stacd, void * p_exinf);

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
 * @brief Whether the front monitor has confirmed an obstacle ahead.
 *
 * @return true once OBST_CONFIRM_READINGS readings in a row were closer
 *         than OBST_DETECT_MM.
 */
bool
obstacle_ahead (void)
{
    return (g_close_count >= OBST_CONFIRM_READINGS);
}

/*!
 * @brief Pause or resume the front monitor; clears the last reading and
 *        the confirmation count.
 *
 * @param[in] b_on true to measure, false to pause.
 */
void
obstacle_monitor_enable (bool b_on)
{
    gb_monitor    = b_on;
    g_front_mm    = ULTRASONIC_NO_ECHO;
    g_close_count = 0u;
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
 * @param[out] p_prof        Obstacle profile; when the scan is stopped
 *                           early only b_found (false) and closest_mm are
 *                           written.
 * @param[in]  p_should_stop Polled before every servo step; true ends the
 *                           scan.  NULL scans to the end.
 *
 * @return true if an obstacle was profiled; false if none was found or the
 *         scan was stopped.
 */
bool
obstacle_scan (avoidance_profile_t * p_prof, obstacle_stop_cb_t p_should_stop)
{
    avoidance_point_t points[SCAN_MAX_POINTS];
    uint32_t          count      = 0u;
    int32_t           low        = FINE_MAX_DEG;
    int32_t           high       = FINE_MIN_DEG;
    bool              b_found    = false;
    bool              b_complete = false;
    UINT              imask      = 0u;

    gb_monitor = false;
    b_complete = coarse_scan(points, &count, &low, &high, p_should_stop);

    /* Something in range: refine around it. */
    if (b_complete && (low <= high))
    {
        b_complete = fine_scan(points, &count, low, high, p_should_stop);
    }

    servo_go(SERVO_CENTRE_DEG);

    /* Readings from before the scan must not confirm a new obstacle. */
    g_close_count = 0u;
    gb_monitor    = true;

    if (b_complete)
    {
        b_found = avoidance_profile(points, count, p_prof);

        DI(imask);
        g_last = *p_prof;
        g_scans++;
        EI(imask);

        /* Casts: tm_printf() takes the kernel's UB string type (the literal
           is ASCII); the values are small counts and distances in mm, so
           they fit in INT (the width is rounded to whole mm first). */
        (void) tm_printf(
            (UB const *) "[scan] pts=%d found=%d d=%d w=%d cl=%d cr=%d\n",
            (INT) count, b_found ? 1 : 0, (INT) p_prof->closest_mm,
            (INT) car_math_round(p_prof->width_mm), (INT) p_prof->clear_left_mm,
            (INT) p_prof->clear_right_mm);
    }
    else
    {
        /* Stopped early: a partial scan is neither profiled nor published
           as the last result. */
        p_prof->b_found    = false;
        p_prof->closest_mm = US_MAX_MM;

        /* Cast: tm_printf() takes the kernel's UB string type; ASCII. */
        (void) tm_printf((UB const *) "[scan] stopped\n");
    }

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
 * @brief Ask the caller whether the scan may go on.
 *
 * @param[in] p_should_stop Stop callback, or NULL (never stop).
 *
 * @return true to take the next scan point.
 */
static bool
keep_scanning (obstacle_stop_cb_t p_should_stop)
{
    return ((NULL == p_should_stop) || (!p_should_stop()));
}

/*!
 * @brief Stage 1: coarse scan at five fixed angles.
 *
 * @param[out] p_points      Scan points.
 * @param[out] p_count       Number of points taken.
 * @param[out] p_low         Lowest angle with an echo closer than
 *                           OBST_DETECT_MM (unchanged if none).
 * @param[out] p_high        Highest such angle (unchanged if none).
 * @param[in]  p_should_stop Stop callback, or NULL.
 *
 * @return false if the scan was stopped early.
 */
static bool
coarse_scan (avoidance_point_t * p_points, uint32_t * p_count, int32_t * p_low,
             int32_t * p_high, obstacle_stop_cb_t p_should_stop)
{
    int32_t angle   = 0;
    int32_t dist_mm = ULTRASONIC_NO_ECHO;
    bool    b_go    = true;

    *p_count = 0u;

    for (angle = COARSE_START; b_go && (angle <= COARSE_END);
         angle += COARSE_STEP)
    {
        b_go = keep_scanning(p_should_stop);

        if (b_go)
        {
            dist_mm = obstacle_look_mm(angle);

            if ((ULTRASONIC_NO_ECHO != dist_mm) && (dist_mm < OBST_DETECT_MM))
            {
                *p_low  = (angle < *p_low) ? angle : *p_low;
                *p_high = (angle > *p_high) ? angle : *p_high;
            }

            /* Casts: angles are 0..180 and distances at most US_MAX_MM (or
               -1), so both fit int16_t. */
            p_points[*p_count].angle_deg = (int16_t) angle;
            p_points[*p_count].dist_mm   = (int16_t) dist_mm;
            (*p_count)++;
        }
    }

    return (b_go);
}

/*!
 * @brief Stage 2: fine scan around the region found by the coarse scan.
 *
 * @param[out] p_points      Scan points.
 * @param[out] p_count       Number of points taken.
 * @param[in]  low           Lowest coarse angle with a close echo.
 * @param[in]  high          Highest coarse angle with a close echo.
 * @param[in]  p_should_stop Stop callback, or NULL.
 *
 * @return false if the scan was stopped early.
 */
static bool
fine_scan (avoidance_point_t * p_points, uint32_t * p_count, int32_t low,
           int32_t high, obstacle_stop_cb_t p_should_stop)
{
    int32_t first = low - OBST_FINE_PAD_DEG;
    int32_t last  = high + OBST_FINE_PAD_DEG;
    int32_t angle = 0;
    bool    b_go  = true;

    first    = (first < FINE_MIN_DEG) ? FINE_MIN_DEG : first;
    last     = (last > FINE_MAX_DEG) ? FINE_MAX_DEG : last;
    *p_count = 0u;

    for (angle = first; b_go && (angle <= last) && (*p_count < SCAN_MAX_POINTS);
         angle += OBST_FINE_STEP_DEG)
    {
        b_go = keep_scanning(p_should_stop);

        if (b_go)
        {
            /* Casts: as in coarse_scan(), both values fit int16_t. */
            p_points[*p_count].angle_deg = (int16_t) angle;
            p_points[*p_count].dist_mm   = (int16_t) obstacle_look_mm(angle);
            (*p_count)++;
        }
    }

    return (b_go);
}

/*!
 * @brief Background front-distance monitor (only while the servo is
 *        centred).  Counts close readings in a row for obstacle_ahead().
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
monitor_task (INT stacd, void * p_exinf)
{
    int32_t front_mm = ULTRASONIC_NO_ECHO;

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        if (gb_monitor && (SERVO_CENTRE_DEG == servo_get_angle()))
        {
            front_mm = ultrasonic_measure_mm();

            if ((ULTRASONIC_NO_ECHO != front_mm) && (front_mm < OBST_DETECT_MM))
            {
                g_close_count = (g_close_count < OBST_CONFIRM_READINGS)
                                    ? (g_close_count + 1u)
                                    : g_close_count;
            }
            else
            {
                /* A far reading or no echo restarts the count. */
                g_close_count = 0u;
            }

            g_front_mm = front_mm;
        }

        (void) tk_dly_tsk(MONITOR_PERIOD_MS);
    }
}

/*** end of file ***/
