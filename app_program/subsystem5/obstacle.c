/** @file obstacle.c
 *
 * @brief Buddy 5 - scanning task and coarse-to-fine scan (see obstacle.h).
 *
 * Stage 1 coarse : the front monitor, OBST_CONFIRM_READINGS close pings
 *                  straight ahead.  A separate 65/90/115 deg pass missed
 *                  an 8 cm box between those angles and let the car drive
 *                  on into it (25 Oct), so it was dropped.
 * Stage 2 fine   : the whole SERVO_MIN_DEG..SERVO_MAX_DEG range in
 *                  OBST_FINE_STEP_DEG steps (11 points, median of 3 each),
 *                  left to right (SERVO_MAX_DEG first).  The mount only
 *                  lets the sensor turn about 25 degrees each way, so the
 *                  brief's example angles (30 .. 150) cannot be reached.
 * Stage 3 profile: avoidance_profile() on the fine points.
 *
 * The caller's stop callback is polled every SETTLE_STEP_MS while the servo
 * settles and after every ping, so a STOP ends a scan within about one
 * ping (the callback also feeds the caller's watchdog).  The front monitor
 * reports an obstacle only after OBST_CONFIRM_READINGS close readings in a
 * row, so one spurious echo does not stop the car; it also counts FAULTs
 * in a row and time-stamps every reading, so the vehicle task can stop the
 * car if the sensor fails or the readings go stale.  The monitor and the
 * scans hand the sensor over through gb_monitor_busy (monitor_pause()), so
 * a "front" ping is never taken while the servo is moving.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "car_math.h"
#include "servo.h"
#include "ultrasonic.h"
#include "fault.h"
#include "packed.h"
#include "obstacle.h"

#define TASK_PRIORITY     (9)
#define TASK_STACK        (2048)
#define MONITOR_PERIOD_MS (70u)
#define FINE_MIN_DEG      (SERVO_MIN_DEG)
#define FINE_MAX_DEG      (SERVO_MAX_DEG)
#define FINE_POINTS_ALL \
    ((uint32_t) (((FINE_MAX_DEG - FINE_MIN_DEG) / OBST_FINE_STEP_DEG) + 1))
#define FINE_POINTS \
    ((FINE_POINTS_ALL < SCAN_MAX_POINTS) ? FINE_POINTS_ALL : SCAN_MAX_POINTS)
#define MEDIAN_SAMPLES    (3u)
#define SETTLE_STEP_MS    (20u) /* stop checks while the servo settles */
#define PAUSE_STEP_MS     (10u)
#define PAUSE_STEPS       (20u) /* wait at most 200 ms for the monitor */

/* Change Requirement 1: the close-echo and FAULT counts (0..3 each) share
   one byte, written whole by monitor_update() and obstacle_monitor_enable()
   and read whole by obstacle_ahead(). */
#define COUNT_BITS        (2u)
#define COUNT_MASK        (0x03u)
#define POINT_DIST_NONE   (0xFFFFu) /* stored scan point without an echo */

#if (OBST_CONFIRM_READINGS > COUNT_MASK) || (OBST_FAULT_LIMIT > COUNT_MASK)
#error "OBST_CONFIRM_READINGS and OBST_FAULT_LIMIT must fit in 2 bits"
#endif

/* Written by the monitor and vehicle tasks, read by the vehicle and
   telemetry tasks: volatile (Rule 1.8.c).  Change Requirement 1: the front
   reading is one packed 32-bit word (class, distance, time) stored with a
   single write, so the vehicle reads it without a lock; its time gives the
   reading's age.  The last profile is packed to 16 bytes. */
static volatile packed_front_t   g_front         = PACKED_FRONT_NONE;
static volatile uint8_t          g_counts        = 0u; /* see COUNT_BITS */
static volatile bool             gb_monitor      = true;
static volatile bool             gb_monitor_busy = false;
static volatile packed_profile_t g_last;
static volatile uint8_t          g_scans = 0u; /* wraps: readers test != */
/* Last scan points: angle 0..180 deg in 8 bits, distance 0..4000 mm in 16
   bits (POINT_DIST_NONE = no echo); 3 B per point instead of 4. */
static volatile uint8_t  g_last_angle[SCAN_MAX_POINTS];
static volatile uint16_t g_last_dist[SCAN_MAX_POINTS];
static volatile uint8_t  g_last_count = 0u;


static uint32_t       now_ms(void);
static packed_front_t no_reading_yet(void);
static void           monitor_pause(void);
static bool     servo_go(int32_t angle_deg, obstacle_stop_cb_t p_should_stop,
                         bool b_abortable);
static bool     look(int32_t angle_deg, obstacle_stop_cb_t p_should_stop,
                     int32_t * p_dist_mm);
static int32_t  median_mm(int32_t * p_values, uint32_t count);
static int32_t  step_toward(int32_t angle, int32_t target);
static bool     keep_scanning(obstacle_stop_cb_t p_should_stop);
static uint32_t count_up(uint32_t count, uint32_t limit);
static void     monitor_update(ultrasonic_ping_t const * p_ping);
static bool     fine_scan(avoidance_point_t * p_points, uint32_t * p_count,
                          obstacle_stop_cb_t p_should_stop);
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
    ercd    = ultrasonic_init();
    g_front = no_reading_yet();

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
    return (packed_front_mm(g_front));
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
    return ((g_counts & COUNT_MASK) >= OBST_CONFIRM_READINGS);
}

/*!
 * @brief Whether the front reading is out of date: the monitor is enabled
 *        but has not finished a ping for OBST_STALE_MS (a starved monitor
 *        task, a sensor held by another task, or a servo off centre).
 *
 * @return true if the latest reading is stale.
 */
bool
obstacle_front_stale (void)
{
    return (gb_monitor
            && (packed_front_age_ms(g_front, now_ms()) > OBST_STALE_MS));
}

/*!
 * @brief Pause or resume the front monitor.  Both clear the last reading,
 *        the counts and the reading age; pausing also waits for a monitor
 *        ping in progress, so the caller may move the servo afterwards.
 *
 * @param[in] b_on true to measure, false to pause.
 */
void
obstacle_monitor_enable (bool b_on)
{
    if (!b_on)
    {
        monitor_pause();
    }

    /* Start afresh: earlier readings must not confirm, fault or age the
       next ones.  The FAULT_NO_ECHO bit is left to the monitor, its only
       writer, which clears it at the next reading that is not a FAULT. */
    g_front    = no_reading_yet();
    g_counts   = 0u;
    gb_monitor = b_on;
}

/*!
 * @brief Full fine scan (the car should be stopped).  Leaves the
 *        servo centred.
 *
 * @param[out] p_prof        Obstacle profile; when the scan is stopped
 *                           early only b_found (false) and closest_mm are
 *                           written.
 * @param[in]  p_should_stop Polled every SETTLE_STEP_MS and after every
 *                           ping; true ends the scan.  NULL scans to the
 *                           end.
 *
 * @return true if an obstacle was profiled; false if none was found or the
 *         scan was stopped.
 */
bool
obstacle_scan (avoidance_profile_t * p_prof, obstacle_stop_cb_t p_should_stop)
{
    avoidance_point_t points[SCAN_MAX_POINTS];
    uint32_t          count      = 0u;
    bool              b_found    = false;
    bool              b_complete = false;
    uint32_t          idx        = 0u;
    UINT              imask      = 0u;
    packed_profile_t  packed;

    monitor_pause();
    b_complete = fine_scan(points, &count, p_should_stop);

    /* Always re-centre fully, so the monitor's pings point ahead. */
    (void) servo_go(SERVO_CENTRE_DEG, p_should_stop, false);
    obstacle_monitor_enable(true);

    if (b_complete)
    {
        b_found = avoidance_profile(points, count, p_prof);
        packed_profile_pack(p_prof, &packed);

        DI(imask);
        g_last = packed;

        for (idx = 0u; idx < count; idx++)
        {
            /* Casts: angles are 0..180 deg and echoes 0..US_MAX_MM, inside
               8 and 16 bits; a negative distance means no echo. */
            g_last_angle[idx] = (uint8_t) points[idx].angle_deg;
            g_last_dist[idx]  = (points[idx].dist_mm < 0)
                                    ? (uint16_t) POINT_DIST_NONE
                                    : (uint16_t) points[idx].dist_mm;
        }

        /* Cast: count is at most SCAN_MAX_POINTS (40). */
        g_last_count = (uint8_t) count;
        g_scans++;
        EI(imask);

        /* Casts: tm_printf() takes the kernel's UB string type (the literal
           is ASCII); the values are small counts and distances in mm, so
           they fit in INT (the width is rounded to whole mm first).
           w>= : an edge was not seen, the width is a lower bound. */
        (void) tm_printf(
            (UB const *) "[scan] pts=%d found=%d d=%d w%s%d cl=%d cr=%d\n",
            (INT) count, b_found ? 1 : 0, (INT) p_prof->closest_mm,
            avoidance_width_known(p_prof) ? "=" : ">=",
            (INT) car_math_round(p_prof->width_mm), (INT) p_prof->clear_left_mm,
            (INT) p_prof->clear_right_mm);

        /* Every point as angle:mm (-1 = no echo), so a wrong side choice
           can be traced to the readings.  Casts: as above. */
        (void) tm_printf((UB const *) "[scan] pts");

        for (idx = 0u; idx < count; idx++)
        {
            (void) tm_printf((UB const *) " %d:%d", (INT) points[idx].angle_deg,
                             (INT) points[idx].dist_mm);
        }

        (void) tm_printf((UB const *) "\n");
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
    packed_profile_t copy;
    UINT             imask = 0u;

    DI(imask);
    copy     = g_last;
    *p_scans = g_scans;
    EI(imask);

    packed_profile_unpack(&copy, p_prof);
}

/*!
 * @brief Consistent copy of the points of the last completed scan (printed
 *        by TEST_ULTRASONIC for tools/scan_plot.py).
 *
 * @param[out] p_points   Destination for up to max_points points.
 * @param[in]  max_points Capacity of p_points.
 *
 * @return Number of points copied.
 */
uint32_t
obstacle_get_points (avoidance_point_t * p_points, uint32_t max_points)
{
    uint32_t count = 0u;
    uint32_t idx   = 0u;
    UINT     imask = 0u;

    DI(imask);
    count = (g_last_count < max_points) ? g_last_count : max_points;

    for (idx = 0u; idx < count; idx++)
    {
        /* Casts: 0..180 and 0..US_MAX_MM fit int16_t. */
        p_points[idx].angle_deg = (int16_t) g_last_angle[idx];
        p_points[idx].dist_mm   = (POINT_DIST_NONE == g_last_dist[idx])
                                      ? (int16_t) ULTRASONIC_NO_ECHO
                                      : (int16_t) g_last_dist[idx];
    }

    EI(imask);

    return (count);
}

/*!
 * @brief The kernel's millisecond clock.
 *
 * @return Milliseconds since boot, modulo 2^32.
 */
static uint32_t
now_ms (void)
{
    SYSTIM tim = {0, 0u};

    (void) tk_get_tim(&tim);

    return (tim.lo);
}

/*!
 * @brief A front word that holds no reading yet but is dated now, so a
 *        monitor that has just started is not reported stale.
 *
 * @return Packed front word.
 */
static packed_front_t
no_reading_yet (void)
{
    return (packed_front_make(packed_front_class(PACKED_FRONT_NONE),
                              ULTRASONIC_NO_ECHO, now_ms()));
}

/*!
 * @brief Stop the front monitor and wait until no monitor ping is running
 *        (at most PAUSE_STEPS * PAUSE_STEP_MS).
 */
static void
monitor_pause (void)
{
    uint32_t steps = 0u;

    gb_monitor = false;

    /* The monitor raises gb_monitor_busy before it reads gb_monitor, so
       once the flag is down here no ping is running or can start. */
    while (gb_monitor_busy && (steps < PAUSE_STEPS))
    {
        (void) tk_dly_tsk(PAUSE_STEP_MS);
        steps++;
    }
}

/*!
 * @brief Move the servo gently, SERVO_STEP_DEG every SETTLE_STEP_MS so the
 *        sensor is not jerked on its mount, then wait SERVO_SETTLE_MS for
 *        it to stop shaking.  Calls the stop callback every SETTLE_STEP_MS.
 *
 * @param[in] angle_deg     Servo angle, 0 (right) .. 180 (left).
 * @param[in] p_should_stop Stop callback, or NULL.
 * @param[in] b_abortable   true: return as soon as a stop is requested;
 *                          false: always finish the move (re-centring).
 *
 * @return false if a stop was requested.
 */
static bool
servo_go (int32_t angle_deg, obstacle_stop_cb_t p_should_stop, bool b_abortable)
{
    int32_t  angle     = servo_get_angle();
    uint32_t settle_ms = 0u;
    bool     b_go      = true;

    while ((b_go || (!b_abortable))
           && ((angle != angle_deg) || (settle_ms < SERVO_SETTLE_MS)))
    {
        if (angle != angle_deg)
        {
            angle = step_toward(angle, angle_deg);
            servo_set_angle(angle);
        }
        else
        {
            settle_ms += SETTLE_STEP_MS;
        }

        (void) tk_dly_tsk(SETTLE_STEP_MS);
        b_go = (keep_scanning(p_should_stop)) && b_go;
    }

    return (b_go);
}

/*!
 * @brief One servo step towards a target angle.
 *
 * @param[in] angle  Current angle.
 * @param[in] target Target angle.
 *
 * @return The target, or SERVO_STEP_DEG closer to it.
 */
static int32_t
step_toward (int32_t angle, int32_t target)
{
    int32_t next = target;

    if ((target - angle) > SERVO_STEP_DEG)
    {
        next = angle + SERVO_STEP_DEG;
    }
    else if ((angle - target) > SERVO_STEP_DEG)
    {
        next = angle - SERVO_STEP_DEG;
    }
    else
    {
        /* Within one step: go straight there. */
    }

    return (next);
}

/*!
 * @brief Point the sensor and take the median of MEDIAN_SAMPLES valid
 *        pings, calling the stop callback while the servo settles and
 *        after each ping.
 *
 * @param[in]  angle_deg     Servo angle, 0 (right) .. 180 (left).
 * @param[in]  p_should_stop Stop callback, or NULL.
 * @param[out] p_dist_mm     Median in mm, or ULTRASONIC_NO_ECHO.
 *
 * @return false if a stop was requested.
 */
static bool
look (int32_t angle_deg, obstacle_stop_cb_t p_should_stop, int32_t * p_dist_mm)
{
    ultrasonic_ping_t ping;
    int32_t           readings[MEDIAN_SAMPLES];
    uint32_t          count = 0u;
    uint32_t          idx   = 0u;
    bool              b_go  = false;

    b_go = servo_go(angle_deg, p_should_stop, true);

    for (idx = 0u; b_go && (idx < MEDIAN_SAMPLES); idx++)
    {
        ultrasonic_ping(&ping);

        if ((ULTRASONIC_VALID == ping.result) && (ping.dist_mm >= US_MIN_MM))
        {
            readings[count] = ping.dist_mm;
            count++;
        }

        b_go = keep_scanning(p_should_stop);
    }

    *p_dist_mm = median_mm(readings, count);

    return (b_go);
}

/*!
 * @brief Median of a few readings (sorted in place, insertion sort).
 *
 * @param[in,out] p_values Readings.
 * @param[in]     count    Number of readings.
 *
 * @return Median, or ULTRASONIC_NO_ECHO if there are none.
 */
static int32_t
median_mm (int32_t * p_values, uint32_t count)
{
    uint32_t idx    = 0u;
    uint32_t pos    = 0u;
    int32_t  value  = 0;
    int32_t  result = ULTRASONIC_NO_ECHO;

    for (idx = 1u; idx < count; idx++)
    {
        value = p_values[idx];
        pos   = idx;

        while ((pos > 0u) && (p_values[pos - 1u] > value))
        {
            p_values[pos] = p_values[pos - 1u];
            pos--;
        }

        p_values[pos] = value;
    }

    if (count > 0u)
    {
        result = p_values[count / 2u];
    }

    return (result);
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
 * @brief Stage 2: fine scan of the whole range the mount allows.
 *
 * @param[out] p_points      Scan points.
 * @param[out] p_count       Number of points taken.
 * @param[in]  p_should_stop Stop callback, or NULL.
 *
 * @return false if the scan was stopped early.
 */
static bool
fine_scan (avoidance_point_t * p_points, uint32_t * p_count,
           obstacle_stop_cb_t p_should_stop)
{
    int32_t  angle   = 0;
    int32_t  dist_mm = ULTRASONIC_NO_ECHO;
    uint32_t points  = FINE_POINTS;
    uint32_t idx     = 0u;
    bool     b_go    = true;

    *p_count = 0u;

    /* Left to right; the points stay sorted by angle (right first) for
       avoidance_profile(), each in its own slot. */
    while (b_go && (points > 0u))
    {
        points--;

        /* Cast: points is at most SCAN_MAX_POINTS, exact in int32_t. */
        angle = FINE_MIN_DEG + ((int32_t) points * OBST_FINE_STEP_DEG);
        b_go  = look(angle, p_should_stop, &dist_mm);

        if (b_go)
        {
            idx = points;

            /* Casts: angles are 0..180 and distances at most US_MAX_MM (or
               -1), so both fit int16_t. */
            p_points[idx].angle_deg = (int16_t) angle;
            p_points[idx].dist_mm   = (int16_t) dist_mm;
            (*p_count)++;
        }
    }

    return (b_go);
}

/*!
 * @brief Count up to a limit.
 *
 * @param[in] count Current count.
 * @param[in] limit Largest value.
 *
 * @return count + 1, but at most limit.
 */
static uint32_t
count_up (uint32_t count, uint32_t limit)
{
    return ((count < limit) ? (count + 1u) : limit);
}

/*!
 * @brief Fold one monitor ping into the front state: the distance, the
 *        close readings and FAULTs in a row, and the time of the reading.
 *        A BUSY ping (the sensor was held by another task) tells nothing.
 *
 * @param[in] p_ping Ping result.
 */
static void
monitor_update (ultrasonic_ping_t const * p_ping)
{
    bool     b_valid = (ULTRASONIC_VALID == p_ping->result);
    bool     b_close = b_valid && (p_ping->dist_mm < OBST_DETECT_MM);
    bool     b_fault = (ULTRASONIC_FAULT == p_ping->result);
    uint32_t counts  = g_counts;
    uint32_t n_close = counts & COUNT_MASK;
    uint32_t n_fault = (counts >> COUNT_BITS) & COUNT_MASK;

    if (ULTRASONIC_BUSY != p_ping->result)
    {
        /* Any other reading restarts a count.  Both counts are worked out
           first and stored with one byte write. */
        n_close  = b_close ? count_up(n_close, OBST_CONFIRM_READINGS) : 0u;
        n_fault  = b_fault ? count_up(n_fault, OBST_FAULT_LIMIT) : 0u;
        /* Cast: both counts are at most 3 (checked above). */
        g_counts = (uint8_t) (n_close | (n_fault << COUNT_BITS));
        g_front = packed_front_make(
            p_ping->result, b_valid ? p_ping->dist_mm : ULTRASONIC_NO_ECHO,
            now_ms());

        /* Shared fault mask (fault.h): this task is the only writer of the
           no-echo bit. */
        if (n_fault >= OBST_FAULT_LIMIT)
        {
            fault_raise(FAULT_NO_ECHO);
        }
        else if (0u == n_fault)
        {
            fault_clear(FAULT_NO_ECHO);
        }
        else
        {
            /* One FAULT ping: wait for the next one. */
        }
    }
}

/*!
 * @brief Background front-distance monitor (only while enabled and the
 *        servo is centred).
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
monitor_task (INT stacd, void * p_exinf)
{
    ultrasonic_ping_t ping;

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        /* Raised before gb_monitor is read: see monitor_pause(). */
        gb_monitor_busy = true;

        if (gb_monitor && (SERVO_CENTRE_DEG == servo_get_angle()))
        {
            ultrasonic_ping(&ping);
            monitor_update(&ping);
        }

        gb_monitor_busy = false;
        (void) tk_dly_tsk(MONITOR_PERIOD_MS);
    }
}

/*** end of file ***/
