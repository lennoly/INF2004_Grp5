/** @file subsystem5_test.c
 *
 * @brief Buddy 5 test program: full coarse-to-fine scan, obstacle profile
 *        and bypass plan (TEST_ULTRASONIC).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h" /* APP_MODE_* names: without it every #if is true */
#include "ultrasonic.h"
#include "obstacle.h"
#include "avoidance.h"
#include "app_tests.h"
#include "subsystem5_test.h"

#define A9_PINGS (10u)

#if APP_MODE == APP_MODE_TEST_ULTRASONIC
static void         a9_check(void);
static char const * side_name(int32_t angle_deg);

/*!
 * @brief Buddy 5 test: the A9 no-target check, then a full coarse-to-fine
 *        scan, profile and plan.  The raw scan points follow as
 *        "pt,angle,mm" lines for tools/scan_plot.py, and the side of the
 *        closest echo checks the servo direction (A9): a box ahead but
 *        offset to the car's right must be found on the RIGHT; otherwise
 *        set SERVO_INVERT in car_config.h.
 */
void
subsystem5_test_ultrasonic (void)
{
    avoidance_profile_t profile = {0};
    avoidance_plan_t    plan;
    avoidance_point_t   points[SCAN_MAX_POINTS];
    uint32_t            count = 0u;
    uint32_t            idx   = 0u;

    a9_check();

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
        (INT) profile.closest_angle, app_tests_print_int(profile.left_edge_mm),
        app_tests_print_int(profile.right_edge_mm),
        app_tests_print_int(profile.width_mm), (INT) profile.clear_left_mm,
        (INT) profile.clear_right_mm, avoidance_action_name(plan.action),
        app_tests_print_int(plan.offset_mm));

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
 * @brief Assumption A9 (CSC05 review: test it first): what this module
 *        gives at 3.3 V when nothing is in range.  Aim the sensor at open
 *        space beyond 4 m (or point it at the sky) before pressing START.
 *        A module that gives a long pulse (OUT_OF_RANGE) can treat
 *        no-target as free space; one that gives no pulse (FAULT) cannot,
 *        and then needs 5 V with a 1 k / 2 k divider on ECHO.
 */
static void
a9_check (void)
{
    ultrasonic_ping_t ping;
    uint32_t          counts[ULTRASONIC_FAULT + 1] = {0u, 0u, 0u};
    uint32_t          idx                          = 0u;

    /* Casts in this function: tm_printf() takes the kernel's UB string
       type (the literals are ASCII); pulse widths are below the 40 ms
       timeout and the counters below A9_PINGS, so they fit UINT. */
    (void) tm_printf((UB const *) "# A9 no-target check (aim at open "
                                  "space > 4 m): a9,class,width_us,mm\n");

    for (idx = 0u; idx < A9_PINGS; idx++)
    {
        ultrasonic_ping(&ping);
        counts[ping.result]++;
        (void) tm_printf((UB const *) "a9,%s,%u,%d\n",
                         ultrasonic_class_name(ping.result),
                         (UINT) ping.width_us, (INT) ping.dist_mm);
    }

    (void) tm_printf((UB const *) "# A9: VALID %u, OUT_OF_RANGE %u, FAULT %u "
                                  "(FAULT with no target: use 5 V + "
                                  "divider)\n",
                     (UINT) counts[ULTRASONIC_VALID],
                     (UINT) counts[ULTRASONIC_OUT_OF_RANGE],
                     (UINT) counts[ULTRASONIC_FAULT]);
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
