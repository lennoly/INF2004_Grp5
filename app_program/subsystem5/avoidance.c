/** @file avoidance.c
 *
 * @brief Buddy 5 - obstacle profile generator and avoidance planner
 *        (see avoidance.h).
 */

#include <stdint.h>
#include <stdbool.h>
#include "car_config.h"
#include "car_math.h"
#include "avoidance.h"

#define CAR_HALF_MM      (CAR_WIDTH_MM / 2.0f)
#define NEIGHBOUR_RANGE  (2) /* other objects within 2x cluster depth */
#define CORRIDOR_HALF_MM (CAR_HALF_MM + OBST_MARGIN_MM)

/* Names indexed by avoidance_action_t. */
static char const * const g_action_names[] = {"CONTINUE", "STOP", "TURN_LEFT",
                                              "TURN_RIGHT", "REVERSE"};

static bool      point_is_valid(avoidance_point_t const * p_point);
static float32_t point_lateral(avoidance_point_t const * p_point);

static int32_t find_closest(avoidance_point_t const * p_points, uint32_t count,
                            uint32_t * p_best);

static void grow_cluster(avoidance_point_t const * p_points, uint32_t count,
                         int32_t limit_mm, uint32_t * p_low, uint32_t * p_high);

static void measure_extent(avoidance_point_t const * p_points, uint32_t low,
                           uint32_t high, avoidance_profile_t * p_out);

static void measure_clearance(avoidance_point_t const * p_points,
                              uint32_t count, uint32_t low, uint32_t high,
                              avoidance_profile_t * p_out);

static avoidance_plan_t choose_side(avoidance_profile_t const * p_prof,
                                    uint32_t                    attempts);

/*!
 * @brief Build the obstacle profile from one scan.
 *
 * The obstacle is the cluster of neighbouring points within
 * OBST_CLUSTER_DEPTH_MM of the closest echo inside the car's corridor.
 *
 * @param[in]  p_points Scan points, sorted by angle.
 * @param[in]  count    Number of points.
 * @param[out] p_out    Profile; only b_found and closest_mm are written
 *                      when nothing is found.
 *
 * @return true if an obstacle closer than OBST_DETECT_MM was found.
 */
bool
avoidance_profile (avoidance_point_t const * p_points, uint32_t count,
                   avoidance_profile_t * p_out)
{
    uint32_t best = 0u;
    uint32_t low  = 0u;
    uint32_t high = 0u;

    p_out->closest_mm = find_closest(p_points, count, &best);
    p_out->b_found    = (0u != count) && (p_out->closest_mm < OBST_DETECT_MM);

    if (p_out->b_found)
    {
        low  = best;
        high = best;
        grow_cluster(p_points, count, p_out->closest_mm + OBST_CLUSTER_DEPTH_MM,
                     &low, &high);
        p_out->closest_angle = p_points[best].angle_deg;
        measure_extent(p_points, low, high, p_out);
        measure_clearance(p_points, count, low, high, p_out);
    }

    return (p_out->b_found);
}

/*!
 * @brief Choose the avoidance action for a profile.
 *
 * @param[in] p_prof   Obstacle profile from avoidance_profile().
 * @param[in] attempts Avoidance attempts already made at this obstacle.
 *
 * @return Action and, for a bypass, the sideways shift needed.
 */
avoidance_plan_t
avoidance_plan (avoidance_profile_t const * p_prof, uint32_t attempts)
{
    avoidance_plan_t plan      = {AVOID_CONTINUE, 0.0f};
    bool             b_in_path = false;

    b_in_path = (p_prof->b_found) && (p_prof->left_edge_mm <= CORRIDOR_HALF_MM)
                && (p_prof->right_edge_mm >= -CORRIDOR_HALF_MM);

    if (b_in_path && (p_prof->closest_mm < OBST_TOO_CLOSE_MM))
    {
        /* Too close to turn: back off and re-scan, or give up. */
        plan.action =
            (attempts < OBST_MAX_ATTEMPTS) ? AVOID_REVERSE : AVOID_STOP;
    }
    else if (b_in_path)
    {
        plan = choose_side(p_prof, attempts);
    }
    else
    {
        /* Nothing in the car's corridor: keep going. */
    }

    return (plan);
}

/*!
 * @brief Printable name of an avoidance action.
 *
 * @param[in] action Avoidance action.
 *
 * @return Upper-case name, or "?" for an out-of-range value.
 */
char const *
avoidance_action_name (avoidance_action_t action)
{
    /* Cast: enumeration constants are non-negative, so the conversion to
       an unsigned index keeps their value; the range is checked below. */
    uint32_t     index  = (uint32_t) action;
    char const * p_name = "?";

    if (index < (sizeof(g_action_names) / sizeof(g_action_names[0])))
    {
        p_name = g_action_names[index];
    }

    return (p_name);
}

/*!
 * @brief Whether a scan point holds a usable echo.
 *
 * @param[in] p_point Scan point.
 *
 * @return true if the distance is inside the sensor's range.
 */
static bool
point_is_valid (avoidance_point_t const * p_point)
{
    return ((p_point->dist_mm >= US_MIN_MM) && (p_point->dist_mm <= US_MAX_MM));
}

/*!
 * @brief Sideways position of a scan point.
 *
 * @param[in] p_point Scan point.
 *
 * @return Lateral offset in mm, positive to the right of the car.
 */
static float32_t
point_lateral (avoidance_point_t const * p_point)
{
    /* Casts: int16_t values convert exactly to float32_t. */
    return (
        (float32_t) p_point->dist_mm
        * car_math_cos((float32_t) p_point->angle_deg * CAR_MATH_RAD_PER_DEG));
}

/*!
 * @brief Find the closest valid echo inside the car's corridor.
 *
 * Side walls and the like are used for clearance only: a closer wall must
 * not hide the object that is actually in the car's path.
 *
 * @param[in]  p_points Scan points.
 * @param[in]  count    Number of points.
 * @param[out] p_best   Index of the closest point (unchanged if none).
 *
 * @return Distance of the closest point, or US_MAX_MM if there is none.
 */
static int32_t
find_closest (avoidance_point_t const * p_points, uint32_t count,
              uint32_t * p_best)
{
    int32_t  closest_mm = US_MAX_MM;
    uint32_t idx        = 0u;

    for (idx = 0u; idx < count; idx++)
    {
        if ((point_is_valid(&p_points[idx]))
            && (p_points[idx].dist_mm < closest_mm)
            && (car_math_abs(point_lateral(&p_points[idx]))
                <= CORRIDOR_HALF_MM))
        {
            closest_mm = p_points[idx].dist_mm;
            *p_best    = idx;
        }
    }

    return (closest_mm);
}

/*!
 * @brief Grow the cluster left and right while the points stay close.
 *
 * @param[in]     p_points Scan points.
 * @param[in]     count    Number of points.
 * @param[in]     limit_mm Farthest distance still part of the obstacle.
 * @param[in,out] p_low    First index of the cluster.
 * @param[in,out] p_high   Last index of the cluster.
 */
static void
grow_cluster (avoidance_point_t const * p_points, uint32_t count,
              int32_t limit_mm, uint32_t * p_low, uint32_t * p_high)
{
    while ((*p_low > 0u) && (point_is_valid(&p_points[*p_low - 1u]))
           && (p_points[*p_low - 1u].dist_mm <= limit_mm))
    {
        (*p_low)--;
    }

    while (((*p_high + 1u) < count) && (point_is_valid(&p_points[*p_high + 1u]))
           && (p_points[*p_high + 1u].dist_mm <= limit_mm))
    {
        (*p_high)++;
    }
}

/*!
 * @brief Width and centre of the obstacle cluster.
 *
 * @param[in]     p_points Scan points.
 * @param[in]     low      First index of the cluster.
 * @param[in]     high     Last index of the cluster.
 * @param[out]    p_out    Profile (edges, width and centre written).
 */
static void
measure_extent (avoidance_point_t const * p_points, uint32_t low, uint32_t high,
                avoidance_profile_t * p_out)
{
    uint32_t  idx     = 0u;
    float32_t lateral = 0.0f;

    p_out->left_edge_mm  = point_lateral(&p_points[low]);
    p_out->right_edge_mm = p_out->left_edge_mm;

    for (idx = low; idx <= high; idx++)
    {
        lateral = point_lateral(&p_points[idx]);
        p_out->left_edge_mm =
            (lateral < p_out->left_edge_mm) ? lateral : p_out->left_edge_mm;
        p_out->right_edge_mm =
            (lateral > p_out->right_edge_mm) ? lateral : p_out->right_edge_mm;
    }

    p_out->width_mm  = p_out->right_edge_mm - p_out->left_edge_mm;
    p_out->centre_mm = 0.5f * (p_out->left_edge_mm + p_out->right_edge_mm);
}

/*!
 * @brief Clearance = gap to any other object at a similar range.
 *
 * Points after the cluster (higher angles) are to its left, points before
 * it are to its right.
 *
 * @param[in]     p_points Scan points.
 * @param[in]     count    Number of points.
 * @param[in]     low      First index of the cluster.
 * @param[in]     high     Last index of the cluster.
 * @param[in,out] p_out    Profile (edges set).
 */
static void
measure_clearance (avoidance_point_t const * p_points, uint32_t count,
                   uint32_t low, uint32_t high, avoidance_profile_t * p_out)
{
    int32_t   near_limit = 0;
    uint32_t  idx        = 0u;
    float32_t gap_left   = 0.0f;
    float32_t gap_right  = 0.0f;
    bool      b_other    = false;

    near_limit = p_out->closest_mm + (NEIGHBOUR_RANGE * OBST_CLUSTER_DEPTH_MM);
    p_out->clear_left_mm  = OBST_MAX_CLEARANCE_MM;
    p_out->clear_right_mm = OBST_MAX_CLEARANCE_MM;

    for (idx = 0u; idx < count; idx++)
    {
        b_other = (point_is_valid(&p_points[idx]))
                  && (p_points[idx].dist_mm <= near_limit)
                  && ((idx < low) || (idx > high));
        gap_left  = p_out->left_edge_mm - point_lateral(&p_points[idx]);
        gap_right = point_lateral(&p_points[idx]) - p_out->right_edge_mm;

        /* Casts: clearances are at most OBST_MAX_CLEARANCE_MM, so they
           convert exactly to float32_t. */
        if (b_other && (idx > high)
            && (gap_left < (float32_t) p_out->clear_left_mm))
        {
            p_out->clear_left_mm = car_math_round(gap_left);
        }

        if (b_other && (idx < low)
            && (gap_right < (float32_t) p_out->clear_right_mm))
        {
            p_out->clear_right_mm = car_math_round(gap_right);
        }
    }
}

/*!
 * @brief Pick the feasible side needing the smaller sideways shift (the
 *        minimum deviation from the original route).
 *
 * @param[in] p_prof   Obstacle profile (obstacle in the car's path).
 * @param[in] attempts Avoidance attempts already made at this obstacle.
 *
 * @return Action and sideways shift.
 */
static avoidance_plan_t
choose_side (avoidance_profile_t const * p_prof, uint32_t attempts)
{
    avoidance_plan_t plan       = {AVOID_CONTINUE, 0.0f};
    float32_t        need       = CAR_WIDTH_MM + OBST_MARGIN_MM;
    float32_t        off_left   = -p_prof->left_edge_mm + CORRIDOR_HALF_MM;
    float32_t        off_right  = p_prof->right_edge_mm + CORRIDOR_HALF_MM;
    bool             b_left_ok  = false;
    bool             b_right_ok = false;

    /* Casts: clearances are at most OBST_MAX_CLEARANCE_MM, so they convert
       exactly to float32_t. */
    b_left_ok = (car_math_is_finite(off_left))
                && ((float32_t) p_prof->clear_left_mm >= need);
    b_right_ok = (car_math_is_finite(off_right))
                 && ((float32_t) p_prof->clear_right_mm >= need);

    if (b_left_ok && ((!b_right_ok) || (off_left <= off_right)))
    {
        plan.action    = AVOID_TURN_LEFT;
        plan.offset_mm = off_left;
    }
    else if (b_right_ok)
    {
        plan.action    = AVOID_TURN_RIGHT;
        plan.offset_mm = off_right;
    }
    else
    {
        plan.action =
            (attempts < OBST_MAX_ATTEMPTS) ? AVOID_REVERSE : AVOID_STOP;
    }

    return (plan);
}

/*** end of file ***/
