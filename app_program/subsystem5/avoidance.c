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

static void merge_near(avoidance_point_t const * p_points, uint32_t count,
                       int32_t near_mm, uint32_t * p_low, uint32_t * p_high);

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
 * @param[out] p_out    Profile; when nothing is found every field but
 *                      closest_mm is zero (no stale or random edges).
 *
 * @return true if anything echoed in the corridor (closer than OBST_FLOOR_MM).
 *         Not OBST_DETECT_MM: after a REVERSE the box is often just past
 *         it (221 mm, 25 Oct) and the car drove back into it.
 */
bool
avoidance_profile (avoidance_point_t const * p_points, uint32_t count,
                   avoidance_profile_t * p_out)
{
    avoidance_profile_t const empty = {false, 0, 0,    0.0f, 0.0f,
                                       0.0f,  0.0f, 0, 0};
    uint32_t                  best  = 0u;
    uint32_t                  low   = 0u;
    uint32_t                  high  = 0u;

    /* Scans printed cl=70647 when nothing was found: start from zero. */
    *p_out            = empty;
    p_out->closest_mm = find_closest(p_points, count, &best);
    p_out->b_found    = (0u != count) && (p_out->closest_mm < OBST_FLOOR_MM);

    if (p_out->b_found)
    {
        low  = best;
        high = best;
        grow_cluster(p_points, count, p_out->closest_mm + OBST_CLUSTER_DEPTH_MM,
                     &low, &high);
        merge_near(p_points, count,
                   p_out->closest_mm
                       + (NEIGHBOUR_RANGE * OBST_CLUSTER_DEPTH_MM),
                   &low, &high);
        p_out->closest_angle = p_points[best].angle_deg;
        measure_extent(p_points, low, high, p_out);
        measure_clearance(p_points, count, low, high, p_out);

        /* An edge needs OBST_EDGE_POINTS points past it.  One nearer the
           end of the sweep is where the sweep ends, not the obstacle
           (the mount turns only 25 degrees each way): no known room on
           that side, so no pass there.  Neither side seen:
           avoidance_plan() passes wide on OBST_BLIND_LEFT's side. */
        p_out->clear_left_mm  = ((high + OBST_EDGE_POINTS) >= count)
                                    ? 0
                                    : p_out->clear_left_mm;
        p_out->clear_right_mm =
            (low < OBST_EDGE_POINTS) ? 0 : p_out->clear_right_mm;
    }

    return (p_out->b_found);
}

/*!
 * @brief Choose the avoidance action for a profile.
 *
 * @param[in] p_prof   Obstacle profile from avoidance_profile().
 * @param[in] attempts Avoidance attempts already made at this obstacle.
 *
 * @return Action and, for a bypass, the sideways shift needed and the
 *         distance to drive beside the obstacle.
 */
avoidance_plan_t
avoidance_plan (avoidance_profile_t const * p_prof, uint32_t attempts)
{
    avoidance_plan_t plan      = {AVOID_CONTINUE, 0.0f, 0.0f};
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
 * @brief Whether the profile's width is a measurement: both box edges were
 *        seen.  An edge at the end of the sweep has clearance 0, and the
 *        width is then only the sweep's reach (S5-U07, 25 Oct: both boxes
 *        read 170-210 mm), a lower bound.
 *
 * @param[in] p_prof Obstacle profile.
 *
 * @return true if both edges were seen.
 */
bool
avoidance_width_known (avoidance_profile_t const * p_prof)
{
    return ((p_prof->b_found) && (0 != p_prof->clear_left_mm)
            && (0 != p_prof->clear_right_mm));
}

/*!
 * @brief Whether a scan point holds a usable echo.
 *
 * @param[in] p_point Scan point.
 *
 * @return true if the distance is inside the sensor's range and closer
 *         than the floor (OBST_FLOOR_MM): the sensor cannot tilt up, and
 *         the floor echo looked like an object beside the box.
 */
static bool
point_is_valid (avoidance_point_t const * p_point)
{
    return ((p_point->dist_mm >= US_MIN_MM)
            && (p_point->dist_mm < OBST_FLOOR_MM));
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
 * @brief Join echoes the car could not fit between to the cluster.  Slant
 *        echoes off one box past OBST_FLOOR_MM split it into pieces 37 mm
 *        apart, which read as no room either side (REVERSE loop, 25 Oct).
 *
 * @param[in]     p_points Scan points.
 * @param[in]     count    Number of points.
 * @param[in]     near_mm  Farthest distance of another object nearby (as
 *                         in measure_clearance()).
 * @param[in,out] p_low    First index of the cluster.
 * @param[in,out] p_high   Last index of the cluster.
 */
static void
merge_near (avoidance_point_t const * p_points, uint32_t count, int32_t near_mm,
            uint32_t * p_low, uint32_t * p_high)
{
    float32_t const need = CAR_WIDTH_MM + OBST_MARGIN_MM;
    uint32_t        idx  = 0u;

    /* Lower indices are to the right (larger lateral), higher to the left;
       each joined echo becomes the end the next one is measured from. */
    for (idx = *p_low; idx > 0u; idx--)
    {
        if ((point_is_valid(&p_points[idx - 1u]))
            && (p_points[idx - 1u].dist_mm <= near_mm)
            && ((point_lateral(&p_points[idx - 1u])
                 - point_lateral(&p_points[*p_low])) < need))
        {
            *p_low = idx - 1u;
        }
    }

    for (idx = *p_high + 1u; idx < count; idx++)
    {
        if ((point_is_valid(&p_points[idx]))
            && (p_points[idx].dist_mm <= near_mm)
            && ((point_lateral(&p_points[*p_high])
                 - point_lateral(&p_points[idx]))
                < need))
        {
            *p_high = idx;
        }
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
    float32_t beam_mm = 0.0f;

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

    p_out->centre_mm = 0.5f * (p_out->left_edge_mm + p_out->right_edge_mm);

    /* The cone echoes past each real edge, so each seen edge is pulled in
       by closest x tan(beam) (a box that read 78 mm too wide made the car
       swing that much further, 25 Oct); never past the centre.  Cast:
       closest_mm is below US_MAX_MM. */
    beam_mm = (float32_t) p_out->closest_mm
              * (car_math_sin(OBST_BEAM_HALF_DEG * CAR_MATH_RAD_PER_DEG)
                 / car_math_cos(OBST_BEAM_HALF_DEG * CAR_MATH_RAD_PER_DEG));
    beam_mm = (beam_mm < (p_out->centre_mm - p_out->left_edge_mm))
                  ? beam_mm
                  : (p_out->centre_mm - p_out->left_edge_mm);
    p_out->left_edge_mm  += beam_mm;
    p_out->right_edge_mm -= beam_mm;
    p_out->width_mm = p_out->right_edge_mm - p_out->left_edge_mm;
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
 *        minimum deviation from the original route).  The sensor mount
 *        allows only SERVO_MIN_DEG..SERVO_MAX_DEG, too little to watch the
 *        obstacle go by, so its depth is unknown: the car passes the
 *        closest echo plus the deepest expected obstacle, its own length
 *        and the margin.
 *
 * @param[in] p_prof   Obstacle profile (obstacle in the car's path).
 * @param[in] attempts Avoidance attempts already made at this obstacle.
 *
 * @return Action, sideways shift and pass distance.
 */
static avoidance_plan_t
choose_side (avoidance_profile_t const * p_prof, uint32_t attempts)
{
    avoidance_plan_t plan       = {AVOID_CONTINUE, 0.0f, 0.0f};
    float32_t        need       = CAR_WIDTH_MM + OBST_MARGIN_MM;
    float32_t        off_left   = -p_prof->left_edge_mm + CORRIDOR_HALF_MM;
    float32_t        off_right  = p_prof->right_edge_mm + CORRIDOR_HALF_MM;
    float32_t        pass       = 0.0f;
    bool             b_left_ok  = false;
    bool             b_right_ok = false;
    bool             b_unsure   = false;
    bool             b_prefer_l = false;

    /* Casts: clearances are at most OBST_MAX_CLEARANCE_MM and the closest
       echo at most US_MAX_MM, so they convert exactly to float32_t. */
    pass = (float32_t) p_prof->closest_mm + OBST_DEPTH_MAX_MM + CAR_LENGTH_MM
           + OBST_MARGIN_MM;
    b_left_ok = (car_math_is_finite(off_left))
                && ((float32_t) p_prof->clear_left_mm >= need);
    b_right_ok = (car_math_is_finite(off_right))
                 && ((float32_t) p_prof->clear_right_mm >= need);

    /* Near the centre, or seen narrower than any real obstacle, the seen
       side cannot be trusted: shift wide, away from the way the box leans
       (OBST_BLIND_LEFT's side if it is dead centre), so a wrong lean still
       clears it.  Any side pass shifts at least OBST_MIN_OFFSET_MM. */
    b_unsure   = (car_math_abs(p_prof->centre_mm) < OBST_CENTRE_UNSURE_MM)
               || (p_prof->width_mm < OBST_WIDTH_MIN_MM);
    b_prefer_l = b_unsure ? ((p_prof->centre_mm > 0.0f)
                             || ((!(p_prof->centre_mm < 0.0f))
                                 && (0 != OBST_BLIND_LEFT)))
                          : (off_left <= off_right);
    off_left   = b_unsure ? OBST_BLIND_OFFSET_MM : off_left;
    off_right  = b_unsure ? OBST_BLIND_OFFSET_MM : off_right;
    off_left   = (off_left > OBST_MIN_OFFSET_MM) ? off_left
                                                 : OBST_MIN_OFFSET_MM;
    off_right  = (off_right > OBST_MIN_OFFSET_MM) ? off_right
                                                  : OBST_MIN_OFFSET_MM;

    if (b_left_ok && ((!b_right_ok) || b_prefer_l))
    {
        plan.action    = AVOID_TURN_LEFT;
        plan.offset_mm = off_left;
        plan.pass_mm   = pass;
    }
    else if (b_right_ok)
    {
        plan.action    = AVOID_TURN_RIGHT;
        plan.offset_mm = off_right;
        plan.pass_mm   = pass;
    }
    else if ((0 == p_prof->clear_left_mm) && (0 == p_prof->clear_right_mm))
    {
        /* Neither edge seen: wide enough for the largest box either way;
           the front sensor still watches the pass. */
#if OBST_BLIND_LEFT
        plan.action = AVOID_TURN_LEFT;
#else
        plan.action = AVOID_TURN_RIGHT;
#endif
        plan.offset_mm = OBST_BLIND_OFFSET_MM;
        plan.pass_mm   = pass;
    }
    else
    {
        plan.action =
            (attempts < OBST_MAX_ATTEMPTS) ? AVOID_REVERSE : AVOID_STOP;
    }

    return (plan);
}

/*** end of file ***/
