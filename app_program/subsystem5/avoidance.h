/** @file avoidance.h
 *
 * @brief Buddy 5 - obstacle profiling and avoidance planning from a servo
 *        scan (pure C, host-testable).
 *
 * Frame: angle 90 = straight ahead, more than 90 = left.  For a reading d
 * at angle a: lateral = d * cos(a) (positive = RIGHT of the car's
 * centre-line, negative = LEFT).
 */

#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"

#define SCAN_MAX_POINTS (40u)

typedef struct
{
    int16_t angle_deg;
    int16_t dist_mm; /* negative = no echo */
} avoidance_point_t;

typedef struct
{
    bool      b_found;
    int32_t   closest_mm;
    int32_t   closest_angle;
    float32_t left_edge_mm;  /* lateral of the left-most point (neg)  */
    float32_t right_edge_mm; /* lateral of the right-most point       */
    float32_t width_mm;
    float32_t centre_mm;      /* lateral centre                        */
    int32_t   clear_left_mm;  /* free gap beyond the left edge         */
    int32_t   clear_right_mm; /* free gap beyond the right edge        */
} avoidance_profile_t;

typedef enum
{
    AVOID_CONTINUE = 0, /* nothing in the car's corridor */
    AVOID_STOP,
    AVOID_TURN_LEFT, /* bypass on the left            */
    AVOID_TURN_RIGHT,
    AVOID_REVERSE /* back off and re-scan          */
} avoidance_action_t;

typedef struct
{
    avoidance_action_t action;
    float32_t          offset_mm; /* sideways shift needed for the bypass */
} avoidance_plan_t;

bool avoidance_profile(avoidance_point_t const * p_points, uint32_t count,
                       avoidance_profile_t * p_out);
avoidance_plan_t avoidance_plan(avoidance_profile_t const * p_prof,
                                uint32_t                    attempts);
char const *     avoidance_action_name(avoidance_action_t action);

#endif /* AVOIDANCE_H */

/*** end of file ***/
