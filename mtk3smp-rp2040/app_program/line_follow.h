/** @file line_follow.h
 *
 * @brief Buddy 3 - line position estimate, PID steering and junction /
 *        line-lost detection for two edge-tracking IR sensors.
 *
 * Mounting: the two IR spots straddle the 18 mm line, one on each edge
 * (about 18 mm apart).  Centred, both read about 0.5.  The error is
 * left - right in [-1, 1]; positive means the line is under the left
 * sensor, so the car steers left.  Pure C (no RTOS calls), so it is unit
 * tested on the host.  Gains start from car_config.h and can be changed
 * live with line_follow_set_gains(); call it only from the task that owns
 * the follower (the vehicle task).
 */

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>
#include "car_types.h"
#include "pid.h"

typedef enum
{
    LINE_ON = 0,
    LINE_LOST,
    LINE_JUNCTION
} line_follow_state_t;

typedef struct
{
    pid_ctrl_t          pid;
    float32_t           error;           /* latest error                    */
    float32_t           last_seen_error; /* error when the line was visible */
    float32_t           junction_mm;     /* distance with both on black     */
    uint32_t            lost_ms;         /* time with both on white         */
    line_follow_state_t state;
} line_follow_t;

void line_follow_init(line_follow_t * p_follow);
void line_follow_set_gains(line_follow_t * p_follow, float32_t gain_p,
                           float32_t gain_i, float32_t gain_d);
void line_follow_get_gains(float32_t * p_gain_p, float32_t * p_gain_i,
                           float32_t * p_gain_d);
line_follow_state_t line_follow_classify(line_follow_t * p_follow,
                                         float32_t left, float32_t right,
                                         float32_t distance_mm,
                                         uint32_t  elapsed_ms);
float32_t           line_follow_steer(line_follow_t * p_follow, float32_t left,
                                      float32_t right, float32_t dt_s);
float32_t           line_follow_speed(float32_t cruise_mm_s, float32_t error);

#endif /* LINE_FOLLOW_H */

/*** end of file ***/
