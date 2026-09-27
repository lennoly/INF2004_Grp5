/** @file line_follow.c
 *
 * @brief Buddy 3 - line-following algorithm (see line_follow.h).
 */

#include <stdbool.h>
#include <stddef.h>
#include "car_config.h"
#include "car_math.h"
#include "line_follow.h"

#define STEER_LIMIT_MM_S   (250.0f)
#define SPEED_DROP_AT_EDGE (0.5f) /* slow to 50 % at full error */

/* Active gains: compile-time defaults, changed live by the "line="
   command.  Written by the vehicle task and read by telemetry: volatile. */
static volatile float32_t g_gain_p = LINE_KP;
static volatile float32_t g_gain_i = LINE_KI;
static volatile float32_t g_gain_d = LINE_KD;

/*!
 * @brief Reset a follower and load the active steering gains.
 *
 * @param[out] p_follow Follower state to initialise.
 */
void
line_follow_init (line_follow_t * p_follow)
{
    pid_init(&p_follow->pid, g_gain_p, g_gain_i, g_gain_d, 0.0f,
             -STEER_LIMIT_MM_S, STEER_LIMIT_MM_S);
    p_follow->error           = 0.0f;
    p_follow->last_seen_error = 0.0f;
    p_follow->junction_mm     = 0.0f;
    p_follow->lost_ms         = 0u;
    p_follow->state           = LINE_ON;
}

/*!
 * @brief Change the steering gains.
 *
 * The follower's running PID is updated in place (no reset, so there is
 * no steering jump) and the new values are kept for every later
 * line_follow_init().
 *
 * @param[in,out] p_follow Running follower, or NULL to change only the
 *                         stored defaults.
 * @param[in]     gain_p   Proportional gain.
 * @param[in]     gain_i   Integral gain.
 * @param[in]     gain_d   Derivative gain.
 */
void
line_follow_set_gains (line_follow_t * p_follow, float32_t gain_p,
                       float32_t gain_i, float32_t gain_d)
{
    g_gain_p = gain_p;
    g_gain_i = gain_i;
    g_gain_d = gain_d;

    if (NULL != p_follow)
    {
        p_follow->pid.gain_p = gain_p;
        p_follow->pid.gain_i = gain_i;
        p_follow->pid.gain_d = gain_d;
    }
}

/*!
 * @brief Read the active steering gains.
 *
 * @param[out] p_gain_p Proportional gain.
 * @param[out] p_gain_i Integral gain.
 * @param[out] p_gain_d Derivative gain.
 */
void
line_follow_get_gains (float32_t * p_gain_p, float32_t * p_gain_i,
                       float32_t * p_gain_d)
{
    *p_gain_p = g_gain_p;
    *p_gain_i = g_gain_i;
    *p_gain_d = g_gain_d;
}

/*!
 * @brief Update the line state.
 *
 * LOST needs LINE_LOST_CONFIRM_MS of both sensors on white; JUNCTION needs
 * LINE_JUNCTION_MIN_MM of travel with both on black, so a single noisy
 * sample cannot trigger a turn.
 *
 * @param[in,out] p_follow    Follower state.
 * @param[in]     left        Left sensor, 0 (white) .. 1 (black).
 * @param[in]     right       Right sensor, 0 (white) .. 1 (black).
 * @param[in]     distance_mm Distance travelled since the last call.
 * @param[in]     elapsed_ms  Time since the last call.
 *
 * @return The new line state.
 */
line_follow_state_t
line_follow_classify (line_follow_t * p_follow, float32_t left, float32_t right,
                      float32_t distance_mm, uint32_t elapsed_ms)
{
    bool b_both_white = (left < LINE_LOST_LEVEL) && (right < LINE_LOST_LEVEL);
    bool b_both_black =
        (left > LINE_JUNCTION_LEVEL) && (right > LINE_JUNCTION_LEVEL);

    p_follow->lost_ms = b_both_white ? (p_follow->lost_ms + elapsed_ms) : 0u;
    p_follow->junction_mm =
        b_both_black ? (p_follow->junction_mm + car_math_abs(distance_mm))
                     : 0.0f;

    if (p_follow->lost_ms >= LINE_LOST_CONFIRM_MS)
    {
        p_follow->state = LINE_LOST;
    }
    else if (p_follow->junction_mm >= LINE_JUNCTION_MIN_MM)
    {
        p_follow->state = LINE_JUNCTION;
    }
    else
    {
        p_follow->state = LINE_ON;
    }

    if (!b_both_white)
    {
        p_follow->last_seen_error = left - right;
    }

    return (p_follow->state);
}

/*!
 * @brief Differential steering speed: vL = v - s, vR = v + s.
 *
 * A lost line is not steered: the vehicle controller searches for it.
 *
 * @param[in,out] p_follow Follower state.
 * @param[in]     left     Left sensor, 0 (white) .. 1 (black).
 * @param[in]     right    Right sensor, 0 (white) .. 1 (black).
 * @param[in]     dt_s     Time since the last call in seconds.
 *
 * @return Steering speed s in mm/s; positive turns left.
 */
float32_t
line_follow_steer (line_follow_t * p_follow, float32_t left, float32_t right,
                   float32_t dt_s)
{
    p_follow->error = left - right;

    /* Set-point 0 and measurement -error, so the output grows with a
       positive error.  pid_update() returns 0 for non-finite input. */
    return (pid_update(&p_follow->pid, 0.0f, -p_follow->error, dt_s));
}

/*!
 * @brief Slow down on sharp corrections for stability.
 *
 * @param[in] cruise_mm_s Cruise speed on a straight line.
 * @param[in] error       Line error, -1..1.
 *
 * @return Target forward speed in mm/s (0 if the inputs are not finite).
 */
float32_t
line_follow_speed (float32_t cruise_mm_s, float32_t error)
{
    float32_t drop =
        SPEED_DROP_AT_EDGE * car_math_clamp(car_math_abs(error), 0.0f, 1.0f);
    float32_t speed = cruise_mm_s * (1.0f - drop);

    if (!car_math_is_finite(speed))
    {
        speed = 0.0f;
    }

    return (speed);
}

/*** end of file ***/
