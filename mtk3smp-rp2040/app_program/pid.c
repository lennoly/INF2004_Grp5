/** @file pid.c
 *
 * @brief PID controller implementation (see pid.h).
 */

#include <stdbool.h>
#include "car_math.h"
#include "pid.h"

/*!
 * @brief Set the gains and output limits, and clear the controller state.
 *
 * @param[out] p_pid   Controller to initialise.
 * @param[in]  gain_p  Proportional gain.
 * @param[in]  gain_i  Integral gain (per second).
 * @param[in]  gain_d  Derivative gain (seconds).
 * @param[in]  gain_ff Feed-forward gain applied to the set-point.
 * @param[in]  out_min Lowest output value.
 * @param[in]  out_max Highest output value.
 */
void
pid_init (pid_ctrl_t * p_pid, float32_t gain_p, float32_t gain_i,
          float32_t gain_d, float32_t gain_ff, float32_t out_min,
          float32_t out_max)
{
    p_pid->gain_p  = gain_p;
    p_pid->gain_i  = gain_i;
    p_pid->gain_d  = gain_d;
    p_pid->gain_ff = gain_ff;
    p_pid->out_min = out_min;
    p_pid->out_max = out_max;
    pid_reset(p_pid);
}

/*!
 * @brief Forget the integral and derivative history (gains are kept).
 *
 * @param[in,out] p_pid Controller to reset.
 */
void
pid_reset (pid_ctrl_t * p_pid)
{
    p_pid->integ     = 0.0f;
    p_pid->prev_meas = 0.0f;
    p_pid->b_primed  = false;
}

/*!
 * @brief One controller step.
 *
 * Non-finite inputs or results (Rule 5.4.b.v) reset the controller and
 * give an output of 0, so a bad sample can never drive an actuator.
 *
 * @param[in,out] p_pid    Controller.
 * @param[in]     setpoint Desired value.
 * @param[in]     measured Measured value.
 * @param[in]     dt_s     Time since the previous step in seconds (> 0).
 *
 * @return Controller output clamped to [out_min, out_max].
 */
float32_t
pid_update (pid_ctrl_t * p_pid, float32_t setpoint, float32_t measured,
            float32_t dt_s)
{
    float32_t error     = setpoint - measured;
    float32_t deriv     = 0.0f;
    float32_t out_unsat = 0.0f;
    float32_t result    = 0.0f;
    bool      b_valid   = (dt_s > 0.0f) && (car_math_is_finite(error));

    if (b_valid && (p_pid->b_primed))
    {
        deriv = (measured - p_pid->prev_meas) / dt_s;
    }

    if (b_valid)
    {
        p_pid->prev_meas = measured;
        p_pid->b_primed  = true;
        out_unsat        = (p_pid->gain_ff * setpoint) + (p_pid->gain_p * error)
                    + p_pid->integ - (p_pid->gain_d * deriv);
        b_valid = car_math_is_finite(out_unsat);
    }

    if (b_valid)
    {
        result = car_math_clamp(out_unsat, p_pid->out_min, p_pid->out_max);

        /* Only integrate when it would not push further into saturation. */
        if (!(((out_unsat > p_pid->out_max) && (error > 0.0f))
              || ((out_unsat < p_pid->out_min) && (error < 0.0f))))
        {
            p_pid->integ += p_pid->gain_i * error * dt_s;
            p_pid->integ =
                car_math_clamp(p_pid->integ, p_pid->out_min, p_pid->out_max);
        }
    }
    else
    {
        pid_reset(p_pid);
    }

    return (result);
}

/*** end of file ***/
