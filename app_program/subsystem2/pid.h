/** @file pid.h
 *
 * @brief Reusable PID controller (pure C, host-testable).
 *
 * output = gain_ff * setpoint + gain_p * error + integral
 *          - gain_d * d(measured)/dt
 *
 * The derivative acts on the measurement (no kick on set-point steps) and
 * integration is conditional and clamped (anti-windup).
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include "car_types.h"

typedef struct
{
    float32_t gain_p;  /* proportional gain                        */
    float32_t gain_i;  /* integral gain, per second                */
    float32_t gain_d;  /* derivative gain, seconds                 */
    float32_t gain_ff; /* feed-forward gain                        */
    float32_t out_min;
    float32_t out_max;
    float32_t integ; /* integral term, already scaled by gain_i  */
    float32_t prev_meas;
    bool      b_primed; /* prev_meas holds a real sample            */
} pid_ctrl_t;

void      pid_init(pid_ctrl_t * p_pid, float32_t gain_p, float32_t gain_i,
                   float32_t gain_d, float32_t gain_ff, float32_t out_min,
                   float32_t out_max);
void      pid_reset(pid_ctrl_t * p_pid);
float32_t pid_update(pid_ctrl_t * p_pid, float32_t setpoint, float32_t measured,
                     float32_t dt_s);

#endif /* PID_H */

/*** end of file ***/
