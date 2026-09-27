/** @file motor.h
 *
 * @brief Buddy 2 - Robo Pico dual DC motor driver (PWM, drive-brake mode).
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "car_types.h"

void motor_init(void);
void motor_set(float32_t left_pct, float32_t right_pct);
void motor_brake(void);

#endif /* MOTOR_H */

/*** end of file ***/
