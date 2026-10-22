/** @file servo.h
 *
 * @brief Buddy 5 - RC servo that pans the ultrasonic sensor.
 *        0 degrees = right, 90 = straight ahead, 180 = left
 *        (SERVO_INVERT in car_config.h corrects a mirrored mount).
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

void    servo_init(void);
void    servo_set_angle(int32_t angle_deg);
int32_t servo_get_angle(void);

#endif /* SERVO_H */

/*** end of file ***/
