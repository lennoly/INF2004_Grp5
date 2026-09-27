/** @file ultrasonic.h
 *
 * @brief Buddy 5 - HC-SR04 ranging: 10 us trigger, echo width captured by
 *        GPIO interrupt time stamps (no busy-waiting on the echo).
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

#define ULTRASONIC_NO_ECHO (-1)

int32_t ultrasonic_init(void);
int32_t ultrasonic_measure_mm(void);
int32_t ultrasonic_median_mm(uint32_t samples);

#endif /* ULTRASONIC_H */

/*** end of file ***/
