/** @file car_math.h
 *
 * @brief Small single-precision maths helpers.
 *
 * The RP2040 Cortex-M0+ has no FPU and the template does not link libm, so
 * these self-contained approximations are used instead.  Adding -lm is not
 * enough: newlib's sqrtf() and atan2f() set errno, which pulls newlib's
 * stdio and system calls into an image that has none.  Accuracy verified
 * by tests/host: square root better than 1e-5 relative, arctangent better
 * than 0.01 degree, sine better than 1e-4.
 */

#ifndef CAR_MATH_H
#define CAR_MATH_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"

#define CAR_MATH_PI          (3.14159265f)
#define CAR_MATH_DEG_PER_RAD (57.2957795f)
#define CAR_MATH_RAD_PER_DEG (0.0174532925f)

float32_t car_math_abs(float32_t value);
float32_t car_math_clamp(float32_t value, float32_t low, float32_t high);
float32_t car_math_sqrt(float32_t value);
float32_t car_math_atan2(float32_t y_value, float32_t x_value);
float32_t car_math_sin(float32_t angle_rad);
float32_t car_math_cos(float32_t angle_rad);
int32_t   car_math_round(float32_t value);
bool      car_math_is_finite(float32_t value);

#endif /* CAR_MATH_H */

/*** end of file ***/
