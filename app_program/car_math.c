/** @file car_math.c
 *
 * @brief Single-precision maths helpers (see car_math.h).  Pure C with no
 *        kernel headers, so <math.h> can be used here for isfinite().
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "car_math.h"

#define ATAN_POLY_C1    (0.9998660f) /* minimax atan() on [0,1]       */
#define ATAN_POLY_C3    (-0.3302995f)
#define ATAN_POLY_C5    (0.1801410f)
#define ATAN_POLY_C7    (-0.0851330f)
#define ATAN_POLY_C9    (0.0208351f)
#define SQRT_ITERATIONS (3)
#define SQRT_MAGIC      (0x1FC00000u) /* halves the float exponent     */
#define TINY            (1e-12f)
#define TWO_PI          (2.0f * CAR_MATH_PI)
#define HALF_PI         (0.5f * CAR_MATH_PI)
#define ROUND_HALF      (0.5f)
#define SIN_MAX_ARG     (1.0e4f) /* larger angles give 0          */

/* Taylor-series divisors for sin(): 2*3, 4*5, 6*7, 8*9. */
#define SIN_TERM_3 (6.0f)
#define SIN_TERM_5 (20.0f)
#define SIN_TERM_7 (42.0f)
#define SIN_TERM_9 (72.0f)

/*!
 * @brief Absolute value.
 *
 * @param[in] value Any finite value.
 *
 * @return |value|.
 */
float32_t
car_math_abs (float32_t value)
{
    return ((value < 0.0f) ? -value : value);
}

/*!
 * @brief Limit a value to the closed range [low, high].
 *
 * @param[in] value The value to limit.
 * @param[in] low   Lower bound.
 * @param[in] high  Upper bound (must be >= low).
 *
 * @return The limited value.
 */
float32_t
car_math_clamp (float32_t value, float32_t low, float32_t high)
{
    float32_t result = value;

    if (value < low)
    {
        result = low;
    }
    else if (value > high)
    {
        result = high;
    }
    else
    {
        /* Already inside the range. */
    }

    return (result);
}

/*!
 * @brief Square root by a bit-level first guess and Newton-Raphson steps.
 *
 * @param[in] value Input; values <= 0 (and non-finite input) give 0.
 *
 * @return sqrt(value).
 */
float32_t
car_math_sqrt (float32_t value)
{
    union
    {
        float32_t as_float;
        uint32_t  as_bits;
    } guess;
    float32_t result = 0.0f;
    int32_t   iter   = 0;

    if ((value > 0.0f) && (car_math_is_finite(value)))
    {
        /* NOTE: reading the float through the union's integer member is
           the C99-defined way to access its IEEE-754 bit pattern. */
        guess.as_float = value;
        guess.as_bits  = (guess.as_bits >> 1) + SQRT_MAGIC;
        result         = guess.as_float;

        for (iter = 0; iter < SQRT_ITERATIONS; iter++)
        {
            result = 0.5f * (result + (value / result));
        }
    }

    return (result);
}

/*!
 * @brief Four-quadrant arctangent.
 *
 * @param[in] y_value Opposite side.
 * @param[in] x_value Adjacent side.
 *
 * @return Angle in radians in (-pi, pi]; 0 if both inputs are ~0.
 */
float32_t
car_math_atan2 (float32_t y_value, float32_t x_value)
{
    float32_t abs_x  = car_math_abs(x_value);
    float32_t abs_y  = car_math_abs(y_value);
    float32_t result = 0.0f;
    float32_t ratio  = 0.0f;
    float32_t square = 0.0f;
    float32_t larger = 0.0f;
    float32_t poly   = 0.0f;

    if ((abs_x >= TINY) || (abs_y >= TINY))
    {
        /* Evaluate atan() on [0,1] and use symmetry for the rest. */
        larger = (abs_x > abs_y) ? abs_x : abs_y;
        ratio  = ((abs_x < abs_y) ? abs_x : abs_y) / larger;
        square = ratio * ratio;
        /* Odd polynomial in ratio, evaluated by Horner's rule. */
        poly   = ATAN_POLY_C9;
        poly   = ATAN_POLY_C7 + (square * poly);
        poly   = ATAN_POLY_C5 + (square * poly);
        poly   = ATAN_POLY_C3 + (square * poly);
        poly   = ATAN_POLY_C1 + (square * poly);
        result = ratio * poly;

        if (abs_y > abs_x)
        {
            result = HALF_PI - result;
        }

        if (x_value < 0.0f)
        {
            result = CAR_MATH_PI - result;
        }

        if (y_value < 0.0f)
        {
            result = -result;
        }
    }

    return (result);
}

/*!
 * @brief Sine by range reduction and a Taylor series on [-pi/2, pi/2].
 *
 * @param[in] angle_rad Angle in radians, |angle_rad| <= 1e4.  Larger or
 *                      non-finite input gives 0: repeated subtraction of
 *                      2 pi never ends there, because it no longer changes
 *                      a float32_t.
 *
 * @return sin(angle_rad).
 */
float32_t
car_math_sin (float32_t angle_rad)
{
    float32_t angle  = angle_rad;
    float32_t square = 0.0f;
    float32_t series = 0.0f;
    float32_t result = 0.0f;

    if ((car_math_is_finite(angle)) && (car_math_abs(angle) <= SIN_MAX_ARG))
    {
        /* Remove whole turns in one step, then fix up the rounding.
           Cast: |turns| <= 1600, exact in float32_t. */
        angle -= ((float32_t) car_math_round(angle / TWO_PI)) * TWO_PI;

        while (angle > CAR_MATH_PI)
        {
            angle -= TWO_PI;
        }

        while (angle < -CAR_MATH_PI)
        {
            angle += TWO_PI;
        }

        /* Fold into [-pi/2, pi/2] where the series converges quickly. */
        if (angle > HALF_PI)
        {
            angle = CAR_MATH_PI - angle;
        }
        else if (angle < -HALF_PI)
        {
            angle = -CAR_MATH_PI - angle;
        }
        else
        {
            /* Already inside the fast range. */
        }

        /* Taylor series x(1 - x^2/6 (1 - x^2/20 (1 - x^2/42 (1 - x^2/72)))),
           evaluated from the innermost term outwards. */
        square = angle * angle;
        series = 1.0f - (square / SIN_TERM_9);
        series = 1.0f - ((square / SIN_TERM_7) * series);
        series = 1.0f - ((square / SIN_TERM_5) * series);
        series = 1.0f - ((square / SIN_TERM_3) * series);
        result = angle * series;
    }

    return (result);
}

/*!
 * @brief Cosine through the identity cos(a) = sin(a + pi/2).
 *
 * @param[in] angle_rad Angle in radians.
 *
 * @return cos(angle_rad).
 */
float32_t
car_math_cos (float32_t angle_rad)
{
    return (car_math_sin(angle_rad + HALF_PI));
}

/*!
 * @brief Round half away from zero to a 32-bit integer.
 *
 * @param[in] value Value to round; must lie within the int32_t range.
 *
 * @return The rounded value (0 for non-finite input).
 */
int32_t
car_math_round (float32_t value)
{
    int32_t result = 0;

    if (car_math_is_finite(value))
    {
        /* Cast: the caller guarantees |value| < 2^31, so the truncating
           float-to-int conversion after adding +-0.5 cannot overflow. */
        result = (value >= 0.0f) ? (int32_t) (value + ROUND_HALF)
                                 : (int32_t) (value - ROUND_HALF);
    }

    return (result);
}

/*!
 * @brief BARR-C Rule 5.4.b.v check that a result is neither NaN nor
 *        infinite.
 *
 * @param[in] value Result of a floating-point calculation.
 *
 * @return true if value is finite.
 */
bool
car_math_is_finite (float32_t value)
{
    return (0 != isfinite(value));
}

/*** end of file ***/
