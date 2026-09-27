/** @file motor.c
 *
 * @brief Buddy 2 - Robo Pico motor driver.
 *
 * Robo Pico truth table: A=H,B=L forward; A=L,B=H backward; A=L,B=L brake.
 * Forward = PWM on A with B held low, so the "off" part of each PWM period
 * brakes the motor (drive-brake), giving a near-linear speed/duty curve.
 */

#include <stdint.h>
#include "car_config.h"
#include "car_math.h"
#include "hal.h"
#include "motor.h"

#define PWM_CLKDIV (1u)
#define PWM_WRAP   ((HAL_CLK_SYS_HZ / MOTOR_PWM_FREQ_HZ) - 1u)
#define PWM_COUNTS (PWM_WRAP + 1u)
#define PCT_FULL   (100.0f)

static void channel_set(uint32_t pin_a, uint32_t pin_b, float32_t duty_pct);

/*!
 * @brief Configure the four motor pins as 10 kHz PWM outputs, braked.
 */
void
motor_init (void)
{
    hal_pwm_setup(PIN_MOTOR_L_A, PWM_CLKDIV, PWM_WRAP);
    hal_pwm_setup(PIN_MOTOR_L_B, PWM_CLKDIV, PWM_WRAP);
    hal_pwm_setup(PIN_MOTOR_R_A, PWM_CLKDIV, PWM_WRAP);
    hal_pwm_setup(PIN_MOTOR_R_B, PWM_CLKDIV, PWM_WRAP);
    motor_brake();
}

/*!
 * @brief Set both motors.  Positive means forward after the INVERT options.
 *
 * @param[in] left_pct  Left duty, -100..100 percent.
 * @param[in] right_pct Right duty, -100..100 percent.
 */
void
motor_set (float32_t left_pct, float32_t right_pct)
{
    float32_t left  = car_math_clamp(left_pct, -PCT_FULL, PCT_FULL);
    float32_t right = car_math_clamp(right_pct, -PCT_FULL, PCT_FULL);

#if MOTOR_L_INVERT
    channel_set(PIN_MOTOR_L_A, PIN_MOTOR_L_B, -left);
#else
    channel_set(PIN_MOTOR_L_A, PIN_MOTOR_L_B, left);
#endif
#if MOTOR_R_INVERT
    channel_set(PIN_MOTOR_R_A, PIN_MOTOR_R_B, -right);
#else
    channel_set(PIN_MOTOR_R_A, PIN_MOTOR_R_B, right);
#endif
}

/*!
 * @brief Both inputs low: electrical brake (fast stop).
 */
void
motor_brake (void)
{
    motor_set(0.0f, 0.0f);
}

/*!
 * @brief Drive one H-bridge channel with a signed duty in percent.
 *
 * A non-finite duty (Rule 5.4.b.v) brakes the channel instead.
 *
 * @param[in] pin_a    Channel input A (PWM for forward).
 * @param[in] pin_b    Channel input B (PWM for backward).
 * @param[in] duty_pct Signed duty, -100..100 percent.
 */
static void
channel_set (uint32_t pin_a, uint32_t pin_b, float32_t duty_pct)
{
    float32_t duty  = 0.0f;
    uint32_t  level = 0u;

    if (car_math_is_finite(duty_pct))
    {
        duty = car_math_clamp(duty_pct, -PCT_FULL, PCT_FULL);
    }

    /* Casts: PWM_COUNTS (12 500) converts exactly to float32_t, and the
       product is 0..PWM_COUNTS because |duty| <= 100, so it converts back
       to uint32_t without overflow. */
    level =
        (uint32_t) ((car_math_abs(duty) * (float32_t) PWM_COUNTS) / PCT_FULL);

    if (duty >= 0.0f)
    {
        hal_pwm_set_level(pin_b, 0u);
        hal_pwm_set_level(pin_a, level);
    }
    else
    {
        hal_pwm_set_level(pin_a, 0u);
        hal_pwm_set_level(pin_b, level);
    }
}

/*** end of file ***/
