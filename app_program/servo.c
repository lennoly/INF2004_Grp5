/** @file servo.c
 *
 * @brief Buddy 5 - servo on GP12: 50 Hz PWM with a 1 us tick
 *        (125 MHz / 125 = 1 MHz, wrap 19999 = 20 ms).
 */

#include <stdint.h>
#include "car_config.h"
#include "hal.h"
#include "servo.h"

#define SERVO_CLKDIV    (125u)
#define SERVO_WRAP      (19999u)
#define SERVO_RANGE_DEG (180)

/* Set by the vehicle task, read by the sonar monitor task: volatile. */
static volatile int32_t g_angle = SERVO_CENTRE_DEG;

/*!
 * @brief Start the 50 Hz servo PWM and centre the sensor.
 */
void
servo_init (void)
{
    hal_pwm_setup(PIN_SERVO, SERVO_CLKDIV, SERVO_WRAP);
    servo_set_angle(SERVO_CENTRE_DEG);
}

/*!
 * @brief Move the servo; the angle is limited to 0..180 degrees.
 *
 * @param[in] angle_deg Requested angle in degrees, 0 (right) .. 180 (left).
 *                      SERVO_INVERT mirrors the pulse for a sensor mounted
 *                      the other way round, so callers always use these
 *                      directions.
 */
void
servo_set_angle (int32_t angle_deg)
{
    int32_t  limited  = angle_deg;
    int32_t  physical = 0;
    uint32_t pulse_us = 0u;

    if (limited < 0)
    {
        limited = 0;
    }
    else if (limited > SERVO_RANGE_DEG)
    {
        limited = SERVO_RANGE_DEG;
    }
    else
    {
        /* Already inside the servo's range. */
    }

#if SERVO_INVERT
    physical = SERVO_RANGE_DEG - limited;
#else
    physical = limited;
#endif

    /* Casts: physical is 0..180 (limited is checked above) and the range
       constant is positive, so both convert to uint32_t without change. */
    pulse_us = SERVO_MIN_US
               + (((uint32_t) physical * (SERVO_MAX_US - SERVO_MIN_US))
                  / (uint32_t) SERVO_RANGE_DEG);
    hal_pwm_set_level(PIN_SERVO, pulse_us); /* 1 count = 1 us */
    g_angle = limited;
}

/*!
 * @brief Last angle commanded with servo_set_angle().
 *
 * @return Angle in degrees, 0..180.
 */
int32_t
servo_get_angle (void)
{
    return (g_angle);
}

/*** end of file ***/
