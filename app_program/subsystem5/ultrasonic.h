/** @file ultrasonic.h
 *
 * @brief Buddy 5 - HC-SR04 ranging: 10 us trigger, echo width captured by
 *        GPIO interrupt time stamps (no busy-waiting on the echo).
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

#define ULTRASONIC_NO_ECHO (-1)

/* What one ping saw (FR7, assumption A9). */
typedef enum
{
    ULTRASONIC_VALID = 0,    /* an echo pulse within range            */
    ULTRASONIC_OUT_OF_RANGE, /* a pulse longer than US_MAX_MM (4 m),
                                or one still high at the timeout       */
    ULTRASONIC_FAULT         /* no echo pulse at all                   */
} ultrasonic_class_t;

typedef struct
{
    ultrasonic_class_t result;
    int32_t            dist_mm;  /* VALID only, else ULTRASONIC_NO_ECHO */
    uint32_t           width_us; /* raw echo pulse width, 0 if none     */
} ultrasonic_ping_t;

int32_t      ultrasonic_init(void);
void         ultrasonic_ping(ultrasonic_ping_t * p_out);
char const * ultrasonic_class_name(ultrasonic_class_t result);
int32_t      ultrasonic_measure_mm(void);
int32_t      ultrasonic_median_mm(uint32_t samples);

#endif /* ULTRASONIC_H */

/*** end of file ***/
