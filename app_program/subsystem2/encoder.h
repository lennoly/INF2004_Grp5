/** @file encoder.h
 *
 * @brief Buddy 2 - single-channel wheel encoders (interrupt driven).
 *
 * Both edges are counted (2 ticks per slot) for distance resolution; speed
 * uses rising-to-rising periods, which are immune to slot / bar duty
 * errors.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "car_types.h"

#define ENCODER_LEFT  (0u)
#define ENCODER_RIGHT (1u)
#define ENCODER_COUNT (2u)

typedef struct
{
    uint32_t  ticks;      /* edges since boot (unsigned, no direction) */
    float32_t speed_mm_s; /* magnitude                                 */
} encoder_reading_t;

int32_t   encoder_init(void);
void      encoder_read(uint32_t wheel, encoder_reading_t * p_out);
float32_t encoder_mm_per_tick(void);

#endif /* ENCODER_H */

/*** end of file ***/
