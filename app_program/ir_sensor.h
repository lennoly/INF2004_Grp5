/** @file ir_sensor.h
 *
 * @brief Buddy 3 - three analog IR reflectance sensors (left, right,
 *        barcode) sampled at 1 kHz by a hardware-alarm ISR.
 *
 * Normalised value: 0.0 = calibrated white, 1.0 = calibrated black.  The
 * barcode channel is also turned into a stream of bar / space widths (in
 * microseconds) with hysteresis, consumed by barcode.c.
 */

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"

#define IR_LEFT    (0u)
#define IR_RIGHT   (1u)
#define IR_BARCODE (2u)
#define IR_COUNT   (3u)

typedef struct
{
    uint16_t  raw[IR_COUNT];  /* filtered ADC counts (0..4095) */
    float32_t norm[IR_COUNT]; /* 0 white .. 1 black            */
} ir_sensor_snapshot_t;

typedef struct
{
    bool     b_bar; /* true = black bar, false = white space */
    uint32_t width_us;
} ir_sensor_element_t;

int32_t  ir_sensor_init(void);
void     ir_sensor_read(ir_sensor_snapshot_t * p_out);
void     ir_sensor_calib_begin(void);
bool     ir_sensor_calib_end(void);
bool     ir_sensor_barcode_pop(ir_sensor_element_t * p_out);
bool     ir_sensor_barcode_is_black(void);
uint32_t ir_sensor_barcode_idle_us(void);

#endif /* IR_SENSOR_H */

/*** end of file ***/
