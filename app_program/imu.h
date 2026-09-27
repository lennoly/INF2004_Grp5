/** @file imu.h
 *
 * @brief Buddy 4 - LSM303DLHC (GY-511) accelerometer and magnetometer
 *        processing task: tilt, humps, impacts, turn rate and events.
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"
#include "terrain.h"

typedef struct
{
    bool            b_ok; /* sensor answered on I2C         */
    uint32_t        i2c_errors;
    float32_t       ax_mg; /* filtered, bias-corrected       */
    float32_t       ay_mg;
    float32_t       az_mg;
    float32_t       pitch_deg;    /* nose up positive               */
    float32_t       roll_deg;     /* right side down positive       */
    float32_t       heading_deg;  /* magnetic, 0..360               */
    float32_t       mag_rate_dps; /* heading rate from magnetometer */
    float32_t       enc_rate_dps; /* turn rate from wheel encoders  */
    float32_t       accel_mm_s2;  /* longitudinal, from encoders    */
    terrain_hump_t  hump;
    terrain_event_t event;
    uint32_t        impacts;
} imu_status_t;

int32_t imu_init(void);
void    imu_get_status(imu_status_t * p_out);
void    imu_mag_calib_begin(void);
void    imu_mag_calib_end(void);
void    imu_reset_run(void);

#endif /* IMU_H */

/*** end of file ***/
