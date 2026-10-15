/** @file terrain.h
 *
 * @brief Buddy 4 - hump detection / height estimation and motion-event
 *        classification (pure C, host-testable).
 *
 * Hump height: the LSM303DLHC has no gyro, and double-integrating
 * acceleration drifts badly, so the height is the path integral
 *      h = sum(ds * sin(pitch))
 * using pitch from the (acceleration-compensated) accelerometer and ds
 * from the wheel encoders.  Integration only runs while on a hump, which
 * keeps drift bounded.
 */

#ifndef TERRAIN_H
#define TERRAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"

typedef enum
{
    HUMP_FLAT = 0,
    HUMP_CLIMBING,
    HUMP_DESCENDING
} terrain_hump_state_t;

typedef struct
{
    terrain_hump_state_t state;
    float32_t            height_mm;   /* height above the hump's start    */
    float32_t            peak_mm;     /* peak of the current / last hump  */
    float32_t            max_peak_mm; /* highest peak of the whole run    */
    float32_t            active_mm;   /* distance travelled on this hump  */
    float32_t            level_mm;    /* distance travelled level again   */
    float32_t            preroll_mm;  /* height gained before the trigger */
    uint32_t             humps;       /* humps completed                  */
} terrain_hump_t;

typedef enum
{
    EVT_STATIONARY = 0,
    EVT_CRUISING,
    EVT_ACCELERATING,
    EVT_DECELERATING,
    EVT_TURNING,
    EVT_CLIMBING,
    EVT_DESCENDING,
    EVT_IMPACT
} terrain_event_t;

void terrain_hump_init(terrain_hump_t * p_hump);
void terrain_hump_update(terrain_hump_t * p_hump, float32_t pitch_deg,
                         float32_t ds_mm);
terrain_event_t terrain_classify(float32_t speed_mm_s, float32_t accel_mm_s2,
                                 float32_t            turn_rate_dps,
                                 terrain_hump_state_t hump, bool b_impact);
char const *    terrain_event_name(terrain_event_t event);

#endif /* TERRAIN_H */

/*** end of file ***/
