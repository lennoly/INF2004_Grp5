/** @file obstacle.h
 *
 * @brief Buddy 5 - adaptive scanning subsystem: a background front
 *        monitor plus an on-demand coarse-to-fine scan.
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdint.h>
#include <stdbool.h>
#include "avoidance.h"

/* Called by obstacle_scan() and obstacle_look_mm() while the servo settles
   and after every ping, in the caller's task; returns true to end the scan
   or look early. */
typedef bool (*obstacle_stop_cb_t)(void);

int32_t  obstacle_init(void);
int32_t  obstacle_front_mm(void);
bool     obstacle_ahead(void);
bool     obstacle_front_stale(void);
void     obstacle_monitor_enable(bool b_on);
bool     obstacle_scan(avoidance_profile_t * p_prof,
                       obstacle_stop_cb_t    p_should_stop);
int32_t  obstacle_look_mm(int32_t servo_deg, obstacle_stop_cb_t p_should_stop);
void     obstacle_get_last(avoidance_profile_t * p_prof, uint32_t * p_scans);
uint32_t obstacle_get_points(avoidance_point_t * p_points, uint32_t max_points);

#endif /* OBSTACLE_H */

/*** end of file ***/
