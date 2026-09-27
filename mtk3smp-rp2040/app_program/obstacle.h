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

int32_t obstacle_init(void);
int32_t obstacle_front_mm(void);
void    obstacle_monitor_enable(bool b_on);
bool    obstacle_scan(avoidance_profile_t * p_prof);
int32_t obstacle_look_mm(int32_t servo_deg);
void    obstacle_get_last(avoidance_profile_t * p_prof, uint32_t * p_scans);

#endif /* OBSTACLE_H */

/*** end of file ***/
