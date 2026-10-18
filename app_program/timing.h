/** @file timing.h
 *
 * @brief Execution-time, control-jitter and CPU-load measurement (CSC05
 *        review: WCET of motion, vehicle, barcode and the hump estimator,
 *        the cost of the 1 kHz IR sampler, total CPU load, A2 jitter).
 *
 * Each measured section brackets itself with timing_start() and
 * timing_stop().  Total load comes from an idle probe task at the lowest
 * priority, so it includes the kernel and the template's USB and Wi-Fi
 * tasks.  Read and restart the window with the "timing" command.
 * Standard types only, so the lwIP side (mqtt_lwip.c) can use it too.
 */

#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>

typedef enum
{
    TIMING_MOTION = 0, /* one 20 ms control step                  */
    TIMING_VEHICLE,    /* one vehicle loop in a non-waiting state */
    TIMING_BARCODE,    /* one barcode task cycle                  */
    TIMING_IMU,        /* one IMU step (includes the I2C reads)    */
    TIMING_HUMP,       /* the hump estimator alone                */
    TIMING_IR_ISR,     /* the 1 kHz IR sampler interrupt          */
    TIMING_NET,        /* the MQTT poll hook in the cyw43 task    */
    TIMING_REJOIN,     /* one Wi-Fi re-join request               */
    TIMING_COUNT
} timing_id_t;

typedef struct
{
    uint32_t max_us;
    uint32_t sum_us;
    uint32_t count;
} timing_stat_t;

typedef struct
{
    timing_stat_t stat[TIMING_COUNT];
    uint32_t      window_ms;     /* length of this measurement window */
    uint32_t      idle_ms;       /* time the idle probe ran           */
    uint32_t      jitter_max_us; /* worst |motion period - 20 ms|     */
    uint32_t      periods;       /* motion periods measured           */
} timing_report_t;

int32_t      timing_init(void);
uint32_t     timing_start(void);
void         timing_stop(timing_id_t section, uint32_t start_us);
void         timing_period(uint32_t now_us);
void         timing_take(timing_report_t * p_out);
char const * timing_name(timing_id_t section);

#endif /* TIMING_H */

/*** end of file ***/
