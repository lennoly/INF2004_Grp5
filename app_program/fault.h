/** @file fault.h
 *
 * @brief Shared fault bitmask (Change Requirement 1, Tribunal 1 decision).
 *
 * One bit per fault condition, held in a uT-Kernel event flag: set and
 * clear are atomic system calls (tk_set_flg is even allowed from an
 * interrupt handler), so no task ever does a read-modify-write on a shared
 * word (the Cortex-M0+ has no LDREX/STREX).  The vehicle task reads every bit
 * with one call and decides what each fault does.
 *
 * Bit map (owned by Buddy 5).  Each bit has exactly one writer:
 *   FAULT_MOTOR_STALL  motion task  (a move ended by its stall timeout)
 *   FAULT_ENCODER      motion task  (drive applied, no encoder edges)
 *   FAULT_IMU          imu task     (three failed reads in a row)
 *   FAULT_NO_ECHO      sonar task   (two FAULT pings in a row)
 *   FAULT_WIFI_LOSS    cyw43 task   (Wi-Fi link down after it was up)
 *
 * Because the vehicle samples the bits every 20 ms, a fault that sets and
 * clears between two samples would be missed, and a stalled owner cannot
 * set its own bit.  So each owner also keeps a saturating 8-bit count of
 * how often its fault was raised and the 16-bit time of its last update;
 * both are written only by that owner, as single-byte / half-word stores,
 * which are atomic on this CPU.
 */

#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    FAULT_MOTOR_STALL = 0,
    FAULT_ENCODER,
    FAULT_IMU,
    FAULT_NO_ECHO,
    FAULT_WIFI_LOSS,
    FAULT_COUNT
} fault_id_t;

#define FAULT_BITS (5u) /* width of the mask: one bit per fault_id_t */

int32_t  fault_init(void);
void     fault_raise(fault_id_t fault);
void     fault_clear(fault_id_t fault);
void     fault_alive(fault_id_t fault);
uint32_t fault_mask(void);
uint8_t  fault_count(fault_id_t fault);
uint32_t fault_age_ms(fault_id_t fault);

#endif /* FAULT_H */

/*** end of file ***/
