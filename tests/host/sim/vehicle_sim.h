/* Host simulation of the vehicle task (vehicle.c) and the obstacle scanner
   (obstacle.c, servo.c) on a fake kernel, with simulated time, buttons,
   motion, IR, IMU, barcode and sonar.  Used by test_main.c; like lwip/,
   this folder holds test scaffolding and is outside the BARR-C checks. */
#ifndef VEHICLE_SIM_H
#define VEHICLE_SIM_H

#include <stdbool.h>
#include <stdint.h>
#include "barcode_decode.h"
#include "terrain.h"

void     vehicle_sim_init(void);
void     vehicle_sim_step(void);
void     vehicle_sim_run(uint32_t ms);
void     vehicle_sim_press(uint32_t pin, uint32_t after_ms, uint32_t hold_ms);
void     vehicle_sim_stop_command_at(uint32_t after_ms);
void     vehicle_sim_stop_on_next_turn(void);
void     vehicle_sim_front(int32_t mm);
void     vehicle_sim_box(bool b_present);
void     vehicle_sim_line(bool b_on_line);
void     vehicle_sim_hump(terrain_hump_state_t state);
void     vehicle_sim_impact(void);
void     vehicle_sim_barcode(barcode_decode_nav_t cmd);
void     vehicle_sim_clear_log(void);
uint32_t vehicle_sim_count(char const * p_text);
bool     vehicle_sim_log_starts(char const * p_text);
uint32_t vehicle_sim_pings(void);
uint32_t vehicle_sim_servo_us(void);
uint32_t vehicle_sim_queued_barcodes(void);
uint32_t vehicle_sim_watchdog_gap_ms(void);
uint32_t vehicle_sim_watchdog_timeout_ms(void);

#endif /* VEHICLE_SIM_H */
