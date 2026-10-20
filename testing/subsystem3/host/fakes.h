#ifndef S3_FAKES_H
#define S3_FAKES_H
#include <stdint.h>
#include <stdbool.h>
void fake_sample(uint16_t left, uint16_t right, uint16_t barcode,
                 uint32_t ticks, bool poll);
void fake_set_time(uint32_t time_us);
extern int32_t g_fake_init_error;
extern uint32_t g_fake_tasks;
extern uint32_t g_fake_message_size;
extern bool gb_fake_stop;
#endif
