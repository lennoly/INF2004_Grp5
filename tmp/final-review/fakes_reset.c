/* Deterministic host adapters: actual sensor, framing and decoder code runs. */
#include <stdarg.h>
#include <string.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "hal.h"
#include "hal_irq.h"
#include "timing.h"
#include "barcode.h"
#include "car_config.h"
#include "fakes.h"

static hal_irq_tick_cb_t g_sampler;
static uint16_t g_adc[3];
static uint32_t g_time;
static uint8_t g_messages[4][32];
static uint32_t g_head;
static uint32_t g_tail;
static uint32_t g_count;
static uint32_t g_capacity;
static uint32_t g_poll_ticks;
int32_t g_fake_init_error;
uint32_t g_fake_tasks;
uint32_t g_fake_message_size;
bool gb_fake_stop;

void fake_set_time(uint32_t time_us) { g_time = time_us; }
void fake_sample(uint16_t left, uint16_t right, uint16_t barcode,
                 uint32_t ticks, bool poll)
{
    g_adc[0] = left;
    g_adc[1] = right;
    g_adc[2] = barcode;
    for (uint32_t i = 0u; i < ticks; i++)
    {
        g_time += 1000u;
        g_sampler(g_time);
        g_poll_ticks++;
        if (poll && (g_poll_ticks % 10u == 0u)) { barcode_poll(); }
    }
}
uint32_t hal_time_us(void) { return g_time; }
void hal_adc_init(void) {}
void hal_adc_pin_init(uint32_t pin) { (void) pin; }
uint16_t hal_adc_read(uint32_t channel) { return g_adc[channel]; }
extern bool inject_reset; void hal_memory_barrier(void) { if (inject_reset) { inject_reset=false; barcode_set_enabled(false); } }
void hal_gpio_init_in(uint32_t pin, bool pull) { (void) pin; (void) pull; }
void hal_gpio_init_out(uint32_t pin, bool level) { (void) pin; (void) level; }
bool hal_gpio_get(uint32_t pin) { return (pin != PIN_BTN_STOP) || !gb_fake_stop; }
int32_t hal_irq_sampler_attach(uint32_t period, hal_irq_tick_cb_t callback)
{
    (void) period;
    if (g_fake_init_error == 0) { g_sampler = callback; }
    return g_fake_init_error;
}
ID tk_cre_mbf(T_CMBF const *p_cfg)
{
    g_fake_message_size = (uint32_t) p_cfg->maxmsz;
    g_capacity = (uint32_t) p_cfg->bufsz
                 / ((g_fake_message_size + 3u) / 4u * 4u + 4u);
    return (g_fake_init_error != 0) ? g_fake_init_error : 1;
}
ID tk_cre_tsk(T_CTSK const *p_cfg)
{
    (void) p_cfg;
    if (g_fake_init_error != 0) { return g_fake_init_error; }
    g_fake_tasks++;
    return (ID) g_fake_tasks;
}
ER tk_sta_tsk(ID task, INT code) { (void) task; (void) code; return E_OK; }
ER tk_del_tsk(ID task) { (void) task; return E_OK; }
ER tk_del_mbf(ID buffer) { (void) buffer; return E_OK; }
ER tk_snd_mbf(ID buffer, void const *p_src, INT size, INT timeout)
{
    (void) buffer; (void) timeout;
    if ((g_count >= g_capacity) || (g_count == 4u)
        || (size != (INT) g_fake_message_size)) { return -1; }
    memcpy(g_messages[g_head], p_src, (size_t) size);
    g_head = (g_head + 1u) % 4u;
    g_count++;
    return E_OK;
}
INT tk_rcv_mbf(ID buffer, void *p_dst, INT timeout)
{
    (void) buffer; (void) timeout;
    if (g_count == 0u) { return -1; }
    memcpy(p_dst, g_messages[g_tail], g_fake_message_size);
    g_tail = (g_tail + 1u) % 4u;
    g_count--;
    return (INT) g_fake_message_size;
}
ER tk_get_tim(SYSTIM *p_time) { p_time->lo = g_time / 1000u; return E_OK; }
ER tk_dly_tsk(uint32_t ms) { g_time += ms * 1000u; return E_OK; }
INT tm_printf(UB const *p_fmt, ...) { (void) p_fmt; return 0; }
uint32_t timing_start(void) { return g_time; }
void timing_stop(timing_id_t slot, uint32_t start) { (void) slot; (void) start; }
