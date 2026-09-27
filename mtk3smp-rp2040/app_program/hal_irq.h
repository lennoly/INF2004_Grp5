/** @file hal_irq.h
 *
 * @brief Binding of RP2040 interrupts to micro T-Kernel interrupt
 *        handlers: GPIO edge callbacks and the 1 kHz sampler alarm.
 *
 * Plain C (stdint only) so it can be included from kernel-side and
 * SDK-side translation units alike.
 */

#ifndef HAL_IRQ_H
#define HAL_IRQ_H

#include <stdint.h>

/* RP2040 NVIC numbers (checked against the SDK in hal.c). */
#define HAL_IRQ_TIMER3   (3u)
#define HAL_IRQ_IO_BANK0 (13u)
#define HAL_IRQ_PRIORITY (2)

/* Callback run in interrupt context for a GPIO edge. */
typedef void (*hal_irq_gpio_cb_t)(uint32_t pin, uint32_t events, uint32_t t_us);

/* Callback run in interrupt context on every sampler tick. */
typedef void (*hal_irq_tick_cb_t)(uint32_t t_us);

int32_t hal_irq_gpio_attach(uint32_t pin, uint32_t edges,
                            hal_irq_gpio_cb_t callback);
int32_t hal_irq_sampler_attach(uint32_t period_us, hal_irq_tick_cb_t callback);

#endif /* HAL_IRQ_H */

/*** end of file ***/
