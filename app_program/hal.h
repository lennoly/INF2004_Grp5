/** @file hal.h
 *
 * @brief Plain-C hardware abstraction layer for the PicoCar.
 *
 * hal.c implements this API with the Raspberry Pi Pico C SDK
 * (hardware_pwm, hardware_adc, hardware_gpio, hardware_structs ...).
 * hal_irq.c and hal_irq.h bind the RP2040 interrupts to micro T-Kernel
 * handlers.
 *
 * Why the split: the template forbids mixing <tk/tkernel.h> and Pico SDK
 * headers in one translation unit (their size_t typedefs collide), so this
 * header only uses <stdint.h> and <stdbool.h> and can be included from
 * both kinds of file.
 */

#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdbool.h>

/* GPIO edge event bits (same layout as the RP2040 INTR registers). */
#define HAL_GPIO_EDGE_FALL (0x4u)
#define HAL_GPIO_EDGE_RISE (0x8u)

#define HAL_CLK_SYS_HZ     (125000000u) /* set by the template BSP */
#define HAL_I2C_TIMEOUT_US (2000u)

/* Time. */
uint32_t hal_time_us(void);
void     hal_delay_us(uint32_t delay_us);

/* GPIO. */
void     hal_gpio_init_out(uint32_t pin, bool b_level);
void     hal_gpio_init_in(uint32_t pin, bool b_pull_up);
void     hal_gpio_put(uint32_t pin, bool b_level);
bool     hal_gpio_get(uint32_t pin);
void     hal_gpio_irq_enable(uint32_t pin, uint32_t edges);
uint32_t hal_gpio_irq_take(uint32_t pin);

/* PWM. */
void hal_pwm_setup(uint32_t pin, uint32_t clkdiv, uint32_t wrap);
void hal_pwm_set_level(uint32_t pin, uint32_t level);

/* ADC. */
void     hal_adc_init(void);
void     hal_adc_pin_init(uint32_t pin);
uint16_t hal_adc_read(uint32_t channel);

/* I2C (master, polled, with timeouts). */
bool    hal_i2c_init(uint32_t sda, uint32_t scl, uint32_t baud_hz);
int32_t hal_i2c_write(uint8_t addr, uint8_t const * p_src, uint32_t len,
                      bool b_nostop);
int32_t hal_i2c_read(uint8_t addr, uint8_t * p_dst, uint32_t len);

/* Console UART0 receive (GP1), non-blocking. */
bool hal_uart0_getc(uint8_t * p_byte);

/* Hardware alarm used as the 1 kHz sensor sampler. */
void hal_alarm_start(uint32_t period_us);
void hal_alarm_ack_rearm(uint32_t period_us);

/* Memory barrier for data shared between tasks and cores. */
void hal_memory_barrier(void);

/* Hardware watchdog: resets the chip unless fed in time. */
void hal_watchdog_start(uint32_t timeout_ms);
void hal_watchdog_feed(void);
bool hal_watchdog_caused_reset(void);

#endif /* HAL_H */

/*** end of file ***/
