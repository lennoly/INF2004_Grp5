/** @file hal.c
 *
 * @brief HAL implementation on the Raspberry Pi Pico C SDK.
 *
 * Only static-inline SDK APIs and hardware_structs register maps are used,
 * because the template links no SDK runtime (no clocks_init, no
 * pico_time).  The I2C driver is register level for the same reason; its
 * logic mirrors the SDK's i2c_write_blocking() and i2c_read_blocking()
 * (pico-sdk src/rp2_common/hardware_i2c/i2c.c) and the RP2040 datasheet,
 * section 4.3.  This file must NOT include <tk/tkernel.h> (see hal.h).
 *
 * NOTE: this is the one file compiled as GNU C11 rather than C99: the
 * Pico SDK headers it includes use C11 features (see the report's
 * deviations table).  The code in this file itself is plain C99.
 */

#include <stdint.h>
#include <stdbool.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/resets.h"
#include "hardware/irq.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/structs/i2c.h"
#include "hardware/structs/timer.h"
#include "hardware/structs/uart.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/psm.h"
#include "hal.h"
#include "hal_irq.h"

/* C99 compile-time checks (a negative array size is a compile error) that
   the IRQ numbers in hal_irq.h match the SDK's; C11 _Static_assert is not
   used because the application is written in C99 (Rule 1.1.a).
   Casts: the SDK's enumeration constants are small and non-negative. */
typedef char hal_irq_bank_matches_sdk_t
    [(HAL_IRQ_IO_BANK0 == (uint32_t) IO_IRQ_BANK0) ? 1 : -1];
typedef char hal_irq_timer_matches_sdk_t
    [(HAL_IRQ_TIMER3 == (uint32_t) TIMER_IRQ_3) ? 1 : -1];

#define GPIO_EVENTS_PER_REG  (8u)
#define GPIO_EVENT_BITS      (4u)
#define GPIO_EVENT_MASK      (0xFu)
#define SAMPLER_ALARM        (3u)
#define I2C_TX_FIFO_DEPTH    (16u)
#define I2C_FAST_MODE_MAX_HZ (1000000u)
#define I2C_PINS_PER_BLOCK   (2u) /* I2C0 and I2C1 alternate every 2 pins */
#define I2C_BLOCKS           (2u)
#define SCL_LOW_NUM          (3u) /* SCL low for 3/5 of the period        */
#define SCL_LOW_DEN          (5u)
#define SDA_HOLD_NUM         (3u) /* 300 ns hold below 1 MHz, else 120 ns */
#define SDA_HOLD_DEN_STD     (10000000u)
#define SDA_HOLD_DEN_FAST    (25000000u)
#define SPIKE_DIV            (16u) /* spike filter = lcnt / 16, at least 1 */
#define I2C_ERROR            (-1)
#define HALF_RANGE_US        (0x7FFFFFFFu) /* wrap-safe "in the past" test */
#define US_PER_MS            (1000u)
#define WATCHDOG_X_FACTOR    (2u) /* RP2040-E1: counts down twice per tick */
#define WATCHDOG_LOAD_PER_MS (US_PER_MS * WATCHDOG_X_FACTOR)
#define WATCHDOG_MAX_MS      (WATCHDOG_LOAD_BITS / WATCHDOG_LOAD_PER_MS)
#define WATCHDOG_SCRATCH     (4u)
#define WATCHDOG_MAGIC       (0x6ab73121u) /* as the SDK's watchdog_enable() */

/* Chosen by the init task and then used by the IMU task, or shared by a
   task and the sampler ISR: volatile (Rule 1.8.c). */
static i2c_hw_t * volatile gp_i2c       = i2c0_hw;
static volatile bool     gb_i2c_restart = false;
static volatile uint32_t g_alarm_target = 0u;

/* Watchdog reload value; written and used only by the vehicle task. */
static uint32_t g_watchdog_load = 0u;

static void gpio_route(uint32_t pin, uint32_t func);
static bool i2c_wait_bits(volatile uint32_t const * p_reg, uint32_t mask);
static void i2c_set_target(uint8_t addr);

/*!
 * @brief Free-running 1 MHz timer (wraps every 71.6 minutes).
 *
 * @return Microseconds since boot, modulo 2^32.
 */
uint32_t
hal_time_us (void)
{
    return (timer_hw->timerawl);
}

/*!
 * @brief Busy-wait; only for very short delays (e.g. the 10 us trigger).
 *
 * @param[in] delay_us Delay in microseconds.
 */
void
hal_delay_us (uint32_t delay_us)
{
    uint32_t start = timer_hw->timerawl;

    while ((timer_hw->timerawl - start) < delay_us)
    {
        /* Spin: the delay is shorter than any kernel tick. */
    }
}

/*!
 * @brief Configure a pin as an SIO output with an initial level.
 *
 * @param[in] pin     GPIO number.
 * @param[in] b_level Initial output level.
 */
void
hal_gpio_init_out (uint32_t pin, bool b_level)
{
    gpio_put(pin, b_level);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_route(pin, GPIO_FUNC_SIO);
}

/*!
 * @brief Configure a pin as an SIO input, optionally with a pull-up.
 *
 * @param[in] pin       GPIO number.
 * @param[in] b_pull_up true to enable the internal pull-up.
 */
void
hal_gpio_init_in (uint32_t pin, bool b_pull_up)
{
    gpio_set_dir(pin, GPIO_IN);
    gpio_route(pin, GPIO_FUNC_SIO);
    hw_write_masked(&pads_bank0_hw->io[pin],
                    b_pull_up ? PADS_BANK0_GPIO0_PUE_BITS : 0u,
                    PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
}

/*!
 * @brief Drive an output pin.
 *
 * @param[in] pin     GPIO number.
 * @param[in] b_level Output level.
 */
void
hal_gpio_put (uint32_t pin, bool b_level)
{
    gpio_put(pin, b_level);
}

/*!
 * @brief Read an input pin.
 *
 * @param[in] pin GPIO number.
 *
 * @return true if the pin is high.
 */
bool
hal_gpio_get (uint32_t pin)
{
    return (gpio_get(pin));
}

/*!
 * @brief Enable edge interrupts of a pin on core 0 (processor 1).
 *
 * @param[in] pin   GPIO number.
 * @param[in] edges HAL_GPIO_EDGE_* bits.
 */
void
hal_gpio_irq_enable (uint32_t pin, uint32_t edges)
{
    uint32_t shift = GPIO_EVENT_BITS * (pin % GPIO_EVENTS_PER_REG);
    uint32_t reg   = pin / GPIO_EVENTS_PER_REG;

    io_bank0_hw->intr[reg] = (GPIO_EVENT_MASK << shift); /* clear stale */
    hw_set_bits(&io_bank0_hw->proc0_irq_ctrl.inte[reg],
                (edges & GPIO_EVENT_MASK) << shift);
}

/*!
 * @brief Read and acknowledge the pending edge events of one pin.
 *
 * @param[in] pin GPIO number.
 *
 * @return HAL_GPIO_EDGE_* bits; 0 if nothing is pending.
 */
uint32_t
hal_gpio_irq_take (uint32_t pin)
{
    uint32_t shift  = GPIO_EVENT_BITS * (pin % GPIO_EVENTS_PER_REG);
    uint32_t reg    = pin / GPIO_EVENTS_PER_REG;
    uint32_t events = 0u;

    events = (io_bank0_hw->proc0_irq_ctrl.ints[reg] >> shift) & GPIO_EVENT_MASK;

    /* Edge events are write-1-to-clear. */
    if (0u != events)
    {
        io_bank0_hw->intr[reg] = (events << shift);
    }

    return (events);
}

/*!
 * @brief Route a pin to its PWM slice and start it at level 0.
 *
 * Period = (wrap + 1) * clkdiv / clk_sys.
 *
 * @param[in] pin    GPIO number.
 * @param[in] clkdiv Integer divider of clk_sys, 1..255.
 * @param[in] wrap   Counter TOP, 0..65535.
 */
void
hal_pwm_setup (uint32_t pin, uint32_t clkdiv, uint32_t wrap)
{
    uint32_t slice = pwm_gpio_to_slice_num(pin);

    gpio_route(pin, GPIO_FUNC_PWM);

    /* Casts: the documented ranges of clkdiv (1..255) and wrap
       (0..65535) fit the SDK's 8-bit and 16-bit parameters. */
    pwm_set_clkdiv_int_frac(slice, (uint8_t) clkdiv, 0u);
    pwm_set_wrap(slice, (uint16_t) wrap);
    pwm_set_gpio_level(pin, 0u);
    pwm_set_enabled(slice, true);
}

/*!
 * @brief Set the PWM compare level of a pin.
 *
 * @param[in] pin   GPIO number.
 * @param[in] level Compare level, 0..wrap + 1.
 */
void
hal_pwm_set_level (uint32_t pin, uint32_t level)
{
    /* Cast: level <= wrap + 1, and every wrap used is below 65535. */
    pwm_set_gpio_level(pin, (uint16_t) level);
}

/*!
 * @brief Take the ADC out of reset and enable it (as the SDK's
 *        adc_init()).
 */
void
hal_adc_init (void)
{
    reset_unreset_block_num_wait_blocking(RESET_ADC);
    adc_hw->cs = ADC_CS_EN_BITS;

    while (0u == (adc_hw->cs & ADC_CS_READY_BITS))
    {
        /* Wait for the ADC to power up (a few microseconds). */
    }
}

/*!
 * @brief Make a pin a high-impedance analog input (as the SDK's
 *        adc_gpio_init()).
 *
 * @param[in] pin GPIO number, 26..29.
 */
void
hal_adc_pin_init (uint32_t pin)
{
    /* Cast: GPIO_FUNC_NULL is a small non-negative enumeration value. */
    io_bank0_hw->io[pin].ctrl = (uint32_t) GPIO_FUNC_NULL
                                << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    hw_clear_bits(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_IE_BITS
                                               | PADS_BANK0_GPIO0_PUE_BITS
                                               | PADS_BANK0_GPIO0_PDE_BITS);
}

/*!
 * @brief Single blocking conversion (about 2 us).
 *
 * @param[in] channel ADC input, 0..3.
 *
 * @return 12-bit result.
 */
uint16_t
hal_adc_read (uint32_t channel)
{
    adc_select_input(channel);

    return (adc_read());
}

/*!
 * @brief Set up I2C0 or I2C1 (chosen from the SDA pin) as a fast-mode
 *        master.
 *
 * @param[in] sda     SDA pin.
 * @param[in] scl     SCL pin.
 * @param[in] baud_hz Bus speed.
 *
 * @return true (the set-up cannot fail).
 */
bool
hal_i2c_init (uint32_t sda, uint32_t scl, uint32_t baud_hz)
{
    uint32_t period    = 0u;
    uint32_t lcnt      = 0u;
    uint32_t hcnt      = 0u;
    uint32_t hold      = 0u;
    uint32_t reset_bit = 0u;

    gp_i2c =
        (0u == ((sda / I2C_PINS_PER_BLOCK) % I2C_BLOCKS)) ? i2c0_hw : i2c1_hw;
    reset_bit = (i2c0_hw == gp_i2c) ? RESET_I2C0 : RESET_I2C1;

    reset_block_num(reset_bit);
    reset_unreset_block_num_wait_blocking(reset_bit);

    gp_i2c->enable = 0u;
    gp_i2c->con =
        (I2C_IC_CON_SPEED_VALUE_FAST << I2C_IC_CON_SPEED_LSB)
        | I2C_IC_CON_MASTER_MODE_BITS | I2C_IC_CON_IC_SLAVE_DISABLE_BITS
        | I2C_IC_CON_IC_RESTART_EN_BITS | I2C_IC_CON_TX_EMPTY_CTRL_BITS;
    gp_i2c->tx_tl = 0u;
    gp_i2c->rx_tl = 0u;

    /* SCL timing with a 60/40 low/high split, as the SDK does. */
    period = (HAL_CLK_SYS_HZ + (baud_hz / 2u)) / baud_hz;
    lcnt   = (period * SCL_LOW_NUM) / SCL_LOW_DEN;
    hcnt   = period - lcnt;

    /* SDA hold time in clk_sys cycles. */
    if (baud_hz < I2C_FAST_MODE_MAX_HZ)
    {
        hold = ((HAL_CLK_SYS_HZ * SDA_HOLD_NUM) / SDA_HOLD_DEN_STD) + 1u;
    }
    else
    {
        hold = ((HAL_CLK_SYS_HZ * SDA_HOLD_NUM) / SDA_HOLD_DEN_FAST) + 1u;
    }

    gp_i2c->fs_scl_hcnt = hcnt;
    gp_i2c->fs_scl_lcnt = lcnt;
    gp_i2c->fs_spklen   = (lcnt < SPIKE_DIV) ? 1u : (lcnt / SPIKE_DIV);
    hw_write_masked(&gp_i2c->sda_hold,
                    hold << I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_LSB,
                    I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_BITS);
    gp_i2c->enable = 1u;

    /* Pins, with the internal pull-ups as a fallback for the module's. */
    gpio_route(sda, GPIO_FUNC_I2C);
    gpio_route(scl, GPIO_FUNC_I2C);
    hw_write_masked(&pads_bank0_hw->io[sda], PADS_BANK0_GPIO0_PUE_BITS,
                    PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
    hw_write_masked(&pads_bank0_hw->io[scl], PADS_BANK0_GPIO0_PUE_BITS,
                    PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
    gb_i2c_restart = false;

    return (true);
}

/*!
 * @brief Write bytes to an I2C target.
 *
 * @param[in] addr     7-bit target address.
 * @param[in] p_src    Bytes to send.
 * @param[in] len      Number of bytes (at least 1).
 * @param[in] b_nostop true to keep the bus for a repeated start.
 *
 * @return Bytes written, or -1 on NACK, abort or timeout.
 */
int32_t
hal_i2c_write (uint8_t addr, uint8_t const * p_src, uint32_t len, bool b_nostop)
{
    uint32_t idx     = 0u;
    uint32_t cmd     = 0u;
    bool     b_abort = false;
    bool     b_first = false;
    bool     b_stop  = false;
    int32_t  result  = I2C_ERROR;

    i2c_set_target(addr);

    for (idx = 0u; (idx < len) && (!b_abort); idx++)
    {
        b_first = (0u == idx);
        b_stop  = ((len - 1u) == idx) && (!b_nostop);

        /* Cast: widens the data byte into the 32-bit command word. */
        cmd = (uint32_t) p_src[idx];
        cmd |= (b_first && gb_i2c_restart) ? I2C_IC_DATA_CMD_RESTART_BITS : 0u;
        cmd |= b_stop ? I2C_IC_DATA_CMD_STOP_BITS : 0u;
        gp_i2c->data_cmd = cmd;

        b_abort = !i2c_wait_bits(&gp_i2c->raw_intr_stat,
                                 I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS);

        /* Reading clr_tx_abrt clears the abort. */
        if (0u != gp_i2c->tx_abrt_source)
        {
            (void) gp_i2c->clr_tx_abrt;
            b_abort = true;
        }

        if (b_abort || b_stop)
        {
            (void) i2c_wait_bits(&gp_i2c->raw_intr_stat,
                                 I2C_IC_RAW_INTR_STAT_STOP_DET_BITS);
            (void) gp_i2c->clr_stop_det;
        }
    }

    gb_i2c_restart = b_nostop;

    /* Cast: len is a transfer length of a few bytes. */
    result = b_abort ? I2C_ERROR : (int32_t) len;

    return (result);
}

/*!
 * @brief Read bytes from an I2C target (always ends with STOP).
 *
 * @param[in]  addr  7-bit target address.
 * @param[out] p_dst Destination.
 * @param[in]  len   Number of bytes (at least 1).
 *
 * @return Bytes read, or -1 on NACK, abort or timeout.
 */
int32_t
hal_i2c_read (uint8_t addr, uint8_t * p_dst, uint32_t len)
{
    uint32_t idx     = 0u;
    uint32_t cmd     = 0u;
    uint32_t start   = 0u;
    bool     b_abort = false;
    int32_t  result  = I2C_ERROR;

    i2c_set_target(addr);

    for (idx = 0u; (idx < len) && (!b_abort); idx++)
    {
        /* A read request, with RESTART and STOP where needed. */
        cmd = I2C_IC_DATA_CMD_CMD_BITS;
        cmd |=
            ((0u == idx) && gb_i2c_restart) ? I2C_IC_DATA_CMD_RESTART_BITS : 0u;
        cmd |= ((len - 1u) == idx) ? I2C_IC_DATA_CMD_STOP_BITS : 0u;

        while (gp_i2c->txflr >= I2C_TX_FIFO_DEPTH)
        {
            /* Wait for command FIFO space; the FIFO always drains. */
        }

        gp_i2c->data_cmd = cmd;
        start            = timer_hw->timerawl;

        /* Reading clr_tx_abrt reports and clears an abort. */
        while ((0u == gp_i2c->rxflr) && (!b_abort))
        {
            b_abort = (0u != gp_i2c->clr_tx_abrt)
                      || ((timer_hw->timerawl - start) > HAL_I2C_TIMEOUT_US);
        }

        /* Cast: the received byte is in the low 8 bits of data_cmd. */
        if (!b_abort)
        {
            p_dst[idx] = (uint8_t) gp_i2c->data_cmd;
        }
    }

    gb_i2c_restart = false;

    /* Cast: len is a transfer length of a few bytes. */
    result = b_abort ? I2C_ERROR : (int32_t) len;

    return (result);
}

/*!
 * @brief Take one byte from the UART0 receive FIFO without waiting.
 *
 * The template's tm_getchar() spins with interrupts disabled until a byte
 * arrives, which would freeze the whole system, so the application polls
 * the FIFO itself.  UART0 is configured by the template's T-Monitor
 * (115200 8N1, GP1 = RX); nothing else reads its receive side.  Bytes with
 * framing, parity, break or overrun errors (e.g. from a floating RX pin)
 * are discarded.
 *
 * @param[out] p_byte Received byte.
 *
 * @return true if a valid byte was stored in *p_byte.
 */
bool
hal_uart0_getc (uint8_t * p_byte)
{
    uint32_t data_reg = 0u;
    bool     b_got    = false;

    while ((!b_got) && (0u == (uart0_hw->fr & UART_UARTFR_RXFE_BITS)))
    {
        data_reg = uart0_hw->dr;
        b_got    = (0u
                 == (data_reg
                     & (UART_UARTDR_OE_BITS | UART_UARTDR_BE_BITS
                        | UART_UARTDR_PE_BITS | UART_UARTDR_FE_BITS)));

        /* Cast: the data field is the low 8 bits. */
        if (b_got)
        {
            *p_byte = (uint8_t) (data_reg & UART_UARTDR_DATA_BITS);
        }
    }

    return (b_got);
}

/*!
 * @brief Start hardware alarm 3 as a drift-free periodic sampler.
 *
 * @param[in] period_us Period in microseconds.
 */
void
hal_alarm_start (uint32_t period_us)
{
    timer_hw->intr = 1u << SAMPLER_ALARM;
    hw_set_bits(&timer_hw->inte, 1u << SAMPLER_ALARM);
    g_alarm_target = timer_hw->timerawl + period_us;

    /* Writing the alarm register arms it. */
    timer_hw->alarm[SAMPLER_ALARM] = g_alarm_target;
}

/*!
 * @brief Acknowledge the alarm and schedule the next one a period later
 *        (interrupt context).
 *
 * @param[in] period_us Period in microseconds.
 */
void
hal_alarm_ack_rearm (uint32_t period_us)
{
    uint32_t ahead = 0u;

    timer_hw->intr = 1u << SAMPLER_ALARM;
    g_alarm_target += period_us;

    /* If the target is not in the future (e.g. after a debugger halt),
       resynchronise to "now"; the unsigned difference is wrap-safe. */
    ahead = g_alarm_target - timer_hw->timerawl;

    if ((0u == ahead) || (ahead > HALF_RANGE_US))
    {
        g_alarm_target = timer_hw->timerawl + period_us;
    }

    timer_hw->alarm[SAMPLER_ALARM] = g_alarm_target;
}

/*!
 * @brief Data memory barrier (DMB) plus compiler barrier.
 *
 * Used by the lock-free rings shared between tasks and the USB / lwIP
 * service tasks: the producer writes the data, calls this, then publishes
 * the index (template safety rule 3).
 */
void
hal_memory_barrier (void)
{
    /* WARNING: inline assembly, kept to this one driver function
       (Rule 1.1.c).  DMB orders all earlier memory accesses before later
       ones; the "memory" clobber also stops the compiler reordering. */
    __asm__ volatile("dmb" ::: "memory");
}

/*!
 * @brief Start the hardware watchdog, as the SDK's watchdog_enable() does:
 *        unless hal_watchdog_feed() is called within timeout_ms, every block
 *        except the oscillators is reset, which also turns the motor
 *        outputs off.  The counter runs on the same 1 us tick as the TIMER
 *        and pauses while a debugger halts a core.
 *
 * @param[in] timeout_ms Timeout in ms, limited to WATCHDOG_MAX_MS (about
 *                       8.3 s); 0 leaves the watchdog off.
 */
void
hal_watchdog_start (uint32_t timeout_ms)
{
    uint32_t load = WATCHDOG_LOAD_BITS;

    if (timeout_ms > 0u)
    {
        /* RP2040-E1: the counter decrements twice per tick, so the load is
           doubled; the limit check keeps the product inside 24 bits. */
        if (timeout_ms < WATCHDOG_MAX_MS)
        {
            load = timeout_ms * WATCHDOG_LOAD_PER_MS;
        }

        hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
        hw_set_bits(&psm_hw->wdsel,
                    PSM_WDSEL_BITS
                        & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS));
        hw_set_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_PAUSE_DBG0_BITS
                                            | WATCHDOG_CTRL_PAUSE_DBG1_BITS
                                            | WATCHDOG_CTRL_PAUSE_JTAG_BITS);
        watchdog_hw->scratch[WATCHDOG_SCRATCH] = WATCHDOG_MAGIC;
        g_watchdog_load                        = load;
        watchdog_hw->load                      = load;
        hw_set_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    }
}

/*!
 * @brief Reload the watchdog counter; nothing happens before
 *        hal_watchdog_start().
 */
void
hal_watchdog_feed (void)
{
    if (0u != g_watchdog_load)
    {
        watchdog_hw->load = g_watchdog_load;
    }
}

/*!
 * @brief Whether the last reset was a timeout of hal_watchdog_start()'s
 *        watchdog (the SDK's watchdog_enable_caused_reboot() test; a
 *        power-on clears the scratch marker).
 *
 * @return true after a watchdog reset.
 */
bool
hal_watchdog_caused_reset (void)
{
    return ((0u != watchdog_hw->reason)
            && (WATCHDOG_MAGIC == watchdog_hw->scratch[WATCHDOG_SCRATCH]));
}

/*!
 * @brief Route a pin to a peripheral function (as the SDK's
 *        gpio_set_function()).
 *
 * @param[in] pin  GPIO number.
 * @param[in] func GPIO_FUNC_* value.
 */
static void
gpio_route (uint32_t pin, uint32_t func)
{
    hw_write_masked(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_IE_BITS,
                    PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);
    io_bank0_hw->io[pin].ctrl = func << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
}

/*!
 * @brief Wait until (*p_reg & mask) != 0, or time out.
 *
 * @param[in] p_reg Register to poll.
 * @param[in] mask  Bits to wait for.
 *
 * @return true if a bit was set before HAL_I2C_TIMEOUT_US.
 */
static bool
i2c_wait_bits (volatile uint32_t const * p_reg, uint32_t mask)
{
    uint32_t start     = timer_hw->timerawl;
    bool     b_set     = false;
    bool     b_expired = false;

    while ((!b_set) && (!b_expired))
    {
        b_set     = (0u != (*p_reg & mask));
        b_expired = ((timer_hw->timerawl - start) > HAL_I2C_TIMEOUT_US);
    }

    return (b_set);
}

/*!
 * @brief Point the controller at a 7-bit target address.
 *
 * @param[in] addr Target address.
 */
static void
i2c_set_target (uint8_t addr)
{
    gp_i2c->enable = 0u;
    gp_i2c->tar    = addr;
    gp_i2c->enable = 1u;
}

/*** end of file ***/
