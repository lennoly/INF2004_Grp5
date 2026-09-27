/** @file hal_irq.c
 *
 * @brief Binds RP2040 interrupts to micro T-Kernel interrupt handlers.
 *
 * One GPIO bank handler fans out to per-pin callbacks (encoders,
 * ultrasonic echo); hardware alarm 3 drives the 1 kHz IR sampler.  All
 * handlers run on processor 1 (RP2040 core 0), satisfying the template's
 * single-owner IRQ rule.
 *
 * NOTE: on the Cortex-M0+ an interrupt handler is an ordinary AAPCS
 * function (the hardware stacks the caller-saved registers), and the
 * kernel's TA_HLNG wrapper calls these handlers, so no compiler-specific
 * interrupt keyword is needed (Rule 6.5.a; see the report's deviations).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "hal.h"
#include "hal_irq.h"

#define MAX_GPIO_CALLBACKS (6u)
#define DEFAULT_PERIOD_US  (1000u)

typedef struct
{
    uint32_t          pin;
    hal_irq_gpio_cb_t callback;
} gpio_slot_t;

/* Filled in by tasks and read by the ISRs: volatile (Rule 1.8.c). */
static volatile gpio_slot_t       g_gpio_slots[MAX_GPIO_CALLBACKS];
static volatile uint32_t          g_gpio_count     = 0u;
static volatile hal_irq_tick_cb_t g_tick_cb        = NULL;
static volatile uint32_t          g_tick_period_us = DEFAULT_PERIOD_US;

/* Used only by the task that initialises the drivers. */
static bool gb_gpio_defined = false;

static void gpio_bank_isr(UINT intno);
static void sampler_isr(UINT intno);

/*!
 * @brief Register an edge callback for a pin (call from task context).
 *
 * @param[in] pin      GPIO number.
 * @param[in] edges    HAL_GPIO_EDGE_* bits to enable.
 * @param[in] callback Function called in interrupt context.
 *
 * @return E_OK, E_LIMIT if the table is full, or a tk_def_int() error.
 */
int32_t
hal_irq_gpio_attach (uint32_t pin, uint32_t edges, hal_irq_gpio_cb_t callback)
{
    T_DINT dint;
    ER     ercd         = E_OK;
    UINT   imask        = 0u;
    bool   b_registered = false;

    if (g_gpio_count >= MAX_GPIO_CALLBACKS)
    {
        ercd = E_LIMIT;
    }
    else
    {
        DI(imask);
        g_gpio_slots[g_gpio_count].pin      = pin;
        g_gpio_slots[g_gpio_count].callback = callback;
        g_gpio_count++;
        EI(imask);
        b_registered = true;
    }

    /* The bank handler is defined and enabled once, for the first pin. */
    if ((E_OK == ercd) && (!gb_gpio_defined))
    {
        dint.intatr = TA_HLNG;

        /* Cast: the kernel stores every handler as the generic FP type and
           calls a TA_HLNG handler with its interrupt number, matching
           gpio_bank_isr(). */
        dint.inthdr     = (FP) gpio_bank_isr;
        ercd            = tk_def_int(HAL_IRQ_IO_BANK0, &dint);
        gb_gpio_defined = (E_OK == ercd);

        if (gb_gpio_defined)
        {
            ClearInt(HAL_IRQ_IO_BANK0);
            EnableInt(HAL_IRQ_IO_BANK0, HAL_IRQ_PRIORITY);
        }
    }

    /* A registered pin's edges are enabled even if the bank handler could
       not be defined yet: a later attach retries the definition. */
    if (b_registered)
    {
        hal_gpio_irq_enable(pin, edges);
    }

    return (ercd);
}

/*!
 * @brief Start the periodic hardware-alarm sampler.
 *
 * @param[in] period_us Tick period in microseconds.
 * @param[in] callback  Function called in interrupt context every tick.
 *
 * @return E_OK or a tk_def_int() error.
 */
int32_t
hal_irq_sampler_attach (uint32_t period_us, hal_irq_tick_cb_t callback)
{
    T_DINT dint;
    ER     ercd = E_OK;

    g_tick_cb        = callback;
    g_tick_period_us = period_us;
    dint.intatr      = TA_HLNG;

    /* Cast: as in hal_irq_gpio_attach(), TA_HLNG handlers take the
       interrupt number, matching sampler_isr(). */
    dint.inthdr = (FP) sampler_isr;
    ercd        = tk_def_int(HAL_IRQ_TIMER3, &dint);

    if (E_OK == ercd)
    {
        ClearInt(HAL_IRQ_TIMER3);
        EnableInt(HAL_IRQ_TIMER3, HAL_IRQ_PRIORITY);
        hal_alarm_start(period_us);
    }

    return (ercd);
}

/*!
 * @brief IO_IRQ_BANK0 handler: time stamp once, dispatch per pin.
 *
 * @param[in] intno Interrupt number (unused).
 */
static void
gpio_bank_isr (UINT intno)
{
    uint32_t now    = hal_time_us();
    uint32_t idx    = 0u;
    uint32_t events = 0u;

    (void) intno;

    for (idx = 0u; idx < g_gpio_count; idx++)
    {
        events = hal_gpio_irq_take(g_gpio_slots[idx].pin);

        if (0u != events)
        {
            g_gpio_slots[idx].callback(g_gpio_slots[idx].pin, events, now);
        }
    }
}

/*!
 * @brief TIMER_IRQ_3 handler: re-arm first (no drift), then sample.
 *
 * @param[in] intno Interrupt number (unused).
 */
static void
sampler_isr (UINT intno)
{
    uint32_t          now      = hal_time_us();
    hal_irq_tick_cb_t callback = g_tick_cb;

    (void) intno;
    hal_alarm_ack_rearm(g_tick_period_us);

    if (NULL != callback)
    {
        callback(now);
    }
}

/*** end of file ***/
