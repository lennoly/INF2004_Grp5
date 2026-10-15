/** @file encoder.c
 *
 * @brief Buddy 2 - wheel encoder capture and speed estimation.
 */

#include <tk/tkernel.h>
#include "car_config.h"
#include "car_math.h"
#include "hal.h"
#include "hal_irq.h"
#include "encoder.h"

#define PERIOD_SLOTS   (4u)         /* moving average of 4 periods */
#define US_PER_S       (1000000.0f) /* microseconds per second */
#define TICKS_PER_SLOT (2.0f)       /* both edges counted          */
/* Cast: the slot count is a small integer, which float32_t holds exactly. */
#define SLOTS_PER_TURN ((float32_t) ENCODER_SLOTS)
#define MM_PER_SLOT    ((CAR_MATH_PI * WHEEL_DIAMETER_MM) / SLOTS_PER_TURN)

typedef struct
{
    uint32_t ticks;
    uint32_t last_edge_us;
    uint32_t last_rise_us;
    uint32_t period_us[PERIOD_SLOTS];
    uint32_t n_periods;
    uint32_t head;
} wheel_state_t;

/* Written by the GPIO interrupt, read by the motion task (Rule 1.8.c). */
static volatile wheel_state_t g_wheel[ENCODER_COUNT];

static void encoder_edge_isr(uint32_t pin, uint32_t events, uint32_t t_us);

/*!
 * @brief Configure both encoder inputs and their edge interrupts.
 *
 * @return E_OK or a kernel error code from hal_irq_gpio_attach().
 */
int32_t
encoder_init (void)
{
    int32_t ercd = E_OK;

    hal_gpio_init_in(PIN_ENCODER_L, true);
    hal_gpio_init_in(PIN_ENCODER_R, true);
    ercd = hal_irq_gpio_attach(PIN_ENCODER_L,
                               HAL_GPIO_EDGE_RISE | HAL_GPIO_EDGE_FALL,
                               encoder_edge_isr);

    if (E_OK == ercd)
    {
        ercd = hal_irq_gpio_attach(PIN_ENCODER_R,
                                   HAL_GPIO_EDGE_RISE | HAL_GPIO_EDGE_FALL,
                                   encoder_edge_isr);
    }

    return (ercd);
}

/*!
 * @brief Consistent snapshot of one wheel with a speed estimate.
 *
 * Speed = slot length / mean period.  If the time since the last rising
 * edge already exceeds that mean, the wheel must be slowing, so the
 * longer interval is used instead; this gives a fast, monotonic decay to
 * zero.
 *
 * @param[in]  wheel ENCODER_LEFT or ENCODER_RIGHT.
 * @param[out] p_out Tick count and speed.
 */
void
encoder_read (uint32_t wheel, encoder_reading_t * p_out)
{
    wheel_state_t snap;
    UINT          imask = 0u;
    uint32_t      idx   = 0u;
    uint32_t      sum   = 0u;
    uint32_t      mean  = 0u;
    uint32_t      since = 0u;

    /* Cast: drops the volatile qualifier for one whole-struct copy, which
       is safe because the interrupt cannot run while it is disabled. */
    DI(imask);
    snap = *(wheel_state_t const *) &g_wheel[wheel];
    EI(imask);

    since             = hal_time_us() - snap.last_rise_us;
    p_out->ticks      = snap.ticks;
    p_out->speed_mm_s = 0.0f;

    if ((0u != snap.n_periods) && (since <= ENCODER_TIMEOUT_US))
    {
        for (idx = 0u; idx < snap.n_periods; idx++)
        {
            sum += snap.period_us[idx];
        }

        mean = sum / snap.n_periods;
        mean = (since > mean) ? since : mean;

        /* Cast: mean is a period in microseconds, below the 2^24 range
           float32_t holds exactly (the timeout is far shorter). */
        p_out->speed_mm_s = (MM_PER_SLOT * US_PER_S) / (float32_t) mean;
    }
}

/*!
 * @brief Distance represented by one counted edge.
 *
 * @return Millimetres per tick.
 */
float32_t
encoder_mm_per_tick (void)
{
    return (MM_PER_SLOT / TICKS_PER_SLOT);
}

/*!
 * @brief GPIO edge callback (interrupt context): count the edge and
 *        capture the rising-to-rising period.
 *
 * @param[in] pin    Encoder pin that changed.
 * @param[in] events HAL_GPIO_EDGE_* bits.
 * @param[in] t_us   Time stamp of the interrupt.
 */
static void
encoder_edge_isr (uint32_t pin, uint32_t events, uint32_t t_us)
{
    volatile wheel_state_t * p_wheel = NULL;
    bool                     b_valid = false;
    bool                     b_rise  = false;

    p_wheel = &g_wheel[(PIN_ENCODER_L == pin) ? ENCODER_LEFT : ENCODER_RIGHT];
    b_valid = ((t_us - p_wheel->last_edge_us) >= ENCODER_GLITCH_US);
    b_rise  = b_valid && (0u != (events & HAL_GPIO_EDGE_RISE));

    /* Edges closer than ENCODER_GLITCH_US are comparator chatter. */
    if (b_valid)
    {
        p_wheel->last_edge_us = t_us;
        p_wheel->ticks++;
    }

    /* A rising edge after an earlier one completes a period. */
    if (b_rise && (0u != p_wheel->last_rise_us))
    {
        p_wheel->period_us[p_wheel->head] = t_us - p_wheel->last_rise_us;
        p_wheel->head                     = (p_wheel->head + 1u) % PERIOD_SLOTS;
        p_wheel->n_periods += (p_wheel->n_periods < PERIOD_SLOTS) ? 1u : 0u;
    }

    if (b_rise)
    {
        p_wheel->last_rise_us = t_us;
    }
}

/*** end of file ***/
