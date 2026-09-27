/** @file ir_sensor.c
 *
 * @brief Buddy 3 - IR sampler ISR, calibration and barcode edge stream.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "car_types.h"
#include "car_config.h"
#include "hal.h"
#include "hal_irq.h"
#include "ir_sensor.h"

#define ADC_MAX        (4095u)
#define PERMILLE       (1000)
#define PERMILLE_F     (1000.0f)
#define LINE_EMA_SHIFT (2u)   /* alpha = 1/4 at 1 kHz (about 4 ms)   */
#define BAR_EMA_SHIFT  (1u)   /* alpha = 1/2 keeps the edges sharp   */
#define Q4_SHIFT       (4u)   /* the filter keeps 4 fraction bits    */
#define RING_SIZE      (64u)  /* power of two                        */
#define MIN_CAL_SPAN   (200u) /* reject a calibration with no swing  */

/* Casts: the configured levels are fractions 0..1, so the products are
   0..1000 and convert to int32_t exactly. */
#define BAR_ON_PERMILLE  ((int32_t) (BARCODE_BLACK_LEVEL * PERMILLE_F))
#define BAR_OFF_PERMILLE ((int32_t) (BARCODE_WHITE_LEVEL * PERMILLE_F))

static uint32_t const g_channel[IR_COUNT] = {ADC_CH_IR_LEFT, ADC_CH_IR_RIGHT,
                                             ADC_CH_IR_BARCODE};

/* Filter state and calibration, shared between the sampler ISR and the
   tasks: volatile (Rule 1.8.c). */
static volatile uint32_t g_filt[IR_COUNT]; /* raw << 4 (Q4) */
static volatile uint16_t g_white[IR_COUNT];
static volatile uint16_t g_black[IR_COUNT];
static volatile uint16_t g_cal_min[IR_COUNT];
static volatile uint16_t g_cal_max[IR_COUNT];
static volatile bool     gb_calibrating = false;

/* Barcode single-producer (ISR) / single-consumer (task) ring. */
static volatile ir_sensor_element_t g_ring[RING_SIZE];
static volatile uint32_t            g_ring_head   = 0u;
static volatile uint32_t            g_ring_tail   = 0u;
static volatile bool                gb_bar_black  = false;
static volatile uint32_t            g_bar_edge_us = 0u;

static int32_t to_permille(uint32_t raw, uint32_t channel);
static void    barcode_edge_detect(uint32_t raw, uint32_t t_us);
static void    ir_sample_isr(uint32_t t_us);

/*!
 * @brief Configure the analog pins and the ADC, load the default
 *        calibration and start the 1 kHz sampler.
 *
 * @return E_OK or a hal_irq_sampler_attach() error.
 */
int32_t
ir_sensor_init (void)
{
    uint32_t idx = 0u;

    hal_adc_init();
    hal_adc_pin_init(PIN_IR_LEFT);
    hal_adc_pin_init(PIN_IR_RIGHT);
    hal_adc_pin_init(PIN_IR_BARCODE);

    for (idx = 0u; idx < IR_COUNT; idx++)
    {
#if IR_BLACK_IS_HIGH
        g_white[idx] = IR_DEFAULT_WHITE;
        g_black[idx] = IR_DEFAULT_BLACK;
#else
        g_white[idx] = IR_DEFAULT_BLACK;
        g_black[idx] = IR_DEFAULT_WHITE;
#endif

        /* Cast: widens the 12-bit value before the shift (Rule 5.3.c). */
        g_filt[idx] = (uint32_t) g_white[idx] << Q4_SHIFT;
    }

    g_bar_edge_us = hal_time_us();

    return (hal_irq_sampler_attach(IR_SAMPLE_PERIOD_US, ir_sample_isr));
}

/*!
 * @brief Snapshot of all three channels.
 *
 * @param[out] p_out Filtered counts and normalised values.
 */
void
ir_sensor_read (ir_sensor_snapshot_t * p_out)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < IR_COUNT; idx++)
    {
        /* Casts: the filtered value is at most ADC_MAX (12 bits), and the
           permille value (0..1000) converts to float32_t exactly. */
        p_out->raw[idx] = (uint16_t) (g_filt[idx] >> Q4_SHIFT);
        p_out->norm[idx] =
            (float32_t) to_permille(p_out->raw[idx], idx) / PERMILLE_F;
    }
}

/*!
 * @brief Start the minimum / maximum capture (sweep the sensors over the
 *        line and the floor).
 */
void
ir_sensor_calib_begin (void)
{
    uint32_t idx   = 0u;
    UINT     imask = 0u;

    DI(imask);

    for (idx = 0u; idx < IR_COUNT; idx++)
    {
        g_cal_min[idx] = ADC_MAX;
        g_cal_max[idx] = 0u;
    }

    gb_calibrating = true;
    EI(imask);
}

/*!
 * @brief Stop the capture and apply it per channel (polarity from the
 *        configuration).
 *
 * A channel that saw too little contrast keeps its old calibration.
 *
 * @return false if a line channel saw too little contrast.
 */
bool
ir_sensor_calib_end (void)
{
    uint32_t idx       = 0u;
    bool     b_ok      = true;
    bool     b_span_ok = false;
    UINT     imask     = 0u;

    DI(imask);
    gb_calibrating = false;

    for (idx = 0u; idx < IR_COUNT; idx++)
    {
        /* Casts: widen the 12-bit values so the sum cannot overflow and
           no promoted signed int meets the unsigned constant. */
        b_span_ok = ((uint32_t) g_cal_max[idx]
                     >= ((uint32_t) g_cal_min[idx] + MIN_CAL_SPAN));

        if (b_span_ok)
        {
#if IR_BLACK_IS_HIGH
            g_white[idx] = g_cal_min[idx];
            g_black[idx] = g_cal_max[idx];
#else
            g_white[idx] = g_cal_max[idx];
            g_black[idx] = g_cal_min[idx];
#endif
        }
        else if (IR_BARCODE != idx)
        {
            b_ok = false;
        }
        else
        {
            /* The barcode channel may miss a bar during the sweep. */
        }
    }

    EI(imask);

    return (b_ok);
}

/*!
 * @brief Take one completed bar / space element (task context).
 *
 * @param[out] p_out Element.
 *
 * @return true if an element was taken.
 */
bool
ir_sensor_barcode_pop (ir_sensor_element_t * p_out)
{
    bool b_found = (g_ring_tail != g_ring_head);

    if (b_found)
    {
        hal_memory_barrier();
        p_out->b_bar    = g_ring[g_ring_tail].b_bar;
        p_out->width_us = g_ring[g_ring_tail].width_us;
        hal_memory_barrier();
        g_ring_tail = (g_ring_tail + 1u) & (RING_SIZE - 1u);
    }

    return (b_found);
}

/*!
 * @brief Current state of the barcode channel.
 *
 * @return true while the barcode sensor sees black.
 */
bool
ir_sensor_barcode_is_black (void)
{
    return (gb_bar_black);
}

/*!
 * @brief Time since the last barcode-channel transition.
 *
 * @return Microseconds.
 */
uint32_t
ir_sensor_barcode_idle_us (void)
{
    return (hal_time_us() - g_bar_edge_us);
}

/*!
 * @brief Map raw counts to 0..1000 permille black (polarity aware).
 *
 * @param[in] raw     Filtered ADC counts (0..4095).
 * @param[in] channel IR_LEFT, IR_RIGHT or IR_BARCODE.
 *
 * @return Permille black, 0..1000 (0 if the calibration has no span).
 */
static int32_t
to_permille (uint32_t raw, uint32_t channel)
{
    /* Casts: calibration values and raw counts are 12-bit ADC values, so
       they convert to int32_t exactly. */
    int32_t white = (int32_t) g_white[channel];
    int32_t black = (int32_t) g_black[channel];
    int32_t span  = black - white; /* negative if the sensor is inverted */
    int32_t value = 0;

    /* Cast: raw is a 12-bit ADC value, exact in int32_t; the product is
       at most 4095 * 1000, far inside the int32_t range. */
    if (0 != span)
    {
        value = (((int32_t) raw - white) * PERMILLE) / span;
        value = (value < 0) ? 0 : value;
        value = (value > PERMILLE) ? PERMILLE : value;
    }

    return (value);
}

/*!
 * @brief Hysteresis on the barcode channel; emit completed elements
 *        (interrupt context).
 *
 * @param[in] raw  Filtered barcode-channel counts.
 * @param[in] t_us Sample time stamp.
 */
static void
barcode_edge_detect (uint32_t raw, uint32_t t_us)
{
    int32_t  level     = to_permille(raw, IR_BARCODE);
    bool     b_black   = gb_bar_black;
    bool     b_changed = false;
    uint32_t next      = 0u;

    b_changed = ((!b_black) && (level >= BAR_ON_PERMILLE))
                || (b_black && (level <= BAR_OFF_PERMILLE));

    if (b_changed)
    {
        /* The element that just ended; dropped if the consumer is late. */
        next = (g_ring_head + 1u) & (RING_SIZE - 1u);

        if (next != g_ring_tail)
        {
            g_ring[g_ring_head].b_bar    = b_black;
            g_ring[g_ring_head].width_us = t_us - g_bar_edge_us;
            hal_memory_barrier();
            g_ring_head = next;
        }

        gb_bar_black  = !b_black;
        g_bar_edge_us = t_us;
    }
}

/*!
 * @brief 1 kHz sampler (interrupt context, about 10 us).
 *
 * @param[in] t_us Sample time stamp.
 */
static void
ir_sample_isr (uint32_t t_us)
{
    uint32_t idx    = 0u;
    uint32_t raw    = 0u;
    uint32_t shift  = 0u;
    uint32_t target = 0u;

    for (idx = 0u; idx < IR_COUNT; idx++)
    {
        raw   = hal_adc_read(g_channel[idx]);
        shift = (IR_BARCODE == idx) ? BAR_EMA_SHIFT : LINE_EMA_SHIFT;

        /* EMA in Q4 fixed point, f += (x - f) >> shift, written with
           unsigned arithmetic only (Rules 5.3.b and 5.3.c). */
        target = raw << Q4_SHIFT;

        if (target >= g_filt[idx])
        {
            g_filt[idx] += (target - g_filt[idx]) >> shift;
        }
        else
        {
            g_filt[idx] -= (g_filt[idx] - target) >> shift;
        }

        raw = g_filt[idx] >> Q4_SHIFT;

        /* Casts: raw is at most ADC_MAX (12 bits), so it fits uint16_t. */
        if (gb_calibrating)
        {
            g_cal_min[idx] =
                (raw < g_cal_min[idx]) ? (uint16_t) raw : g_cal_min[idx];
            g_cal_max[idx] =
                (raw > g_cal_max[idx]) ? (uint16_t) raw : g_cal_max[idx];
        }
    }

    barcode_edge_detect(g_filt[IR_BARCODE] >> Q4_SHIFT, t_us);
}

/*** end of file ***/
