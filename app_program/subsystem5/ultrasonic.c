/** @file ultrasonic.c
 *
 * @brief Buddy 5 - HC-SR04 driver.
 *
 * distance_mm = echo_us * 343 m/s / 2 = echo_us * 0.1715 (HC-SR04
 * datasheet, "Formula").  A semaphore serialises the users (front monitor
 * task and scans) and the datasheet's 60 ms minimum measurement cycle is
 * enforced between pings.
 */

#include <stdint.h>
#include <tk/tkernel.h>
#include "car_types.h"
#include "car_config.h"
#include "hal.h"
#include "hal_irq.h"
#include "ultrasonic.h"

#define TRIG_PULSE_US (10u)
#define US_PER_MS     (1000u)
#define PING_GAP_US   (60000u)
#define MM_PER_US     (0.1715f)
#define FLG_ECHO      (1u << 0)
#define MAX_SAMPLES   (7u)

/* Kernel objects and echo capture, shared with the echo interrupt and the
   calling tasks: volatile (Rule 1.8.c). */
static volatile ID       gh_echo_flag   = 0;
static volatile ID       gh_sensor_sem  = 0;
static volatile uint32_t g_rise_us      = 0u;
static volatile uint32_t g_width_us     = 0u;
static volatile uint32_t g_last_ping_us = 0u;

static void sort_readings(int32_t * p_values, uint32_t count);
static void echo_edge_isr(uint32_t pin, uint32_t events, uint32_t t_us);

/*!
 * @brief Configure the pins, create the kernel objects and attach the
 *        echo interrupt.
 *
 * @return E_OK, E_LIMIT if a kernel object could not be created, or a
 *         hal_irq_gpio_attach() error.
 */
int32_t
ultrasonic_init (void)
{
    T_CFLG  cflg = {0};
    T_CSEM  csem = {0};
    int32_t ercd = E_LIMIT;

    hal_gpio_init_out(PIN_US_TRIG, false);
    hal_gpio_init_in(PIN_US_ECHO, false);

    cflg.flgatr   = TA_TFIFO | TA_WSGL;
    gh_echo_flag  = tk_cre_flg(&cflg);
    csem.sematr   = TA_TFIFO;
    csem.isemcnt  = 1;
    csem.maxsem   = 1;
    gh_sensor_sem = tk_cre_sem(&csem);

    if ((gh_echo_flag >= E_OK) && (gh_sensor_sem >= E_OK))
    {
        ercd = hal_irq_gpio_attach(PIN_US_ECHO,
                                   HAL_GPIO_EDGE_RISE | HAL_GPIO_EDGE_FALL,
                                   echo_edge_isr);
    }

    return (ercd);
}

/*!
 * @brief One ranging cycle (blocks for up to about 100 ms).
 *
 * @return Distance in mm, or ULTRASONIC_NO_ECHO.
 */
int32_t
ultrasonic_measure_mm (void)
{
    UINT     pattern = 0u;
    ER       ercd    = E_OK;
    int32_t  dist_mm = ULTRASONIC_NO_ECHO;
    uint32_t since   = 0u;

    (void) tk_wai_sem(gh_sensor_sem, 1, TMO_FEVR);

    /* Respect the minimum cycle time since the previous ping. */
    since = hal_time_us() - g_last_ping_us;

    if (since < PING_GAP_US)
    {
        (void) tk_dly_tsk(((PING_GAP_US - since) / US_PER_MS) + 1u);
    }

    (void) tk_clr_flg(gh_echo_flag, ~FLG_ECHO);
    g_rise_us = 0u;

    hal_gpio_put(PIN_US_TRIG, true);
    hal_delay_us(TRIG_PULSE_US);
    hal_gpio_put(PIN_US_TRIG, false);
    g_last_ping_us = hal_time_us();

    ercd = tk_wai_flg(gh_echo_flag, FLG_ECHO, TWF_ORW | TWF_CLR, &pattern,
                      US_TIMEOUT_MS);

    if (E_OK == ercd)
    {
        /* Casts: the echo lasts at most the timeout (tens of ms), which
           float32_t holds exactly; the product is at most a few metres in
           mm, so the truncation to int32_t cannot overflow. */
        dist_mm = (int32_t) ((float32_t) g_width_us * MM_PER_US);
        dist_mm = ((dist_mm < US_MIN_MM) || (dist_mm > US_MAX_MM))
                      ? ULTRASONIC_NO_ECHO
                      : dist_mm;
    }

    (void) tk_sig_sem(gh_sensor_sem, 1);

    return (dist_mm);
}

/*!
 * @brief Median of several readings (rejects single-ping outliers).
 *
 * @param[in] samples Pings to take, at most MAX_SAMPLES.
 *
 * @return Median distance in mm, or ULTRASONIC_NO_ECHO if no ping had an
 *         echo.
 */
int32_t
ultrasonic_median_mm (uint32_t samples)
{
    int32_t  readings[MAX_SAMPLES];
    uint32_t pings   = (samples > MAX_SAMPLES) ? MAX_SAMPLES : samples;
    uint32_t count   = 0u;
    uint32_t idx     = 0u;
    int32_t  dist_mm = ULTRASONIC_NO_ECHO;

    for (idx = 0u; idx < pings; idx++)
    {
        dist_mm = ultrasonic_measure_mm();

        if (ULTRASONIC_NO_ECHO != dist_mm)
        {
            readings[count] = dist_mm;
            count++;
        }
    }

    dist_mm = ULTRASONIC_NO_ECHO;

    if (count > 0u)
    {
        sort_readings(readings, count);
        dist_mm = readings[count / 2u];
    }

    return (dist_mm);
}

/*!
 * @brief Sort readings in ascending order (insertion sort, few values).
 *
 * @param[in,out] p_values Readings.
 * @param[in]     count    Number of readings.
 */
static void
sort_readings (int32_t * p_values, uint32_t count)
{
    uint32_t idx   = 0u;
    uint32_t pos   = 0u;
    int32_t  value = 0;

    for (idx = 1u; idx < count; idx++)
    {
        value = p_values[idx];
        pos   = idx;

        while ((pos > 0u) && (p_values[pos - 1u] > value))
        {
            p_values[pos] = p_values[pos - 1u];
            pos--;
        }

        p_values[pos] = value;
    }
}

/*!
 * @brief Echo edge callback (interrupt context): time the echo pulse and
 *        wake the waiting task on its falling edge.
 *
 * @param[in] pin    Echo pin (unused).
 * @param[in] events HAL_GPIO_EDGE_* bits.
 * @param[in] t_us   Time stamp of the interrupt.
 */
static void
echo_edge_isr (uint32_t pin, uint32_t events, uint32_t t_us)
{
    (void) pin;

    if (0u != (events & HAL_GPIO_EDGE_RISE))
    {
        g_rise_us = t_us;
    }

    if ((0u != (events & HAL_GPIO_EDGE_FALL)) && (0u != g_rise_us))
    {
        g_width_us = t_us - g_rise_us;
        g_rise_us  = 0u;
        (void) tk_set_flg(gh_echo_flag, FLG_ECHO);
    }
}

/*** end of file ***/
