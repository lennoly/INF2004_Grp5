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
#define WIDTH_MAX_US  (0xFFFFu) /* echo width field saturates here */
#define FLG_ECHO      (1u << 0)
#define MAX_SAMPLES   (7u)

/* Kernel objects and echo capture, shared with the echo interrupt and the
   calling tasks: volatile (Rule 1.8.c). */
static volatile ID       gh_echo_flag   = 0;
static volatile ID       gh_sensor_sem  = 0;
static volatile uint32_t g_rise_us      = 0u;
/* Change Requirement 1: the echo width in 16 bits, saturated at 65 535 us
   (11 m, far past the 4 m range, still OUT_OF_RANGE). */
static volatile uint16_t g_width_us     = 0u;
static volatile uint32_t g_last_ping_us = 0u;

static void               ping_locked(ultrasonic_ping_t * p_out);
static ultrasonic_class_t echo_class(int32_t dist_mm);
static void               sort_readings(int32_t * p_values, uint32_t count);
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
 * @brief One ping, classified (FR7, assumption A9): VALID with a
 *        distance, OUT_OF_RANGE (the echo pulse is longer than US_MAX_MM
 *        or still high at the timeout), FAULT (no echo pulse at all, or
 *        ECHO already high before the trigger) or BUSY (another task held
 *        the sensor for US_SEM_TIMEOUT_MS, NFR9: no ping was sent).
 *        Keeps PING_GAP_US between pings.
 *
 * @param[out] p_out Class, distance (VALID only) and raw pulse width.
 */
void
ultrasonic_ping (ultrasonic_ping_t * p_out)
{
    p_out->result   = ULTRASONIC_BUSY;
    p_out->dist_mm  = ULTRASONIC_NO_ECHO;
    p_out->width_us = 0u;

    if (E_OK == tk_wai_sem(gh_sensor_sem, 1, US_SEM_TIMEOUT_MS))
    {
        ping_locked(p_out);
        (void) tk_sig_sem(gh_sensor_sem, 1);
    }
}

/*!
 * @brief Short name of a ping class, for test output and telemetry.
 *
 * @param[in] result Class.
 *
 * @return "VALID", "OUT_OF_RANGE", "FAULT" or "BUSY".
 */
char const *
ultrasonic_class_name (ultrasonic_class_t result)
{
    char const * p_name = "FAULT";

    if (ULTRASONIC_VALID == result)
    {
        p_name = "VALID";
    }
    else if (ULTRASONIC_OUT_OF_RANGE == result)
    {
        p_name = "OUT_OF_RANGE";
    }
    else if (ULTRASONIC_BUSY == result)
    {
        p_name = "BUSY";
    }
    else
    {
        /* FAULT. */
    }

    return (p_name);
}

/*!
 * @brief Distance from one ping (unchanged behaviour: anything but a
 *        VALID reading from US_MIN_MM to US_MAX_MM is "no echo").
 *
 * @return Distance in mm, or ULTRASONIC_NO_ECHO.
 */
int32_t
ultrasonic_measure_mm (void)
{
    ultrasonic_ping_t ping;
    int32_t           dist_mm = ULTRASONIC_NO_ECHO;

    ultrasonic_ping(&ping);

    if ((ULTRASONIC_VALID == ping.result) && (ping.dist_mm >= US_MIN_MM))
    {
        dist_mm = ping.dist_mm;
    }

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
 * @brief One ping with the sensor semaphore held: keep PING_GAP_US since
 *        the previous ping, check that ECHO is low, trigger and classify.
 *
 * @param[out] p_out Ping result; FAULT unless an echo pulse is seen.
 */
static void
ping_locked (ultrasonic_ping_t * p_out)
{
    UINT     pattern = 0u;
    ER       ercd    = E_OK;
    int32_t  dist_mm = ULTRASONIC_NO_ECHO;
    uint32_t since   = hal_time_us() - g_last_ping_us;

    /* Respect the minimum cycle time since the previous ping. */
    if (since < PING_GAP_US)
    {
        (void) tk_dly_tsk(((PING_GAP_US - since) / US_PER_MS) + 1u);
    }

    p_out->result = ULTRASONIC_FAULT;

    /* ECHO must be low before a trigger.  A line stuck high would
       otherwise time out like a no-target pulse and read as free space. */
    if (!hal_gpio_get(PIN_US_ECHO))
    {
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
               float32_t holds exactly; the product is at most a few metres
               in mm, so the truncation to int32_t cannot overflow. */
            dist_mm         = (int32_t) ((float32_t) g_width_us * MM_PER_US);
            p_out->width_us = g_width_us;
            p_out->result   = echo_class(dist_mm);
            p_out->dist_mm  = (ULTRASONIC_VALID == p_out->result)
                                  ? dist_mm
                                  : ULTRASONIC_NO_ECHO;
        }
        else if (0u != g_rise_us)
        {
            /* The pulse started but had not ended at the timeout. */
            p_out->result   = ULTRASONIC_OUT_OF_RANGE;
            p_out->width_us = hal_time_us() - g_rise_us;
        }
        else
        {
            /* No echo pulse at all: FAULT. */
        }
    }
}

/*!
 * @brief Class of a completed echo pulse by its distance.
 *
 * @param[in] dist_mm Distance from the pulse width.
 *
 * @return OUT_OF_RANGE beyond US_MAX_MM; FAULT below US_MIN_MM (closer than
 *         the HC-SR04 can measure: seen on the car when a loose wire let
 *         the trigger pulse couple onto ECHO); otherwise VALID.
 */
static ultrasonic_class_t
echo_class (int32_t dist_mm)
{
    ultrasonic_class_t result = ULTRASONIC_VALID;

    if (dist_mm > US_MAX_MM)
    {
        result = ULTRASONIC_OUT_OF_RANGE;
    }
    else if (dist_mm < US_MIN_MM)
    {
        result = ULTRASONIC_FAULT;
    }
    else
    {
        /* Within the sensor's range. */
    }

    return (result);
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
        /* Cast: saturated to the 16-bit field first. */
        g_width_us = ((t_us - g_rise_us) > WIDTH_MAX_US)
                         ? (uint16_t) WIDTH_MAX_US
                         : (uint16_t) (t_us - g_rise_us);
        g_rise_us  = 0u;
        (void) tk_set_flg(gh_echo_flag, FLG_ECHO);
    }
}

/*** end of file ***/
