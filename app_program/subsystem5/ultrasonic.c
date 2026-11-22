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
#include <tm/tmonitor.h>
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
/* Shorter ECHO pulses are noise (motor or TRIG coupling), not echoes: the
   nearest real echo, US_MIN_MM, is 117 us.  Ignored, so the real pulse
   after one is still timed. */
#define GLITCH_US     (50u)
/* With nothing in range this module holds ECHO high well past the next
   ping (11 Nov: "ECHO stuck high" FAULTs stopped the car at starts and on
   bends; the first sensor's pulse was 46 ms).  ECHO still high from a pulse
   that was seen to start is that pulse, OUT_OF_RANGE, for this long; only
   after it, or with no pulse start seen (wire off: pulled up), a FAULT. */
#define ECHO_LONG_US  (1000000u)

/* Kernel objects and echo capture, shared with the echo interrupt and the
   calling tasks: volatile (Rule 1.8.c). */
static volatile ID       gh_echo_flag   = 0;
static volatile ID       gh_sensor_sem  = 0;
static volatile uint32_t g_rise_us      = 0u;
/* Change Requirement 1: the echo width in 16 bits, saturated at 65 535 us
   (11 m, far past the 4 m range, still OUT_OF_RANGE). */
static volatile uint16_t g_width_us     = 0u;
static volatile uint32_t g_last_ping_us = 0u;

/* Last ping was a FAULT (only under the sensor semaphore). */
static bool gb_was_fault = false;

static void               ping_locked(ultrasonic_ping_t * p_out);
static ultrasonic_class_t echo_class(int32_t dist_mm);
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
    /* Pulled up: an unplugged ECHO reads stuck high (FAULT ping) instead of
       floating and picking up motor noise as echoes (S5-U12, 30 Oct). */
    hal_gpio_init_in(PIN_US_ECHO, true);

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
    uint32_t rise_us = g_rise_us; /* one read: the ISR clears it */
    char const * p_why = "";

    /* Respect the minimum cycle time since the previous ping. */
    if (since < PING_GAP_US)
    {
        (void) tk_dly_tsk(((PING_GAP_US - since) / US_PER_MS) + 1u);
    }

    p_out->result = ULTRASONIC_FAULT;
    p_why         = "ECHO stuck high";

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
            p_why           = "pulse too short";
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
            p_why = "no ECHO pulse (sensor did not answer)";
        }
    }

    else if ((0u != rise_us) && ((hal_time_us() - rise_us) < ECHO_LONG_US))
    {
        /* The last ping's pulse has not ended: nothing in range. */
        p_out->result   = ULTRASONIC_OUT_OF_RANGE;
        p_out->width_us = WIDTH_MAX_US;
    }
    else
    {
        /* ECHO high with no pulse of ours: FAULT, "ECHO stuck high". */
    }

    /* The first FAULT after a good ping, with its cause: tells a sensor
       that stopped answering (supply, TRIG wire) from an ECHO wire.
       Cast: tm_printf() takes the kernel's UB string type; ASCII. */
    if ((ULTRASONIC_FAULT == p_out->result) && (!gb_was_fault))
    {
        (void) tm_printf((UB const *) "[sonar] FAULT ping: %s\n", p_why);
    }

    gb_was_fault = (ULTRASONIC_FAULT == p_out->result);
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
        if ((t_us - g_rise_us) >= GLITCH_US)
        {
            /* Cast: saturated to the 16-bit field first. */
            g_width_us = ((t_us - g_rise_us) > WIDTH_MAX_US)
                             ? (uint16_t) WIDTH_MAX_US
                             : (uint16_t) (t_us - g_rise_us);
            (void) tk_set_flg(gh_echo_flag, FLG_ECHO);
        }

        g_rise_us = 0u;
    }
}

/*** end of file ***/
