/** @file barcode.c
 *
 * @brief Buddy 3 - barcode framing task (see barcode.h).
 *
 * A frame starts at the first bar after a quiet zone and ends when the
 * sensor has been on white for BARCODE_QUIET_MS.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "ir_sensor.h"
#include "barcode.h"

#define TASK_PRIORITY      (7)
#define TASK_STACK         (2048)
#define POLL_MS            (10u)
#define US_PER_MS          (1000u)
#define QUIET_US           (BARCODE_QUIET_MS * US_PER_MS)
#define MBF_MSGS           (4u)
#define MIN_FRAME_ELEMENTS (29u) /* 3 characters x 10 - 1 */

/* Shared with the vehicle and telemetry tasks: volatile (Rule 1.8.c). */
static volatile ID              gh_mbuf = 0;
static volatile barcode_stats_t g_stats;

/* Frame being collected; used only by barcode_task(). */
static uint32_t g_widths[BARCODE_MAX_ELEMENTS];
static uint32_t g_count = 0u;

static void barcode_task(INT stacd, void * p_exinf);
static void finish_frame(void);
static void add_element(ir_sensor_element_t const * p_element);

/*!
 * @brief Create the event message buffer and start the barcode task.
 *
 * @return E_OK, E_LIMIT if a kernel object could not be created, or a
 *         tk_sta_tsk() error.
 */
int32_t
barcode_init (void)
{
    T_CMBF cmbf   = {0};
    T_CTSK ctsk   = {0};
    ID     h_task = 0;
    ER     ercd   = E_LIMIT;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = MBF_MSGS * (sizeof(barcode_event_t) + sizeof(UW));
    cmbf.maxmsz = sizeof(barcode_event_t);
    gh_mbuf     = tk_cre_mbf(&cmbf);

    ctsk.tskatr = TA_HLNG | TA_RNG3;

    /* Cast: the kernel stores every task entry as the generic FP type and
       calls a TA_HLNG task as (INT stacd, void * exinf), matching
       barcode_task(). */
    ctsk.task    = (FP) barcode_task;
    ctsk.itskpri = TASK_PRIORITY;
    ctsk.stksz   = TASK_STACK;
    h_task       = tk_cre_tsk(&ctsk);

    if ((gh_mbuf >= E_OK) && (h_task >= E_OK))
    {
        ercd = tk_sta_tsk(h_task, 0);
    }

    return (ercd);
}

/*!
 * @brief Receive the next decoded command.
 *
 * @param[out] p_event    Decoded barcode.
 * @param[in]  timeout_ms Wait time; TMO_POL (0) to poll, TMO_FEVR forever.
 *
 * @return true if an event was received.
 */
bool
barcode_get_event (barcode_event_t * p_event, int32_t timeout_ms)
{
    INT msg_size = tk_rcv_mbf(gh_mbuf, p_event, timeout_ms);

    /* Cast: the event is a few dozen bytes, far below INT's range. */
    return ((INT) sizeof(barcode_event_t) == msg_size);
}

/*!
 * @brief Consistent copy of the decoder statistics.
 *
 * @param[out] p_out Statistics.
 */
void
barcode_get_stats (barcode_stats_t * p_out)
{
    UINT imask = 0u;

    DI(imask);
    *p_out = g_stats;
    EI(imask);
}

/*!
 * @brief Barcode task: frames the bar / space stream from the IR sampler.
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
barcode_task (INT stacd, void * p_exinf)
{
    ir_sensor_element_t element = {false, 0u};

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        while (ir_sensor_barcode_pop(&element))
        {
            add_element(&element);
        }

        /* The trailing quiet zone ends the frame. */
        if ((g_count > 0u) && (!ir_sensor_barcode_is_black())
            && (ir_sensor_barcode_idle_us() >= QUIET_US))
        {
            finish_frame();
        }

        (void) tk_dly_tsk(POLL_MS);
    }
}

/*!
 * @brief Try to decode the collected frame and publish the result.
 *
 * Frames shorter than three characters are noise or a line crossing and
 * are dropped silently.
 */
static void
finish_frame (void)
{
    barcode_event_t event = {NAV_NONE, {0}, 0u};
    int32_t         len   = 0;
    SYSTIM          now   = {0, 0u};

    if (g_count >= MIN_FRAME_ELEMENTS)
    {
        g_stats.frames++;
        len = barcode_decode_frame(g_widths, g_count, event.text,
                                   BARCODE_TEXT_MAX);

        /* Casts in the messages: tm_printf() takes the kernel's UB string
           type (the literals are ASCII) and the counts fit in INT. */
        if (len > 0)
        {
            (void) tk_get_tim(&now);
            event.cmd     = barcode_decode_to_command(event.text[0]);
            event.time_ms = now.lo;
            g_stats.decoded++;
            g_stats.last = event;
            (void) tk_snd_mbf(gh_mbuf, &event, (INT) sizeof(event), TMO_POL);
            (void) tm_printf((UB const *) "[barcode] '%s' -> %s\n", event.text,
                             barcode_decode_command_name(event.cmd));
        }
        else
        {
            g_stats.errors++;
            (void) tm_printf((UB const *) "[barcode] frame of %d elements "
                                          "rejected (%d)\n",
                             (INT) g_count, (INT) len);
        }
    }

    g_count = 0u;
}

/*!
 * @brief Add one bar / space element to the frame being collected.
 *
 * @param[in] p_element Element from the IR sampler.
 */
static void
add_element (ir_sensor_element_t const * p_element)
{
    if ((!p_element->b_bar) && (p_element->width_us >= QUIET_US))
    {
        finish_frame(); /* long white: the previous frame is over */
    }
    else if ((0u == g_count) && (!p_element->b_bar))
    {
        /* Frames always start with a bar. */
    }
    else if (g_count < BARCODE_MAX_ELEMENTS)
    {
        g_widths[g_count] = p_element->width_us;
        g_count++;
    }
    else
    {
        g_count = 0u; /* runaway (e.g. driving along the line): reset */
    }
}

/*** end of file ***/
