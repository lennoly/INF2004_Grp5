/** @file barcode.c
 *
 * @brief Buddy 3 - barcode framing task (see barcode.h).
 *
 * Course frames contain exactly one symbol (*A* through *Z*). Decode at
 * element 29, then require white quiet before rearming.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "car_config.h"
#include "ir_sensor.h"
#include "timing.h"
#include "barcode.h"

#define TASK_PRIORITY      (7)
#define TASK_STACK         (2048)
#define POLL_MS            (10u)
#define US_PER_MS          (1000u)
#define QUIET_US           (BARCODE_QUIET_MS * US_PER_MS)
#define MBF_MSGS           (4u)
#define WINDOW_STEP        (2u)
#define MIN_FRAME_ELEMENTS (29u) /* 3 characters x 10 - 1 */
#define EVENT_BYTES        (10u) /* symbol, command, time and session */
#define EVENT_STORAGE      (12u) /* kernel rounds payload to a UW boundary */

/* Shared with the vehicle and telemetry tasks: volatile (Rule 1.8.c). */
static volatile ID              gh_mbuf = 0;
static volatile barcode_stats_t g_stats;
static volatile bool            gb_enabled = true;
static ID                       gh_task    = 0;
static volatile uint32_t        g_requested_session = 0u;
static uint32_t                 g_active_session = 0u;

/* Frame being collected; used only by barcode_task(). */
static uint32_t g_widths[MIN_FRAME_ELEMENTS];
static uint32_t g_count         = 0u;
static uint32_t g_seen_overruns = 0u;
static bool     gb_armed        = false;

static int32_t create_task(void);
static void    barcode_task(INT stacd, void * p_exinf);
static void    finish_frame(void);
static void    add_element(ir_sensor_element_t const * p_element);

/*!
 * @brief Create the event message buffer and start the barcode task.
 *
 * @return E_OK, E_LIMIT if a kernel object could not be created, or a
 *         tk_sta_tsk() error.
 */
int32_t
barcode_init (void)
{
    T_CMBF  cmbf = {0};
    int32_t ercd = E_OK;

    if (gh_task <= 0)
    {
        cmbf.mbfatr = TA_TFIFO;
        cmbf.bufsz  = MBF_MSGS * (EVENT_STORAGE + sizeof(UW));
        cmbf.maxmsz = EVENT_BYTES;
        gh_mbuf     = tk_cre_mbf(&cmbf);
        ercd        = (gh_mbuf > 0) ? create_task() : gh_mbuf;
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
    uint8_t packed[EVENT_BYTES] = {0};
    INT msg_size = 0;
    int32_t wait_ms = timeout_ms;
    uint32_t session = 0u;
    bool b_found = false;

    UINT imask = 0u;

    do
    {
        msg_size = tk_rcv_mbf(gh_mbuf, packed, wait_ms);
        wait_ms = TMO_POL;
        /* Casts widen each stored session byte before shifting. */
        session = (uint32_t) packed[6] | ((uint32_t) packed[7] << 8u)
                  | ((uint32_t) packed[8] << 16u)
                  | ((uint32_t) packed[9] << 24u);
        DI(imask);
        b_found = (EVENT_BYTES == msg_size) && gb_enabled
                  && (session == g_requested_session);

        if (b_found)
        {
            *p_event = (barcode_event_t){NAV_NONE, {0}, 0u};
            /* Casts retain checked ASCII, command bits and time bytes. */
            p_event->text[0] = (char) (packed[0] & 0x7fu);
            p_event->cmd = (barcode_decode_nav_t) (packed[1] & 7u);
            p_event->time_ms = (uint32_t) packed[2]
                              | ((uint32_t) packed[3] << 8u)
                              | ((uint32_t) packed[4] << 16u)
                              | ((uint32_t) packed[5] << 24u);
        }

        EI(imask);
    } while ((EVENT_BYTES == msg_size) && (!b_found));

    return (b_found);
}

/*!
 * @brief Pause capture during manual calibration, or start a fresh session.
 *
 * @param[in] b_enabled Whether to accept new frames.
 */
void
barcode_set_enabled (bool b_enabled)
{
    UINT imask = 0u;

    DI(imask);
    g_requested_session++;
    gb_enabled = b_enabled;
    EI(imask);
}

/*!
 * @brief Consume at most one ring of samples, rejecting any damaged frame.
 *
 * Called by barcode_task, or synchronously by the host tests.
 */
void
barcode_poll (void)
{
    ir_sensor_element_t element  = {false, 0u};
    uint32_t            overruns = ir_sensor_barcode_overruns();
    uint32_t            budget   = 64u;

    if (g_active_session != g_requested_session)
    {
        g_active_session = g_requested_session;
        g_count = 0u;
        gb_armed = false;
        ir_sensor_barcode_reset();
        overruns = ir_sensor_barcode_overruns();
        g_seen_overruns = overruns;
    }

    if (gb_enabled)
    {
        if (overruns != g_seen_overruns)
        {
            g_stats.overruns += overruns - g_seen_overruns;
            g_seen_overruns = overruns;
            g_count         = 0u;
            gb_armed        = false;
            ir_sensor_barcode_reset();
        }

        while ((budget > 0u) && (g_active_session == g_requested_session)
               && (ir_sensor_barcode_pop(&element)))
        {
            budget--;
            add_element(&element);
        }

        if (ir_sensor_barcode_quiet(QUIET_US))
        {
            g_stats.errors += (g_count > 0u) ? 1u : 0u;
            g_count  = 0u;
            gb_armed = true;
        }
    }
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
 * @brief Create and start the sole decoder task, cleaning up on failure.
 *
 * @return Kernel result.
 */
static int32_t
create_task (void)
{
    T_CTSK  ctsk = {0};
    int32_t ercd = E_LIMIT;

    ctsk.tskatr = TA_HLNG | TA_RNG3;
    /* Cast: the kernel's generic task pointer has this entry signature. */
    ctsk.task    = (FP) barcode_task;
    ctsk.itskpri = TASK_PRIORITY;
    ctsk.stksz   = TASK_STACK;
    gh_task      = tk_cre_tsk(&ctsk);
    ercd         = (gh_task > 0) ? tk_sta_tsk(gh_task, 0) : gh_task;

    if (ercd < E_OK)
    {
        if (gh_task > 0)
        {
            (void) tk_del_tsk(gh_task);
        }

        (void) tk_del_mbf(gh_mbuf);
        gh_task = 0;
        gh_mbuf = 0;
    }

    return (ercd);
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
    uint32_t start_us = 0u;

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        start_us = timing_start();

        barcode_poll();

        timing_stop(TIMING_BARCODE, start_us);
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
    barcode_event_t event               = {NAV_NONE, {0}, 0u};
    int32_t         len                 = 0;
    SYSTIM          now                 = {0, 0u};
    uint8_t         packed[EVENT_BYTES] = {0};

    g_stats.frames++;
    len = barcode_decode_frame(g_widths, g_count, event.text, BARCODE_TEXT_MAX);

    if ((1 == len) && (g_seen_overruns == ir_sensor_barcode_overruns()))
    {
        (void) tk_get_tim(&now);
        event.cmd     = barcode_decode_to_command(event.text[0]);
        event.time_ms = now.lo;
        g_stats.decoded++;
        g_stats.last = event;
        /* Casts retain ASCII, command bits and each timestamp byte. */
        packed[0] = (uint8_t) event.text[0] & 0x7fu;
        packed[1] = (uint8_t) event.cmd & 7u;
        packed[2] = (uint8_t) (event.time_ms & 0xffu);
        packed[3] = (uint8_t) ((event.time_ms >> 8u) & 0xffu);
        packed[4] = (uint8_t) ((event.time_ms >> 16u) & 0xffu);
        packed[5] = (uint8_t) ((event.time_ms >> 24u) & 0xffu);
        packed[6] = (uint8_t) (g_active_session & 0xffu);
        packed[7] = (uint8_t) ((g_active_session >> 8u) & 0xffu);
        packed[8] = (uint8_t) ((g_active_session >> 16u) & 0xffu);
        packed[9] = (uint8_t) ((g_active_session >> 24u) & 0xffu);

        if (tk_snd_mbf(gh_mbuf, packed, EVENT_BYTES, TMO_POL) < E_OK)
        {
            g_stats.dropped++;
        }
    }
    else
    {
        g_stats.errors++;
    }

    if (1 == len)
    {
        g_count = 0u;
        gb_armed = false;
    }
    else
    {
        /* Retry the next bar-aligned window after a leading noise pair. */
        for (uint32_t idx = 0u; idx < (MIN_FRAME_ELEMENTS - WINDOW_STEP); idx++)
        {
            g_widths[idx] = g_widths[idx + WINDOW_STEP];
        }

        g_count = MIN_FRAME_ELEMENTS - WINDOW_STEP;
    }
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
        if (g_count > 0u)
        {
            g_stats.errors++;
        }

        g_count  = 0u;
        gb_armed = true;
    }
    else if (!gb_armed)
    {
        /* Ignore the tail of a completed frame until the next quiet zone. */
    }
    else if ((0u == g_count) && (!p_element->b_bar))
    {
        /* Frames always start with a bar. */
    }
    else if (p_element->b_bar != (0u == (g_count & 1u)))
    {
        g_stats.errors++;
        g_count  = 0u;
        gb_armed = false;
    }
    else if (g_count < MIN_FRAME_ELEMENTS)
    {
        g_widths[g_count] = p_element->width_us;
        g_count++;

        if (MIN_FRAME_ELEMENTS == g_count)
        {
            finish_frame();
        }
    }
    else
    {
        g_count = 0u; /* runaway (e.g. driving along the line): reset */
    }
}

/*** end of file ***/
