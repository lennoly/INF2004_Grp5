/** @file barcode.c
 *
 * @brief Buddy 3 - barcode framing task (see barcode.h).
 *
 * A frame starts at the first bar after a quiet zone and ends when the
 * sensor has been on white for BARCODE_QUIET_MS.
 *
 * Change Requirement 1: element widths arrive and are stored in 1 ms
 * sampler ticks, one byte each (the sampler runs at 1 kHz, so no
 * resolution is lost).  A bar longer than 254 ms (the car stopped or
 * crawling over it) is over-range: the frame is reset instead of wrapping
 * into a false short bar (Tribunal 1 decision).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "ir_sensor.h"
#include "timing.h"
#include "packed.h"
#include "barcode.h"

#define TASK_PRIORITY      (7)
#define TASK_STACK         (2048)
#define POLL_MS            (10u)
#define US_PER_MS          (1000u)
#define QUIET_US           (BARCODE_QUIET_MS * US_PER_MS)
#define MBF_MSGS           (4u)
#define MIN_FRAME_ELEMENTS (19u) /* data + gap + '*': first bars may be lost */
#define SHORT_REPORT_MIN   (3u)  /* debug: print shorter frames from here */
#define COUNT16_MASK       (0xFFFFu)

/* Decoder statistics, written only by the barcode task. */
typedef struct
{
    uint32_t frames  : 16;
    uint32_t decoded : 16;
    uint32_t errors  : 16;
    uint32_t last    : 16; /* packed_barcode_t */
} stats_t;

/* Shared with the vehicle and telemetry tasks: volatile (Rule 1.8.c). */
static volatile ID      gh_mbuf = 0;
static volatile stats_t g_stats;

/* Frame being collected; used only by barcode_task(). */
static uint8_t  g_widths[BARCODE_MAX_ELEMENTS]; /* sampler ticks */
static uint32_t g_count = 0u;

static void barcode_task(INT stacd, void * p_exinf);
static void finish_frame(void);
static void add_element(ir_sensor_element_t element);

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
    cmbf.bufsz  = MBF_MSGS * (sizeof(packed_barcode_t) + sizeof(UW));
    cmbf.maxmsz = sizeof(packed_barcode_t);
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
    packed_barcode_t message  = 0u;
    INT              msg_size = tk_rcv_mbf(gh_mbuf, &message, timeout_ms);

    p_event->cmd    = packed_barcode_cmd(message);
    p_event->symbol = packed_barcode_symbol(message);

    /* Cast: the message is 2 bytes, far below INT's range. */
    return ((INT) sizeof(message) == msg_size);
}

/*!
 * @brief Consistent copy of the decoder statistics.
 *
 * @param[out] p_out Statistics.
 */
void
barcode_get_stats (barcode_stats_t * p_out)
{
    stats_t          copy;
    packed_barcode_t last  = 0u;
    UINT             imask = 0u;

    DI(imask);
    copy.frames  = g_stats.frames;
    copy.decoded = g_stats.decoded;
    copy.errors  = g_stats.errors;
    copy.last    = g_stats.last;
    EI(imask);

    /* Cast: the field holds a 16-bit packed_barcode_t. */
    last              = (packed_barcode_t) copy.last;
    p_out->frames     = copy.frames;
    p_out->decoded    = copy.decoded;
    p_out->errors     = copy.errors;
    p_out->last.cmd   = packed_barcode_cmd(last);
    p_out->last.symbol = packed_barcode_symbol(last);
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
    ir_sensor_element_t element  = 0u;
    uint32_t            start_us = 0u;

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        start_us = timing_start();

        while (ir_sensor_barcode_pop(&element))
        {
            add_element(element);
        }

        /* The trailing quiet zone ends the frame. */
        if ((g_count > 0u) && (!ir_sensor_barcode_is_black())
            && (ir_sensor_barcode_idle_us() >= QUIET_US))
        {
            finish_frame();
        }

        timing_stop(TIMING_BARCODE, start_us);
        (void) tk_dly_tsk(POLL_MS);
    }
}

/*!
 * @brief Try to decode the collected frame and publish the result.
 *
 * Frames shorter than MIN_FRAME_ELEMENTS are noise or a line crossing;
 * from SHORT_REPORT_MIN elements they are printed for debugging.  The
 * 1-byte widths are widened on this task's stack for the decoder, which
 * works on ratios only.
 */
static void
finish_frame (void)
{
    uint32_t         widths[BARCODE_MAX_ELEMENTS];
    char             text[BARCODE_TEXT_MAX] = {0};
    packed_barcode_t message                = 0u;
    barcode_decode_nav_t cmd                = NAV_NONE;
    int32_t          len                    = 0;
    uint32_t         idx                    = 0u;

    if (g_count >= MIN_FRAME_ELEMENTS)
    {
        g_stats.frames = (g_stats.frames + 1u) & COUNT16_MASK;

        for (idx = 0u; idx < g_count; idx++)
        {
            widths[idx] = g_widths[idx];
        }

        len = barcode_decode_frame(widths, g_count, text, BARCODE_TEXT_MAX);

        /* Casts in the messages: tm_printf() takes the kernel's UB string
           type (the literals are ASCII) and the counts fit in INT. */
        if (len > 0)
        {
            cmd             = barcode_decode_to_command(text[0]);
            message         = packed_barcode_make(cmd, text[0]);
            g_stats.decoded = (g_stats.decoded + 1u) & COUNT16_MASK;
            g_stats.last    = message;
            (void) tk_snd_mbf(gh_mbuf, &message, (INT) sizeof(message),
                              TMO_POL);
            (void) tm_printf((UB const *) "[barcode] '%s' -> %s\n", text,
                             barcode_decode_command_name(cmd));
        }
        else
        {
            g_stats.errors = (g_stats.errors + 1u) & COUNT16_MASK;
            (void) tm_printf((UB const *) "[barcode] frame of %d elements "
                                          "rejected (%d):",
                             (INT) g_count, (INT) len);

            /* Debug (S3-U03): the widths, to see what went wrong. */
            for (idx = 0u; idx < g_count; idx++)
            {
                (void) tm_printf((UB const *) " %d", (INT) g_widths[idx]);
            }

            (void) tm_printf((UB const *) "\n");
        }
    }
    else if (g_count >= SHORT_REPORT_MIN)
    {
        /* Debug (S3-U03): what the sensor saw of a barcode it did not
           finish, widths in ms, bar first. */
        (void) tm_printf((UB const *) "[barcode] short frame: %d elements:",
                         (INT) g_count);

        for (idx = 0u; idx < g_count; idx++)
        {
            (void) tm_printf((UB const *) " %d", (INT) g_widths[idx]);
        }

        (void) tm_printf((UB const *) "\n");
    }
    else
    {
        /* One or two elements: a speck or an edge, not worth a line. */
    }

    g_count = 0u;
}

/*!
 * @brief Add one bar / space element to the frame being collected.
 *
 * @param[in] element Packed element from the IR sampler.
 */
static void
add_element (ir_sensor_element_t element)
{
    bool     b_bar = packed_ir_element_bar(element);
    uint32_t ticks = packed_ir_element_ticks(element);

    if ((!b_bar) && (ticks >= BARCODE_QUIET_MS))
    {
        finish_frame(); /* long white: the previous frame is over */
    }
    else if (b_bar && (PACKED_WIDTH_OVER == ticks))
    {
        g_count = 0u; /* over-range bar: reset, never wrap into a short one */
    }
    else if ((0u == g_count) && (!b_bar))
    {
        /* Frames always start with a bar. */
    }
    else if (g_count < BARCODE_MAX_ELEMENTS)
    {
        /* Cast: ticks is below PACKED_WIDTH_OVER (255), so it fits. */
        g_widths[g_count] = (uint8_t) ticks;
        g_count++;
    }
    else
    {
        g_count = 0u; /* runaway (e.g. driving along the line): reset */
    }
}

/*** end of file ***/
