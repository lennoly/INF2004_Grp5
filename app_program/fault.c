/** @file fault.c
 *
 * @brief Shared fault bitmask in a uT-Kernel event flag (see fault.h).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "fault.h"

#define COUNT_MAX (0xFFu)   /* saturating 8-bit count       */
#define TIME_MASK (0xFFFFu) /* last update: low 16 bits, ms */

/* Event flag handle: written once by fault_init() before any task runs. */
static volatile ID gh_flag = 0;

/* Per-fault counters and update times.  Each element has one writer (the
   fault's owner) and is stored with one byte / half-word store, which is
   atomic on the Cortex-M0+; readers never write them. */
static volatile uint8_t  g_count[FAULT_COUNT];
static volatile uint16_t g_alive_ms[FAULT_COUNT];

static uint32_t bit_of(fault_id_t fault);
static uint16_t now_ms16(void);

/*!
 * @brief Create the fault event flag (all bits clear).  Call once, before
 *        the tasks that raise faults start.
 *
 * @return E_OK or a tk_cre_flg() error.
 */
int32_t
fault_init (void)
{
    T_CFLG cflg = {0};

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0u;
    gh_flag      = tk_cre_flg(&cflg);

    return ((gh_flag >= E_OK) ? E_OK : gh_flag);
}

/*!
 * @brief Set a fault bit (owner task only).  Counts each clear-to-set
 *        change.
 *
 * @param[in] fault Fault to raise.
 */
void
fault_raise (fault_id_t fault)
{
    uint32_t bit = bit_of(fault);

    /* Only the owner writes this bit, so reading it first cannot race. */
    if ((0u != bit) && (0u == (fault_mask() & bit)))
    {
        /* Cast: fault is a valid index, checked by bit_of(). */
        if (g_count[(uint32_t) fault] < COUNT_MAX)
        {
            g_count[(uint32_t) fault]++;
        }

        (void) tk_set_flg(gh_flag, bit);
    }

    fault_alive(fault);
}

/*!
 * @brief Clear a fault bit (owner task only).
 *
 * @param[in] fault Fault that has gone away.
 */
void
fault_clear (fault_id_t fault)
{
    uint32_t bit = bit_of(fault);

    if (0u != bit)
    {
        /* tk_clr_flg() keeps the bits that are 1 in the pattern. */
        (void) tk_clr_flg(gh_flag, ~bit);
    }

    fault_alive(fault);
}

/*!
 * @brief Record that the owner of a fault ran its check just now, so a
 *        stalled owner can be told from a healthy one.
 *
 * @param[in] fault Fault whose owner is alive.
 */
void
fault_alive (fault_id_t fault)
{
    if (0u != bit_of(fault))
    {
        /* Cast: fault is a valid index, checked by bit_of(). */
        g_alive_ms[(uint32_t) fault] = now_ms16();
    }
}

/*!
 * @brief Every fault bit in one read.
 *
 * @return Mask of active faults (bit n = fault_id_t n); 0 before init.
 */
uint32_t
fault_mask (void)
{
    T_RFLG   rflg = {0};
    uint32_t mask = 0u;

    if ((gh_flag > 0) && (E_OK == tk_ref_flg(gh_flag, &rflg)))
    {
        mask = rflg.flgptn & ((1u << FAULT_BITS) - 1u);
    }

    return (mask);
}

/*!
 * @brief How often a fault was raised (saturates at 255).
 *
 * @param[in] fault Fault.
 *
 * @return Count, 0..255.
 */
uint8_t
fault_count (fault_id_t fault)
{
    uint8_t count = 0u;

    if (0u != bit_of(fault))
    {
        /* Cast: fault is a valid index, checked by bit_of(). */
        count = g_count[(uint32_t) fault];
    }

    return (count);
}

/*!
 * @brief Time since the fault's owner last updated it.
 *
 * @param[in] fault Fault.
 *
 * @return Age in ms, 0..65535 (wrap-safe 16-bit difference).
 */
uint32_t
fault_age_ms (fault_id_t fault)
{
    uint32_t age = 0u;

    if (0u != bit_of(fault))
    {
        /* Cast: fault is a valid index, checked by bit_of(). */
        age = ((uint32_t) now_ms16() - g_alive_ms[(uint32_t) fault])
              & TIME_MASK;
    }

    return (age);
}

/*!
 * @brief The mask bit of a fault.
 *
 * @param[in] fault Fault.
 *
 * @return 1 << fault, or 0 for an invalid value.
 */
static uint32_t
bit_of (fault_id_t fault)
{
    /* Cast: enumeration constants are non-negative, so the conversion to
       an unsigned index keeps their value; the range is checked here. */
    uint32_t index = (uint32_t) fault;

    return ((index < (uint32_t) FAULT_COUNT) ? (1u << index) : 0u);
}

/*!
 * @brief Low 16 bits of the kernel's millisecond clock.
 *
 * @return Time in ms, modulo 65536.
 */
static uint16_t
now_ms16 (void)
{
    SYSTIM tim = {0, 0u};

    (void) tk_get_tim(&tim);

    /* Cast: the mask keeps the low 16 bits. */
    return ((uint16_t) (tim.lo & TIME_MASK));
}

/*** end of file ***/
