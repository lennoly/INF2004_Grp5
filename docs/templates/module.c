/** @file module.c
 *
 * @brief One-sentence description (see module.h).
 *
 * Project template for BARR-C:2018 source files (Rule 4.4.a).  The
 * sections below appear in the order Rule 4.3.b requires.
 */

/* 1. Include statements: standard headers, kernel, project headers, and
      always the module's own header (Rule 4.3.c).  A file that includes
      <tk/tkernel.h> must not include Pico SDK or lwIP headers. */
#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "car_config.h"
#include "module.h"

/* 2. Data types, constants and macros: no magic numbers (Rule 8.4.a). */
#define TASK_PRIORITY (9)
#define TASK_STACK    (2048)
#define PERIOD_MS     (20u)

/* 3. Static data: g prefix (Rule 7.1.j); volatile when shared with an ISR
      or another task (Rule 1.8.c); always initialised (Rule 7.2). */
static volatile module_status_t g_status;
static volatile ID              gh_task = 0;

/* 4. Private function prototypes (Rule 4.3.b); every private function is
      static (Rule 6.2.e). */
static void module_task(INT stacd, void * p_exinf);

/* 5. Public function bodies. */

/*!
 * @brief Create and start the module's task.
 *
 * @return E_OK or a kernel error code.
 */
int32_t
module_init (void)
{
    T_CTSK  ctsk = {0};
    int32_t ercd = E_OK;

    ctsk.tskatr = TA_HLNG | TA_RNG3;

    /* Cast: the kernel stores every task entry as the generic FP type and
       calls a TA_HLNG task as (INT stacd, void * exinf), matching
       module_task(). */
    ctsk.task    = (FP) module_task;
    ctsk.itskpri = TASK_PRIORITY;
    ctsk.stksz   = TASK_STACK;
    gh_task      = tk_cre_tsk(&ctsk);

    /* One exit point, at the bottom (Rule 6.2.c). */
    if (gh_task >= E_OK)
    {
        ercd = tk_sta_tsk(gh_task, 0);
    }
    else
    {
        ercd = gh_task;
    }

    return (ercd);
}

/*!
 * @brief Consistent copy of the module status.
 *
 * @param[out] p_out Status.
 *
 * @return true if the module is busy.
 */
bool
module_read (module_status_t * p_out)
{
    UINT imask = 0u;

    DI(imask);
    *p_out = g_status;
    EI(imask);

    return (MODULE_STATE_BUSY == p_out->state);
}

/* 6. Private function bodies. */

/*!
 * @brief Module task (name ends with _task, Rule 6.4.a).
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
module_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;

    /* Infinite loops are written for (;;) (Rule 8.4.c). */
    for (;;)
    {
        g_status.count++;
        (void) tk_dly_tsk(PERIOD_MS);
    }
}

/*** end of file ***/
