/** @file module.h
 *
 * @brief One-sentence description of the module's purpose.
 *
 * Longer description: what the module owns (hardware, kernel objects,
 * data), which tasks or interrupts use it, and any assumptions
 * (Rule 2.2.g).  Reference external documents by name (Rule 2.2.e).
 *
 * Project template for BARR-C:2018 headers (Rule 4.4.a).  Rename
 * "module" to the lower-case module name (Rule 4.1.a) and delete what is
 * not needed.  Format with clang-format (app_program/.clang-format), then
 * run python3 tools/barr_layout.py and python3 tools/barr_check.py.
 */

#ifndef MODULE_H
#define MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"

/* Public constants: upper case, module prefix (Rule 6.1.f). */
#define MODULE_LIMIT (10u)

/* Public data types: lower case, module prefix, _t suffix (Rule 5.1). */
typedef enum
{
    MODULE_STATE_IDLE = 0,
    MODULE_STATE_BUSY
} module_state_t;

typedef struct
{
    module_state_t state;
    uint32_t       count;    /* aligned member names (Rule 3.2.b) */
    float32_t      value_mm; /* units as a suffix                 */
} module_status_t;

/* Public functions: module prefix (Rule 6.1.i), prototypes only; no
   variables are declared or defined in a header (Rule 4.2.c). */
int32_t module_init(void);
bool    module_read(module_status_t * p_out);

#endif /* MODULE_H */

/*** end of file ***/
