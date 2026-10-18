/** @file timing_stub.c
 *
 * @brief Host-test replacement for the timing calls made by the code
 *        under test (see timing_stub.h).
 */

#include <stdint.h>
#include "timing_stub.h"

/*!
 * @brief No timer in the host tests.
 *
 * @return 0.
 */
uint32_t
timing_start (void)
{
    return (0u);
}

/*!
 * @brief Nothing is recorded in the host tests.
 *
 * @param[in] section  Section (unused).
 * @param[in] start_us Start time (unused).
 */
void
timing_stop (timing_id_t section, uint32_t start_us)
{
    (void) section;
    (void) start_us;
}

/*** end of file ***/
