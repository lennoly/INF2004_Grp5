/** @file fault_stub.c
 *
 * @brief Host stand-in for fault.c (no kernel event flag on the PC): the
 *        same API over a plain variable, so modules that raise faults link
 *        into the host tests.
 */

#include <stdint.h>
#include <stdbool.h>
#include "fault.h"

static uint32_t g_mask = 0u;
static uint8_t  g_count[FAULT_COUNT];

int32_t
fault_init (void)
{
    g_mask = 0u;

    return (0);
}

void
fault_raise (fault_id_t fault)
{
    uint32_t bit = 1u << (uint32_t) fault;

    if (0u == (g_mask & bit))
    {
        g_count[(uint32_t) fault]++;
    }

    g_mask |= bit;
}

void
fault_clear (fault_id_t fault)
{
    g_mask &= ~(1u << (uint32_t) fault);
}

void
fault_alive (fault_id_t fault)
{
    (void) fault;
}

uint32_t
fault_mask (void)
{
    return (g_mask);
}

uint8_t
fault_count (fault_id_t fault)
{
    return (g_count[(uint32_t) fault]);
}

uint32_t
fault_age_ms (fault_id_t fault)
{
    (void) fault;

    return (0u);
}

/*** end of file ***/
