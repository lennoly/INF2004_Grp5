/** @file hal_stub.c
 *
 * @brief Host-test replacement for the one HAL function used by the pure
 *        modules under test (mqtt_bridge.c).
 */

#include "hal_stub.h"

/*!
 * @brief Compiler-only barrier: the host tests are single-threaded.
 */
void
hal_memory_barrier (void)
{
    /* WARNING: inline assembly (Rule 1.1.c), an empty statement that only
       stops the compiler from reordering memory accesses across it. */
    __asm__ volatile("" ::: "memory");
}

/*** end of file ***/
