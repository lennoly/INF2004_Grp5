/** @file test_data.h
 *
 * @brief Host tests for the minimum-bit data representation (Change
 *        Requirement 1): bitpack.c, packed.c and frames.c.
 */

#ifndef TEST_DATA_H
#define TEST_DATA_H

#include <stdbool.h>

void test_data(void (*p_check)(bool b_ok, char const * p_what));

#endif /* TEST_DATA_H */

/*** end of file ***/
