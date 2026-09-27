/** @file app_tests.h
 *
 * @brief Standalone subsystem test programs (one per buddy), selected at
 *        build time with make APP_MODE=TEST_xxx.  Each runs in its own
 *        task and prints CSV-style lines for the evaluation reports.
 */

#ifndef APP_TESTS_H
#define APP_TESTS_H

#include <stdint.h>

int32_t app_tests_start(void);

#endif /* APP_TESTS_H */

/*** end of file ***/
