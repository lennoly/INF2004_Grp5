/** @file subsystem4_test.h
 *
 * @brief Buddy 4 test program (make APP_MODE=TEST_IMU), run by
 *        app_tests.c each time START is pressed.
 */

#ifndef SUBSYSTEM4_TEST_H
#define SUBSYSTEM4_TEST_H

#include <stdint.h>

int32_t subsystem4_test_init(void);
void    subsystem4_test_imu(void);

#endif /* SUBSYSTEM4_TEST_H */

/*** end of file ***/
