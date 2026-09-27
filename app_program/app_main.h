/** @file app_main.h
 *
 * @brief PicoCar entry point, called by micro T-Kernel after start-up.
 *
 * NOTE: the name usermain() and the INT return type are fixed by the
 * kernel (kernel/knlinc/kernel.h), so the function cannot carry this
 * module's prefix (documented deviation from Rule 6.1.i).
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <tk/tkernel.h>

INT usermain(void);

#endif /* APP_MAIN_H */

/*** end of file ***/
