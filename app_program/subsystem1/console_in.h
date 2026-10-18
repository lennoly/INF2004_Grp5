/** @file console_in.h
 *
 * @brief Buddy 1 - non-blocking serial console input (USB-CDC and UART0).
 *
 * USB: the template's USB service task hands every received byte to
 * tm_usb_rx_byte() (weak hook added to usb_tinyusb_glue.c); this module
 * keeps them in a lock-free single-producer / single-consumer ring.
 * UART: bytes are read straight from the UART0 FIFO by hal_uart0_getc().
 * Plain C (no kernel headers) so the USB hook can live here.
 *
 * NOTE: tm_usb_rx_byte() is named by the template's hook, so it cannot
 * carry this module's prefix (documented deviation from Rule 6.1.i).
 */

#ifndef CONSOLE_IN_H
#define CONSOLE_IN_H

#include <stdint.h>
#include <stdbool.h>

void console_in_init(void);
bool console_in_getc(uint8_t * p_byte);
void tm_usb_rx_byte(uint8_t rx_byte);

#endif /* CONSOLE_IN_H */

/*** end of file ***/
