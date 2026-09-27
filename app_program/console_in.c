/** @file console_in.c
 *
 * @brief Buddy 1 - serial console input (see console_in.h).
 */

#include <stdint.h>
#include <stdbool.h>
#include "hal.h"
#include "console_in.h"

#define USB_RX_SIZE (128u) /* power of two */

/* Shared between the USB service task (producer) and the telemetry task
   (consumer): volatile (Rule 1.8.c). */
static volatile uint8_t  g_usb_rx[USB_RX_SIZE];
static volatile uint32_t g_usb_head = 0u; /* written by the USB task */
static volatile uint32_t g_usb_tail = 0u; /* written by the consumer */
static volatile bool     gb_ready   = false;

/*!
 * @brief Start accepting input; bytes received before this are dropped.
 */
void
console_in_init (void)
{
    g_usb_tail = g_usb_head;
    hal_memory_barrier();
    gb_ready = true;
}

/*!
 * @brief Take the next received byte: USB first, then UART0.  Never blocks.
 *
 * @param[out] p_byte Destination for the byte.
 *
 * @return true if *p_byte holds a new byte.
 */
bool
console_in_getc (uint8_t * p_byte)
{
    bool b_got = false;

    if (g_usb_tail != g_usb_head)
    {
        hal_memory_barrier();
        *p_byte = g_usb_rx[g_usb_tail];
        hal_memory_barrier();
        g_usb_tail = (g_usb_tail + 1u) & (USB_RX_SIZE - 1u);
        b_got      = true;
    }
    else
    {
        b_got = hal_uart0_getc(p_byte);
    }

    return (b_got);
}

/*!
 * @brief USB receive hook, called by the template's USB service task for
 *        every byte from the host.  Drops the byte when the ring is full.
 *
 * @param[in] rx_byte Received byte.
 */
void
tm_usb_rx_byte (uint8_t rx_byte)
{
    uint32_t next = (g_usb_head + 1u) & (USB_RX_SIZE - 1u);

    if (gb_ready && (next != g_usb_tail))
    {
        g_usb_rx[g_usb_head] = rx_byte;
        hal_memory_barrier();
        g_usb_head = next;
    }
}

/*** end of file ***/
