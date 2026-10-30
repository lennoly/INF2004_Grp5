/** @file barcode.h
 *
 * @brief Buddy 3 - barcode task: frames the IR edge stream, decodes it and
 *        delivers navigation commands to the vehicle controller.
 *
 * Change Requirement 1: the message to the vehicle task is a 2-byte
 * packed_barcode_t (command + Code 39 symbol index) instead of 20 bytes,
 * and the statistics are stored as 16-bit counters.  Readers get the
 * unpacked view below.
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>
#include <stdbool.h>
#include "barcode_decode.h"

#define BARCODE_TEXT_MAX (9u) /* decoder output buffer */

/* Unpacked view of one decoded barcode. */
typedef struct
{
    barcode_decode_nav_t cmd;
    char                 symbol; /* first data character, '?' if none */
} barcode_event_t;

typedef struct
{
    uint32_t        frames;  /* candidate frames seen (16-bit, wraps) */
    uint32_t        decoded; /* successful decodes    (16-bit, wraps) */
    uint32_t        errors;  /*                       (16-bit, wraps) */
    barcode_event_t last;
} barcode_stats_t;

int32_t barcode_init(void);
bool    barcode_get_event(barcode_event_t * p_event, int32_t timeout_ms);
void    barcode_get_stats(barcode_stats_t * p_out);

#endif /* BARCODE_H */

/*** end of file ***/
