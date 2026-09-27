/** @file barcode.h
 *
 * @brief Buddy 3 - barcode task: frames the IR edge stream, decodes it and
 *        delivers navigation commands to the vehicle controller.
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>
#include <stdbool.h>
#include "barcode_decode.h"

#define BARCODE_TEXT_MAX (9u)

typedef struct
{
    barcode_decode_nav_t cmd;
    char                 text[BARCODE_TEXT_MAX];
    uint32_t             time_ms;
} barcode_event_t;

typedef struct
{
    uint32_t        frames;  /* candidate frames seen */
    uint32_t        decoded; /* successful decodes    */
    uint32_t        errors;
    barcode_event_t last;
} barcode_stats_t;

int32_t barcode_init(void);
bool    barcode_get_event(barcode_event_t * p_event, int32_t timeout_ms);
void    barcode_get_stats(barcode_stats_t * p_out);

#endif /* BARCODE_H */

/*** end of file ***/
