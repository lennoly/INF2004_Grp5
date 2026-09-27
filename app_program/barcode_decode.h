/** @file barcode_decode.h
 *
 * @brief Buddy 3 - Code 39 decoder (pure C, host-testable).
 *
 * Input: element widths starting with a BAR and alternating bar / space,
 * i.e. k characters x 9 elements + (k - 1) inter-character gaps = 10k - 1.
 * Each Code 39 character has exactly 3 wide elements, so the 3 widest of
 * each group of 9 are "wide".  This is independent of speed, as long as
 * the speed is roughly constant over one character (about 40 mm).  The
 * code is tried forwards, then reversed (the car may read it backwards).
 * Reference: ISO/IEC 16388 (Code 39 bar code symbology specification).
 */

#ifndef BARCODE_DECODE_H
#define BARCODE_DECODE_H

#include <stdint.h>

#define BARCODE_ELEMENTS_PER_CHAR (9u)
#define BARCODE_MAX_ELEMENTS      (79u) /* up to 8 characters */

typedef enum
{
    NAV_NONE = 0,
    NAV_LEFT,
    NAV_RIGHT,
    NAV_STRAIGHT,
    NAV_UTURN
} barcode_decode_nav_t;

#define BARCODE_ERR_LENGTH (-1)
#define BARCODE_ERR_RATIO  (-2)
#define BARCODE_ERR_SYMBOL (-3)
#define BARCODE_ERR_FRAME  (-4)

int32_t barcode_decode_frame(uint32_t const * p_widths, uint32_t count,
                             char * p_out, uint32_t out_size);
barcode_decode_nav_t barcode_decode_to_command(char symbol);
char const *         barcode_decode_command_name(barcode_decode_nav_t command);

#endif /* BARCODE_DECODE_H */

/*** end of file ***/
