/** @file bitpack.h
 *
 * @brief Minimum-bit data representation helpers (Change Requirement 1).
 *
 * Two tools, one rule: every field is stored in exactly the bits its valid
 * range needs.
 *
 * - Range helpers clamp a value into an n-bit field and convert signed
 *   values to and from two's complement, for the bit-field structs in
 *   packed.h (data kept inside the firmware).
 * - A bit writer / reader with explicit masks and shifts, for data that
 *   leaves the car (MQTT frames, frames.h).  C leaves bit-field order and
 *   padding to the compiler, so wire formats never use bit-fields.
 *
 * Bit order on the wire: least significant bit first, bytes in increasing
 * address order (bit n of the stream is bit n % 8 of byte n / 8).
 */

#ifndef BITPACK_H
#define BITPACK_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"

#define BITPACK_MAX_BITS (32u) /* widest single field */

/* Bit writer over a caller-owned byte buffer. */
typedef struct
{
    uint8_t * p_buf;
    uint32_t  size;    /* bytes available                       */
    uint32_t  bit_pos; /* next bit to write                     */
    bool      b_overflow; /* a field did not fit: frame invalid */
} bitpack_writer_t;

/* Bit reader over a received byte buffer. */
typedef struct
{
    uint8_t const * p_buf;
    uint32_t        size;
    uint32_t        bit_pos;
    bool            b_overflow; /* read past the end: frame invalid */
} bitpack_reader_t;

uint32_t bitpack_max_u(uint32_t bits);
uint32_t bitpack_clamp_u(int32_t value, uint32_t bits);
int32_t  bitpack_clamp_s(int32_t value, uint32_t bits);
uint32_t bitpack_to_field_s(int32_t value, uint32_t bits);
int32_t  bitpack_from_field_s(uint32_t field, uint32_t bits);
int32_t  bitpack_quantise(float32_t value, float32_t scale, int32_t low,
                          int32_t high);

void     bitpack_writer_init(bitpack_writer_t * p_writer, uint8_t * p_buf,
                             uint32_t size);
void     bitpack_put_u(bitpack_writer_t * p_writer, uint32_t value,
                       uint32_t bits);
void     bitpack_put_s(bitpack_writer_t * p_writer, int32_t value,
                       uint32_t bits);
uint32_t bitpack_writer_bytes(bitpack_writer_t const * p_writer);

void     bitpack_reader_init(bitpack_reader_t * p_reader,
                             uint8_t const * p_buf, uint32_t size);
uint32_t bitpack_get_u(bitpack_reader_t * p_reader, uint32_t bits);
int32_t  bitpack_get_s(bitpack_reader_t * p_reader, uint32_t bits);

#endif /* BITPACK_H */

/*** end of file ***/
