/** @file bitpack.c
 *
 * @brief Minimum-bit data representation helpers (see bitpack.h).
 *
 * Pure functions: no kernel calls, so tests/host covers every one at its
 * range limits.
 */

#include <stdint.h>
#include <stdbool.h>
#include "car_math.h"
#include "bitpack.h"

#define BITS_PER_BYTE (8u)
#define BYTE_MASK     (0xFFu)
#define ALL_ONES      (0xFFFFFFFFu)

/*!
 * @brief Largest value an unsigned field of the given width holds.
 *
 * @param[in] bits Field width, 1..32.
 *
 * @return 2^bits - 1.
 */
uint32_t
bitpack_max_u (uint32_t bits)
{
    uint32_t result = ALL_ONES;

    if (bits < BITPACK_MAX_BITS)
    {
        result = (1u << bits) - 1u;
    }

    return (result);
}

/*!
 * @brief Clamp a value into an unsigned field (saturate, never wrap).
 *
 * @param[in] value Value to store.
 * @param[in] bits  Field width, 1..31.
 *
 * @return 0 .. 2^bits - 1.
 */
uint32_t
bitpack_clamp_u (int32_t value, uint32_t bits)
{
    uint32_t max    = bitpack_max_u(bits);
    uint32_t result = 0u;

    if (value > 0)
    {
        /* Cast: value is positive, so the conversion keeps it. */
        result = ((uint32_t) value > max) ? max : (uint32_t) value;
    }

    return (result);
}

/*!
 * @brief Clamp a value into a signed field (saturate, never wrap).
 *
 * @param[in] value Value to store.
 * @param[in] bits  Field width, 2..31.
 *
 * @return -2^(bits-1) .. 2^(bits-1) - 1.
 */
int32_t
bitpack_clamp_s (int32_t value, uint32_t bits)
{
    /* Cast: bits <= 31, so the half range fits in int32_t. */
    int32_t high   = (int32_t) bitpack_max_u(bits - 1u);
    int32_t low    = -high - 1;
    int32_t result = value;

    if (value > high)
    {
        result = high;
    }
    else if (value < low)
    {
        result = low;
    }
    else
    {
        /* Already inside the field's range. */
    }

    return (result);
}

/*!
 * @brief Clamp a signed value and encode it as an n-bit two's complement
 *        field (for an unsigned bit-field or a wire field).
 *
 * @param[in] value Value to store.
 * @param[in] bits  Field width, 2..31.
 *
 * @return The field's bit pattern.
 */
uint32_t
bitpack_to_field_s (int32_t value, uint32_t bits)
{
    /* Cast: two's complement conversion of a value already clamped into
       the field's range; the mask keeps the low bits. */
    return ((uint32_t) bitpack_clamp_s(value, bits) & bitpack_max_u(bits));
}

/*!
 * @brief Sign-extend an n-bit two's complement field.
 *
 * @param[in] field Field bit pattern (only the low bits are used).
 * @param[in] bits  Field width, 2..31.
 *
 * @return The signed value.
 */
int32_t
bitpack_from_field_s (uint32_t field, uint32_t bits)
{
    uint32_t mask     = bitpack_max_u(bits);
    uint32_t sign_bit = 1u << (bits - 1u);
    uint32_t value    = field & mask;
    int32_t  result   = 0;

    if (0u != (value & sign_bit))
    {
        /* Cast: (mask - value) + 1 is the magnitude of a negative field,
           at most 2^(bits-1), which fits in int32_t for bits <= 31. */
        result = -(int32_t) ((mask - value) + 1u);
    }
    else
    {
        /* Cast: a positive field is below 2^(bits-1). */
        result = (int32_t) value;
    }

    return (result);
}

/*!
 * @brief Scale and round a measurement into an integer range, clamping
 *        before the conversion so that it cannot overflow.
 *
 * @param[in] value Measurement in engineering units.
 * @param[in] scale Steps per unit (10.0f for tenths, 1000.0f for permille).
 * @param[in] low   Lowest representable value (in steps).
 * @param[in] high  Highest representable value (in steps).
 *
 * @return Rounded value in [low, high]; 0 clamped into range if the
 *         measurement is not finite.
 */
int32_t
bitpack_quantise (float32_t value, float32_t scale, int32_t low, int32_t high)
{
    /* Casts: the limits are field ranges below 2^24, exact in float32_t. */
    float32_t scaled =
        car_math_clamp(value * scale, (float32_t) low, (float32_t) high);

    return (car_math_round(scaled));
}

/*!
 * @brief Start writing a frame; the buffer is cleared first.
 *
 * @param[out] p_writer Writer state.
 * @param[out] p_buf    Destination buffer.
 * @param[in]  size     Buffer size in bytes.
 */
void
bitpack_writer_init (bitpack_writer_t * p_writer, uint8_t * p_buf,
                     uint32_t size)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < size; idx++)
    {
        p_buf[idx] = 0u;
    }

    p_writer->p_buf      = p_buf;
    p_writer->size       = size;
    p_writer->bit_pos    = 0u;
    p_writer->b_overflow = false;
}

/*!
 * @brief Append an unsigned field, least significant bit first.  The
 *        value must already be in range (clamp it first); extra high bits
 *        are masked off.
 *
 * @param[in,out] p_writer Writer state.
 * @param[in]     value    Field value.
 * @param[in]     bits     Field width, 1..32.
 */
void
bitpack_put_u (bitpack_writer_t * p_writer, uint32_t value, uint32_t bits)
{
    uint32_t idx  = 0u;
    uint32_t pos  = 0u;
    uint32_t data = value & bitpack_max_u(bits);

    if ((p_writer->bit_pos + bits) > (p_writer->size * BITS_PER_BYTE))
    {
        p_writer->b_overflow = true;
    }
    else
    {
        for (idx = 0u; idx < bits; idx++)
        {
            pos = p_writer->bit_pos + idx;

            if (0u != ((data >> idx) & 1u))
            {
                /* Cast: the shifted bit is 1..128, which fits uint8_t. */
                p_writer->p_buf[pos / BITS_PER_BYTE] |=
                    (uint8_t) (1u << (pos % BITS_PER_BYTE));
            }
        }

        p_writer->bit_pos += bits;
    }
}

/*!
 * @brief Append a signed field (clamped, two's complement).
 *
 * @param[in,out] p_writer Writer state.
 * @param[in]     value    Field value.
 * @param[in]     bits     Field width, 2..31.
 */
void
bitpack_put_s (bitpack_writer_t * p_writer, int32_t value, uint32_t bits)
{
    bitpack_put_u(p_writer, bitpack_to_field_s(value, bits), bits);
}

/*!
 * @brief Bytes used so far (a partly filled last byte counts as one).
 *
 * @param[in] p_writer Writer state.
 *
 * @return Frame length in bytes.
 */
uint32_t
bitpack_writer_bytes (bitpack_writer_t const * p_writer)
{
    return ((p_writer->bit_pos + (BITS_PER_BYTE - 1u)) / BITS_PER_BYTE);
}

/*!
 * @brief Start reading a received frame.
 *
 * @param[out] p_reader Reader state.
 * @param[in]  p_buf    Received bytes.
 * @param[in]  size     Number of bytes.
 */
void
bitpack_reader_init (bitpack_reader_t * p_reader, uint8_t const * p_buf,
                     uint32_t size)
{
    p_reader->p_buf      = p_buf;
    p_reader->size       = size;
    p_reader->bit_pos    = 0u;
    p_reader->b_overflow = false;
}

/*!
 * @brief Read an unsigned field.
 *
 * @param[in,out] p_reader Reader state.
 * @param[in]     bits     Field width, 1..32.
 *
 * @return The field (0 if the frame is too short; b_overflow is set).
 */
uint32_t
bitpack_get_u (bitpack_reader_t * p_reader, uint32_t bits)
{
    uint32_t idx    = 0u;
    uint32_t pos    = 0u;
    uint32_t result = 0u;

    if ((p_reader->bit_pos + bits) > (p_reader->size * BITS_PER_BYTE))
    {
        p_reader->b_overflow = true;
    }
    else
    {
        for (idx = 0u; idx < bits; idx++)
        {
            pos = p_reader->bit_pos + idx;

            /* Cast: widen the byte before shifting (no signed promotion). */
            if (0u != (((uint32_t) p_reader->p_buf[pos / BITS_PER_BYTE]
                        >> (pos % BITS_PER_BYTE))
                       & 1u))
            {
                result |= (1u << idx);
            }
        }

        p_reader->bit_pos += bits;
    }

    return (result);
}

/*!
 * @brief Read a signed (two's complement) field.
 *
 * @param[in,out] p_reader Reader state.
 * @param[in]     bits     Field width, 2..31.
 *
 * @return The sign-extended value.
 */
int32_t
bitpack_get_s (bitpack_reader_t * p_reader, uint32_t bits)
{
    return (bitpack_from_field_s(bitpack_get_u(p_reader, bits), bits));
}

/*** end of file ***/
