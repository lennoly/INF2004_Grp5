/** @file car_types.h
 *
 * @brief Project-wide type names required by BARR-C Rule 5.4.b.i.
 *
 * C99 defines no fixed-width floating-point names, so they are declared
 * here once.  The array-size trick below is a C99 compile-time check (a
 * negative array size is a compile error) that float really is the
 * 32-bit IEEE-754 type on this compiler.
 */

#ifndef CAR_TYPES_H
#define CAR_TYPES_H

typedef float  float32_t;
typedef double float64_t;

typedef char car_types_float32_is_4_bytes_t[(4u == sizeof(float32_t)) ? 1 : -1];

#endif /* CAR_TYPES_H */

/*** end of file ***/
