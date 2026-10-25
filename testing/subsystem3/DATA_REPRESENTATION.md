# Subsystem 3 representation and memory changes

This change packs the high-volume ISR stream and the navigation message
transport. It does **not** claim completion of the project-wide minimum-bit
change requirement, MQTT conversion, total linked RAM comparison, or MCU
performance measurements.

| Data | Valid range | Information bits | Representation |
|---|---|---:|---|
| Captured element duration | 0–250000 microseconds | 18 | Low 18 bits of a three-byte entry |
| Element colour | space/bar | 1 | Bit 18 of that entry |
| Course barcode symbol | seven-bit ASCII | 7 | Masked field in byte 0 of message |
| Navigation command | NONE, LEFT, RIGHT, STRAIGHT, UTURN | 3 | Low three bits of message byte 1 |
| Event timestamp | 0–2^32−1 milliseconds, modulo wrap | 32 | Four explicit little-endian message bytes |
| Capture session | modulo 2^32 reset generation | 32 | Four little-endian bytes reject results from an interrupted older session |

Byte-oriented storage rounds the 19-bit ring element to three bytes and the
74-bit message to ten bytes. Unused bits are zero. Durations longer than the
quiet interval are saturated, because only the comparison with that interval
is needed; a compile-time check prevents configuring an unrepresentable
quiet interval.

## Local storage comparison

| Storage | Before | After | Saving |
|---|---:|---:|---:|
| 64-entry ISR ring | 512 B (64 × 8) | 192 B (64 × 3) | 320 B |
| Barcode capture widths | 316 B (79 × 4) | 116 B (29 × 4) | 200 B |
| Navigation message payload | 20 B | 10 B | 10 B per message |
| Requested four-message kernel buffer, including headers/alignment | 96 B | 64 B | 32 B |

The host tests report `sizeof(barcode_event_t)` and the configured message
payload. Kernel payload storage is rounded to a four-byte word, so ten-byte
payloads reserve twelve bytes each plus the message header. These are local
storage calculations, not the total RAM reduction: additional diagnostics
consume memory and compiler/linker layout affects the final image.

## Concurrency

The ISR is the only ring producer and the barcode task is the only consumer.
It writes all three bytes before publishing the head with a memory barrier.
The consumer copies them before advancing the tail. The producer never
overwrites the consumer's slot. Lost elements increment an overrun counter;
the consumer discards the damaged frame and resynchronises on white.

Calibration application and ADC snapshots use interrupt-protected sections.
START/STOP request a new capture generation under interrupt protection; they
never modify the ring tail or the decoder frame. Only the barcode consumer
applies the reset on its next poll. Queue messages carry that generation so an
older decoder interrupted by a reset cannot publish a usable stale event.
The extra four payload bytes are intentional correctness metadata. The
assumptions remain the single-core build and one consumer. The regression
suite injects reset requests during both consumer barriers and before enqueue;
it does not establish hardware timing or SMP correctness.

## Remaining final-submission work

Public snapshots and diagnostic structs retain the existing types so other
buddies' interfaces remain compatible. ADC values in those structs are 12-bit
information in `uint16_t`, enums use the existing C ABI, and PID/normalised
values use floating point. A complete minimum-bit migration still needs
agreed precision/ranges for those fields and a coordinated interface update.
The byte-packed ring also retains byte-alignment slack.

For Week 13, measure the ARM image's `.data + .bss`, stack high-water marks,
ISR execution time and barcode task execution time before/after on the same
hardware and build settings. Attach those measurements; do not substitute
host runtime or the local table for MCU evidence.
