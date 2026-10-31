# Data Representation (Change Requirement 1)

Week 13 evidence for *Change Requirement 1: Minimum-Bit Data
Representation*: every application data field uses only the bits its valid
range needs, faults are bits of one shared mask, MQTT carries compact binary
frames, and packed data shared between tasks or with an ISR is accessed
safely.

Baseline ("before") is commit `d5f5199` on `main`; "after" is the commit that
adds this file. Both were built with the same toolchain
(`arm-none-eabi-gcc` 13.3.0, MSYS2) as `APP_MODE=MISSION CONSOLE=usb_cdc`.

## 1. Rules applied

These follow the Tribunal 1 decisions (Version B of the Tribunal 1 document).

| Rule | Why |
|---|---|
| **Pack at the interfaces.** Each owner keeps its working values at full width inside its own task and publishes one packed snapshot per cycle; readers unpack a private copy (`packed.c`). | Packing PID integrators, the hump accumulator or the filters would add rounding every step: the hump height grows about 0.06 mm per 20 ms step, so a 0.1 mm field would round it away. |
| **Bit-fields inside the firmware, masks and shifts on the wire.** | C99 leaves bit-field order and padding to the compiler; MQTT frames are read by Python, so they use explicit shifts, least significant bit first (`bitpack.c`, `frames.c`, `tools/dashboard/frames.py`). |
| **Unsigned bit-fields only.** Signed values are stored as n-bit two's complement and sign-extended when read. | Signedness of `int` bit-fields is implementation-defined. |
| **Clamp before storing.** Every value goes through `bitpack_quantise()` / `bitpack_clamp_*()`. | The firmware is built without `-Wconversion`, so a too-large value would otherwise wrap silently; now it saturates. |
| **One writer per word.** | The Cortex-M0+ (ARMv6-M) has no LDREX/STREX, so a read-modify-write of a word two writers share can lose an update. |
| **One documented format, shared encode/decode functions** (risk R3). | Producers and consumers cannot disagree on bit positions. |

## 2. Shared data and how it is accessed safely

| Shared item | Writer | Readers | Access | Why it is safe |
|---|---|---|---|---|
| Fault mask (`fault.c`) | one owner per bit | vehicle, telemetry | µT-Kernel event flag: `tk_set_flg` / `tk_clr_flg` / `tk_ref_flg` | Kernel calls are atomic; no task does a read-modify-write of the mask. |
| Fault counts, last-update times | owner of each fault | vehicle, telemetry | 8-bit / 16-bit array element per fault | Single writer per element; byte and half-word stores are atomic. |
| Motion snapshot `packed_motion_t` | motion task | vehicle, imu, telemetry | built locally, stored and read under DI/EI | One writer; the copy is one short interrupt-off section. |
| Motion request (12 B, `motion.c`) | API callers | motion task | packed and stored under DI/EI | As before the change: whole-struct copy under DI. |
| IMU snapshot `packed_imu_t` | imu task | vehicle, telemetry | as the motion snapshot | One writer. |
| Vehicle status `packed_vehicle_t` | vehicle task | telemetry | as the motion snapshot | One writer; the working status stays private. |
| Front sonar word `packed_front_t` | sonar monitor task | vehicle | one aligned 32-bit store / load, no lock | An aligned word access is atomic on the M0+. |
| IR element ring (64 × 2 B) | 1 kHz IR ISR | barcode task | lock-free SPSC ring; `head` and `tail` in separate words | Each index has one writer; packing them into one word would break this (Tribunal 1). |
| IR filter `g_filt` (3 × 16 bit) | IR ISR | tasks | one half-word store per update | Computed locally, stored once. |
| Sonar counts `g_counts` (1 B: close 2 bits, FAULT 2 bits) | sonar monitor task; reset by `obstacle_monitor_enable()` | vehicle | whole-byte store, whole-byte read | Both counts are worked out from one read and stored with one write; the reset is a whole-byte store (the same as the two separate words before). |
| Monitor handshake `gb_monitor`, `gb_monitor_busy` | vehicle / monitor task | the other task | one byte each | Kept apart on purpose: each flag has a different writer, and the pause handshake relies on two separate stores (section 4). |
| Last scan points (3 B each) and counts | vehicle task (scan) | telemetry / TEST_ULTRASONIC | stored and read under DI/EI | One writer; copied whole. |
| Servo angle `g_angle` (1 B) | the task moving the servo | sonar monitor | one byte store | Single store, atomic. |
| Echo width `g_width_us` (16 bit) | echo ISR | the pinging task | one half-word store | Single store, atomic on the M0+. |
| Encoder counters and periods | encoder ISR | motion task | unchanged 32-bit words | Documented exception (section 4). |
| MQTT link counters (`mqtt_bridge.c`) | CYW43 task | telemetry | packed into 16 B | All packed counters have the same writer; the drop counter (written by the telemetry task) and the link state keep their own words. |
| MQTT TX / RX rings | telemetry / CYW43 | CYW43 / telemetry | SPSC rings with memory barriers | Unchanged design; slots shrank. |

The single-core build (`SMP=0`, assumption A13) is still required for the
interrupt-off copies.

## 3. Data representation table

Ranges are the valid range of the quantity; *bits* is the stored width.
"0.1" means tenths of the unit. Unsigned fields saturate at their maximum.

### Motion snapshot `packed_motion_t` (Buddy 2) — 56 B → 32 B

| Field | Valid range | Resolution | Bits | Representation |
|---|---|---|---|---|
| `b_busy`, `b_last_ok` | 0 / 1 | – | 1 + 1 | flags |
| `speed_l/r`, `target_l/r` | ±600 mm/s (field ±1023) | 1 mm/s | 11 each | signed |
| `pwm_l/r` | −100 … +100 % | 1 % | 8 each | signed |
| `heading` | −180 … +180° (wrapped) | 0.1° | 12 | signed |
| `turn_rate` | ±529°/s (field ±819.1) | 0.1°/s | 14 | signed |
| `odo`, `dist_l/r` | ±838 m | 0.1 mm | 24 each | signed |
| `ticks_l/r` | unbounded counter | 1 edge | 32 each | exception: raw ISR counter, deltas by unsigned subtraction |

### Motion request (Buddy 2, private to `motion.c`) — 36 B → 12 B

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| `mode` | 5 modes | 3 | enum |
| `b_rev_l/r`, `b_invalid` | 0 / 1 | 1 each | direction −1 / move rejected |
| `seq` | wraps | 8 | the control task only tests for a change |
| `speed` | 0 … 1023 mm/s | 10 | 1 mm/s |
| `v_l/r` | ±1023 mm/s or ±100 % | 11 each | signed |
| `goal` | 0 … 104 m | 20 | 0.1 mm |

### IMU snapshot `packed_imu_t` (Buddy 4) — 84 B → 28 B

| Field | Valid range | Resolution | Bits |
|---|---|---|---|
| `b_ok` | last read succeeded | – | 1 |
| `event` | 8 motion classes | – | 3 |
| `hump_state` | 3 states | – | 2 |
| `humps`, `impacts`, `i2c_errors` | 0 … 255 (saturating) | 1 | 8 each |
| `seq` | wraps; +1 per good sample (freshness, risk R1) | 1 | 8 |
| `pitch`, `roll` | ±90° (field ±102.3) | 0.1° | 11 each |
| `heading` | 0 … 359.9° | 0.1° | 12 |
| `mag_rate`, `enc_rate` | ±250°/s (field ±409.5) | 0.1°/s | 13 each |
| `accel` | ±4095 mm/s² | 1 mm/s² | 13 |
| `ax`, `ay`, `az` | ±2 g (field ±2047 mg) | 1 mg | 12 each |
| `height` | ±204.7 mm | 0.1 mm | 12 |
| `peak`, `max_peak` | 0 … 204.7 mm | 0.1 mm | 11 each |

The tracker's running height, filters and calibration stay `float32_t` in
the imu task.

### Vehicle status `packed_vehicle_t` (Buddy 5) — 48 B → 16 B

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| `state` | 7 states | 3 | enum |
| `line_state` | 3 states | 2 | enum |
| `pending_nav`, `last_nav` | 5 commands | 3 each | enum |
| `last_action` | 5 actions | 3 | enum |
| `reason` | 27 texts | 5 | index into the reason table (`packed.c`); unknown text = "other" |
| `state_seq` | wraps | 8 | readers only test for a change |
| `barcodes`, `obstacles` | 0 … 255 (saturating) | 8 each | count |
| `cruise` | 60 … 300 mm/s | 9 | 1 mm/s |
| `line_error` | −1 … +1 | 11 | signed permille |
| `front` | 0 … 4094 mm, 4095 = no echo | 12 | mm |
| `run_ms` | 0 … 4.6 h (saturating) | 24 | ms |

### Obstacle profile `packed_profile_t` (Buddy 5) — 36 B → 16 B

| Field | Valid range | Bits |
|---|---|---|
| `b_found` | 0 / 1 | 1 |
| `closest` | 0 … 4094 mm, 4095 = none | 12 |
| `angle` | 0 … 180° | 8 |
| `left_edge`, `right_edge`, `centre` | ±4095 mm | 13 each (signed) |
| `width`, `clear_left`, `clear_right` | 0 … 4095 mm | 12 each |

### Small shared words

| Item | Layout | Before → after |
|---|---|---|
| Front sonar `packed_front_t` | bits 0–1 class (VALID / OUT_OF_RANGE / FAULT), 2–13 distance mm (4095 = none), 16–31 time ms mod 65536 (age) | 4 B distance only → 4 B with class and age |
| IR element `packed_ir_element_t` | bit 0 bar / space, bits 1–8 width in 1 ms sampler ticks, 255 = over-range | 8 B → 2 B |
| Barcode frame widths | 1 ms ticks, 0 … 254 (a bar of 255 resets the frame) | 4 B → 1 B each |
| Barcode message `packed_barcode_t` | bits 0–2 command (5), bits 3–8 Code 39 index (43 symbols, 63 = none) | 20 B → 2 B |
| Barcode statistics | 3 × 16-bit counters + last message | 28 B → 8 B |
| Vehicle queue message | command 3, speed 9, three line gains 20 bits each (0 … 1000.000 in 0.001) | 20 B → 12 B |
| IR filter state | Q4 of a 12-bit reading = 16 bits | 3 × 4 B → 3 × 2 B |
| Fault mask | 5 bits: motor stall, encoder, IMU, no echo, Wi-Fi loss | (new) |
| Sonar counts `g_counts` (Buddy 5) | bits 0–1 close readings in a row (0…`OBST_CONFIRM_READINGS`), bits 2–3 FAULT pings in a row (0…`OBST_FAULT_LIMIT`); a compile-time check keeps both limits ≤ 3 | 2 × 4 B → 1 B |
| Scan count `g_scans` (Buddy 5) | 8 bits, wraps (telemetry only tests for a change) | 4 B → 1 B |
| Last scan points (Buddy 5) | angle 0…180° in 8 bits + distance 0…4000 mm in 16 bits (0xFFFF = no echo), 40 points; count 0…40 in 8 bits | 160 B + 4 B → 120 B + 1 B |
| Servo angle (Buddy 5) | 0…180° in 8 bits | 4 B → 1 B |
| Echo width (Buddy 5) | 0…65 535 µs in 16 bits, saturated (11 m, still OUT_OF_RANGE past the 4 m range) | 4 B → 2 B |
| IR calibration white / black / sweep min / max (Buddy 3) | 12-bit ADC values in 16-bit storage, per channel | 3 × 4 × 2 B (unchanged; 16 bits is the smallest unit a 12-bit value fits) |
| MQTT link counters | published 16, commands 16 (wrap), reconnects / re-joins / restarts / attempts 8 (saturate), why 4, Wi-Fi state 3, IPv4 32 | 44 B → 16 B + 2 B drops + 1 B link |

## 4. Documented exceptions

| Data | Kept as | Reason |
|---|---|---|
| Encoder edge counters, edge periods (`encoder.c`) | 32-bit words | The counter is unbounded (deltas by unsigned subtraction); periods up to the 200 ms timeout need 18 bits. ISR-written words stay single-writer and lock-free instead of adding interrupt-off sections that would delay encoder time stamps (Tribunal 1). |
| Task-private working state (PID controllers, hump tracker, filters, calibration, odometry) | `float32_t` | Not shared; packing adds rounding that accumulates. |
| Microsecond time stamps | `uint32_t` | Full range needed for wrap-safe differences. |
| Scan points (`avoidance_point_t`) during a scan | 2 × `int16_t` on the vehicle stack | Temporary while scanning; the copy kept after the scan is packed (3 B per point, section 3). |
| Monitor handshake `gb_monitor` / `gb_monitor_busy`; IR flags `gb_calibrating` (task → ISR) and `gb_bar_black` (ISR → task) | one `bool` byte each | Each flag has a different writer. Packing two flags into one byte would need a read-modify-write, which the Cortex-M0+ cannot do atomically (no LDREX/STREX), so one writer could undo the other's change. |
| Line-following gains (`line_follow.c`) | 3 × `float32_t` | PID coefficients written by the `line=` command, read when the follower restarts; the queue message that carries them is packed (20 bits each). |
| Kernel, lwIP, CYW43 and USB data | template | Not application data. |

## 5. RAM usage: before and after

`arm-none-eabi-size`, MISSION build, USB console:

| | text | data | bss | data + bss |
|---|---|---|---|---|
| Before (`d5f5199`) | 79 485 | 92 | 21 624 | 21 716 |
| After | 87 989 | 92 | 17 508 | 17 600 |
| Change | +8 504 | 0 | −4 116 | **−4 116 B (−19 %)** |

Application RAM only (every `.bss` / `.data` symbol of `app_program/`,
from `arm-none-eabi-nm -S` on the object files): **6 128 B → 2 006 B
(−67 %)**.

| Symbol | Module | Before | After |
|---|---|---|---|
| `g_tx` (MQTT TX ring) | mqtt_bridge | 3 120 | 288 |
| `g_ring` (IR elements) | ir_sensor | 512 | 128 |
| `g_buf` → `g_frame` + `g_hex` | telemetry | 512 | 47 + 96 |
| `g_widths` (barcode frame) | barcode | 316 | 79 |
| `g_rx` (MQTT RX ring) | mqtt_bridge | 256 | 64 |
| `g_st` (IMU snapshot) | imu | 84 | 28 |
| `g_status` (motion snapshot) | motion | 56 | 32 |
| `g_stats` → `g_net` + `g_dropped` + `g_link` | mqtt_bridge | 44 | 16 + 2 + 1 |
| `g_cmd` (motion request) | motion | 36 | 12 |
| `g_last` (obstacle profile) | obstacle | 36 | 16 |
| `g_stats` (barcode) | barcode | 28 | 8 |
| `g_filt` (IR filter) | ir_sensor | 12 | 6 |
| `g_pub` (vehicle status, new) | vehicle | – | 16 |
| fault mask, counts, times (new) | fault | – | 19 |
| encoder-fault, IMU-fault and freshness state (new) | motion, imu | – | 24 |

The Wi-Fi builds save more: `mqtt_lwip.c`'s 512-byte publish buffer and
64-byte command buffer became 47 and 15 bytes. Kernel message buffers shrink
with their messages (barcode: 4 × 24 B → 4 × 6 B; vehicle: 20 → 12 B per
message).

**Update (23 Oct, after the Raees merge):** the Buddy 3 / 5 leftovers above
(sonar counts, scan count and points, servo angle, echo width), MISSION build,
USB console:

| | text | data | bss |
|---|---|---|---|
| Before the leftovers | 91 997 | 88 | 17 876 |
| After | 92 101 | 88 | 17 820 |
| Change | +104 | 0 | **−56 B** |

`obstacle.c`, `servo.c`, `ultrasonic.c` shared data: 194 B → 136 B
(sonar counts 8 → 1, scan count 4 → 1, scan points 164 → 121, servo angle
4 → 1, echo width 4 → 2).

**Update (30 Oct, INT-09):** both versions rebuilt with `arm-none-eabi-gcc`
15.2, so text differs from the 13.3.0 figures above. MISSION build, USB
console. The later features (encoder history, obstacle distances and others)
took back some of the saving:

| | text | data | bss | data + bss |
|---|---|---|---|---|
| Before (`d5f5199`) | 81 817 | 92 | 21 624 | 21 716 |
| Now | 95 753 | 96 | 17 880 | 17 976 |
| Change | +13 936 | +4 | −3 744 | **−3 740 B (−17 %)** |

Application RAM (same method): **6 128 B → 2 359 B (−62 %)**.

**Cost:** +8.5 KB of code (pack / unpack and frame functions) out of 2 MB
flash, and task stacks are unchanged. The barcode task now widens a frame's
widths into a 316-byte array on its own stack while it decodes.

## 6. Message sizes: before and after

"Before" telemetry and heartbeat sizes are from the mosquitto log of a run
on 16 Oct (`Received PUBLISH ... (268 bytes)` / `(208 bytes)`); the other
JSON sizes are the old format with typical values. "After" sizes are exact
(`tests/host/test_data.c`).

| Message | JSON / text (before) | Binary frame (after) |
|---|---|---|
| Telemetry, 5 Hz | 268 B | **43 B** (−84 %) |
| Heartbeat, 1 Hz | 208 B | **26 B** (−88 %) |
| State event | 49 B | 2 B |
| Barcode event | 42 B | 3 B |
| Hump event | 46 B | 5 B |
| Obstacle event | 114 B | 11 B |
| Impact event | 23 B | 4 B |
| Command reply | 51 B | 4 B |
| Gains report | 115 B | 24 B |
| Timing report | about 270 B (estimated) | 26 B (averages on the console) |
| Status (retained, last will) | 6–7 B | 2 B |
| Command `speed=150` | 9 B | 5 B |
| Command `#7 pid=0.08,0.6,0` | 17 B | 11 B |
| Command `stop` | 4 B | 4 B |

Telemetry payload traffic drops from about 1.34 KB/s to 215 B/s at 5 Hz.
With the MQTT header and topic (about 26 B per publish), lwIP's 1 024-byte
MQTT output buffer now holds about 14 telemetry messages instead of 3. The MQTT fixed header and topic name (for example
`picocar/car1/telemetry`, 22 B) are now larger than most payloads; topics are
protocol, not application data, so they were left as they are.

### Frame layouts

Byte 0 = frame type (bits 0–3) + format version 1 (bits 4–7); then fields
least significant bit first. `tools/dashboard/frames.py` decodes them into
the same field names as the old JSON.

| Frame (type) | Fields in order (bits) |
|---|---|
| Telemetry (1) | run ms 24, state 3, line state 2, last nav 3, pending nav 3, speed L/R 11 + 11, target L/R 11 + 11, PWM L/R 8 + 8, encoder L/R low 16 + 16, odometer mm 20 (signed), heading 0.1° 12 (signed), IR ×3 permille 10 each, line error permille 11 (signed), barcodes 8, last symbol 6, pitch / roll 0.1° 11 + 11, IMU heading 0.1° 12, turn rate 0.1°/s 13 (signed), hump height 0.1 mm 12 (signed), peak 11, run maximum 11, humps 8, motion class 3, front mm 12 (4095 = none), avoidance action 3, obstacles 8, fault mask 5 |
| Heartbeat (2) | uptime s 20, sequence 16, state 3, link 3, published 16, dropped 16, commands 16, reconnects 8, re-joins 8, chip restarts 8, Wi-Fi state 3, IPv4 32, attempts 8, why 4, IMU ok 1, I2C errors 8, fault mask 5, fault counts 4 × 5 |
| State (3) | state 3, reason 5 |
| Barcode (4) | symbol 6, command 3 |
| Hump (5) | count 8, peak 0.1 mm 11, maximum 11 |
| Obstacle (6) | found 1, closest 12, angle 8, width 12, centre 13 (signed), clear left 12, clear right 12, action 3 |
| Impact (7) | count 8, strength mg 12 (peak jerk of the last two 100 ms event checks, saturating at 4095) |
| Reply (8) | request id 16, command 4, error 4 (0 ok, 1 invalid, 2 bad frame, 3 driving, 4 unavailable) |
| Gains (9) | speed PID 3 × 20, kf 20, offset 17, line PID 3 × 20 (all 0.001 steps), cruise 9, rate ms 13 |
| Timing (10) | window ms 16, load % 7, jitter µs 16, periods 16, max µs 9 × 16 |
| Status (11) | online 1 |
| Command (12, laptop → car) | command 4, request id 16, then: speed 9 / rate 13 / nav 3 / map nav 3 + Code 39 index 6 / detect 9 / ff 20 + 17 / pid or line 3 × 20 |

The serial console keeps text commands for people; MQTT takes only command
frames. A frame longer than 15 bytes is dropped whole, never truncated.

## 7. Correctness and performance

Done on the PC:

- `make -C tests/host`: **187 passed, 0 failed** (123 before; 64 new checks
  in `tests/host/test_data.c`): every pack / unpack pair at its range limits,
  saturation instead of wrapping, two's complement edges, bit order, every
  frame length, every command kind in both directions, and rejection of
  truncated, wrong-version, wrong-type and out-of-range command frames.
- The C frames were decoded by `frames.py` field by field, and Python's
  command frames are byte-identical to `frames_command()`.
- All 9 firmware modes and the Wi-Fi TEST_TELEMETRY and MISSION builds: 0
  warnings.
- `tools/barr_check.py`: no new findings (the 11 that remain were already on
  `main`).
- `sim_car.py` runs offline through `frames.py` with every message type.

Still to measure on the car (no hardware was used for this change):

| Test | Pass |
|---|---|
| `timing` before / after with 5 Hz telemetry: motion, vehicle, barcode, imu, hump, ir_isr, net | no task over its period; `jit_max_us` ≤ 2 000 (A2) |
| Encoders at top speed while motion and imu read the packed snapshots, 5 min | no torn or lost counts against a reference |
| Each barcode 10 times at cruise and at the slowest hump speed, stopping over a bar | same decode rate as before; a stopped bar never decodes |
| 2 m straight drive | drift within the Buddy 4 limit, before and after |
| Fault injection per fault bit (unplug sonar, block IMU, stall a wheel) while line following | STOPPED with the right reason within 100 ms (NFR3) |
| Dashboard over MQTT for 10 min (`wifi_test.py`) | ≥ 4.5 Hz telemetry, no drops, all fields decode |

## 8. Files

| File | Owner | What |
|---|---|---|
| `app_program/bitpack.c/.h` | shared | clamp, sign-extend, bit writer / reader |
| `app_program/packed.c/.h` | shared (bit map: Buddy 5) | packed snapshot types and their pack / unpack |
| `app_program/fault.c/.h` | shared (bit map: Buddy 5) | fault mask in an event flag, counts, update times |
| `app_program/subsystem1/frames.c/.h` | Buddy 1 | MQTT frames, command frame decoder |
| `tools/dashboard/frames.py` | Buddy 1 | the same frames in Python |
| `tests/host/test_data.c` | shared | the tests above |
| `docs/FAULT_PATHS.md` | Buddy 5 | every fault: detection, latency, response, recovery |

