# How the buddies talk to each other (telemetry)

Short overview of how each subsystem's data reaches the laptop, and how
commands come back. Field widths are in `DATA_REPRESENTATION.md`.

## The idea

Each buddy's task owns its own data and **publishes a snapshot**. Nobody
writes into another buddy's variables. The telemetry task (Buddy 1) only
**reads** those snapshots, packs them into binary frames and sends them.

```
 Buddy 2 motion (prio 5) ──┐
 Buddy 3 IR + barcode (7) ─┤   snapshot getters        Buddy 1 telemetry (10)
 Buddy 4 IMU + humps (8) ──┼──────────────────────────► build frame ─► MQTT / USB
 Buddy 5 sonar (9) ────────┤   (read-only copies)            ▲
 Buddy 5 vehicle (6) ──────┘                                 │
                                                             │ commands
 laptop dashboard ◄── picocar/car1/telemetry|event|heartbeat │ picocar/car1/cmd
```

Lower number = higher priority. Motion and the vehicle controller run first;
telemetry runs last, so sending data never delays driving.

## What each buddy shares

| Buddy | Owner task | Shared through | What telemetry gets |
|---|---|---|---|
| 2 – motors, encoders | motion | `motion_get_status()` | wheel speed, target, PWM, encoders, odometer, heading |
| 3 – line, barcode | IR sampler ISR, barcode task | `ir_sensor_read()`, `barcode_get_stats()` | IR levels, line error, last barcode, count |
| 4 – IMU, terrain | imu | `imu_get_status()`, `imu_take_peak_jerk()` | pitch, roll, yaw, turn rate, motion event, humps, impacts, shock mg |
| 5 – sonar, obstacles | obstacle | `obstacle_get_last()`, front reading | front distance, last scan (distance, width, centre, clearances, action) |
| 5 – vehicle | vehicle | `vehicle_get_status()` | state, reason, pending turn, obstacles passed |
| all | any owner | `fault_mask()` | fault bits (one writer per bit) |

**How a snapshot stays consistent:** the owner builds it in a local copy,
then stores it in one step with interrupts off (single core). A reader takes
its own copy the same way. A half-written snapshot is never read, and no
mutex is needed. Small values that an ISR shares (front distance, IR) are
one word, written with a single store.

**Hand-offs between buddies:**
- **Barcode → vehicle:** the barcode task posts each decoded letter into a
  message buffer, and the vehicle task takes it as a turn command.
- **Commands → vehicle:** `vehicle_command()` posts to the vehicle's
  message buffer (start, stop, speed, nav), so only the vehicle task changes
  its own state.

## What goes out, and how often

| Frame | When | Built from |
|---|---|---|
| Telemetry | every 200 ms (5 Hz) over WiFi, 1 s over USB | all snapshots above |
| Event | checked every 100 ms, sent only on a change | state change, barcode, hump, impact (with mg), obstacle scan |
| Heartbeat | every 1 s | uptime, link counters, IMU health, fault counts |
| Reply / gains | after each command | command result, gains in use |

Events are found by comparing counters with the last ones sent, e.g.
`state_seq`, `decoded`, `humps`, `impacts`, scan count. A change that comes
and goes between two checks is still reported once.

**Links:**
- **WiFi:** the frame is the MQTT payload on `picocar/car1/telemetry`,
  `/event`, `/heartbeat` and `/status` (online/offline). It goes through
  the `mqtt_bridge` ring buffer, so a slow network drops and counts frames
  instead of blocking.
- **USB:** the same frame is printed as `@` + hex on the console.

The dashboard decodes both with `tools/dashboard/frames.py`.

## Commands back to the car

```
dashboard ─► picocar/car1/cmd (binary) or USB text ─► command parser (Buddy 1)
          ─► vehicle_command()        start / stop / calibrate / speed / nav
          ─► motion_set_*()           pid= ff=  (Buddy 2 gains)
          ─► barcode_decode_set_symbol()  map=  (Buddy 3 demo letters)
```

Tuning commands (`speed= pid= ff= line= map= detect=`) are refused while the car
drives. Every command gets a reply frame (ok, or the reason it was refused).
