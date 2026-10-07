# Buddy 2 — bench data capture sheet

Print this or keep it open at the bench. Every number here feeds a table in
`docs/REPORT.md` §4.2. Nothing in this file is pre-filled: measure it.

Run the dashboard alongside every session so the raw data is logged for you:

```
python tools/dashboard/picocar_dashboard.py --serial auto
```

Each session writes `logs/<date_time>/console.log` (the raw CSV the firmware
prints) and `telemetry.csv`. Keep those — they are your evidence.

---

## 0. Pre-flight (do once, before any tuning)

Flash `firmware/picocar_TEST_MOTOR_usbcdc.uf2`, open the serial port, press
**GP20**.

| Check | Expected | Seen | Action if wrong |
|---|---|---|---|
| Left wheel direction on positive duty | forward | | set `MOTOR_L_INVERT` to 1 |
| Right wheel direction on positive duty | forward | | set `MOTOR_R_INVERT` to 1 |
| `spdL` non-zero while left wheel turns | yes | | check GP2 wiring / `ENCODER_SLOTS` |
| `spdR` non-zero while right wheel turns | yes | | check GP3 wiring / `ENCODER_SLOTS` |
| Both speeds return to 0 within ~0.2 s of stop | yes | | check `ENCODER_TIMEOUT_US` |

Measured wheel diameter with calipers (before any calibration): ______ mm
Measured track width, wheel centre to wheel centre: ______ mm

Put these in `car_config.h` as the starting point — they beat the 65.0 / 130.0
placeholders even before the encoder calibration in §3.

---

## 1. Open-loop duty sweep → feed-forward

First **GP20** press prints `# open loop: duty,spdL,spdR`. Transcribe:

| Duty % | spdL mm/s | spdR mm/s | mean mm/s |
|---|---|---|---|
| 0 | | | |
| 10 | | | |
| 20 | | | |
| 30 | | | |
| 40 | | | |
| 50 | | | |
| 60 | | | |
| 70 | | | |
| 80 | | | |
| 90 | | | |
| 100 | | | |

**Derive the two feed-forward numbers.** The motor is modelled as
`duty = kf · speed + offset`, so plot **duty on the y-axis against mean speed
on the x-axis** and fit a straight line through the linear region (ignore the
dead-band at the bottom and any saturation at the top):

- `kf` = slope of that line, in % duty per (mm/s) → ______
- `offset` = y-intercept, i.e. the duty at which the wheels first move → ______

> If you plot it the other way round (speed vs duty, which is the more natural
> way to read the sweep), then `kf = 1 / slope` and `offset` is the
> **x-intercept**. Getting this backwards is the usual mistake — `kf` should
> land near 0.1, not near 10.

Apply without reflashing: `ff=<kf>,<offset>`

Dead-band note — lowest duty that produces any movement: ______ %
(This is your sanity check on `offset`; `SPEED_OFFSET_PCT` currently 18.0.)

---

## 2. Closed-loop step responses → PID tuning report

Every **GP20** press after the first repeats only the step test
(0 → 100 → 180 → 0 mm/s), printing the gains in use as a `# step test with
pid=… ff=…` header, then `t_ms,target,spdL,spdR,pwmL,pwmR` at 25 Hz.

Read these four numbers off the 180 mm/s step in the dashboard plot:

- **Rise time** — 10% to 90% of the 180 mm/s target (18 → 162 mm/s)
- **Overshoot** — `(peak − 180) / 180 × 100`
- **Settling time** — first time it enters ±5% (171–189 mm/s) and stays
- **Steady error** — `180 − mean(last 0.5 s)`

| # | kp | ki | kd | kf | offset | Rise (ms) | Overshoot (%) | Settle (ms) | Steady err (mm/s) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 0.08 | 0.60 | 0 | 0.10 | 18.0 | | | | | shipped default |
| 2 | | | | | | | | | | |
| 3 | | | | | | | | | | |
| 4 | | | | | | | | | | |
| 5 | | | | | | | | | | |

**Tuning order** (one variable at a time, ~10 s per iteration):

1. **kf first.** With kp = ki = kd = 0, the car should roughly reach target on
   feed-forward alone. Adjust `ff=` until the steady speed is within ~20%.
2. **kp.** Raise until you see slight overshoot, then back off ~30%.
3. **ki.** Raise until steady error goes to zero. Too much → slow oscillation.
4. **kd.** Only if oscillation remains. Start at 0.001–0.01; it acts on the
   measurement, so encoder noise amplifies here.

You need **at least 3 rows** for the report: the default, one intermediate,
and the final. Row 1 is already determined — just run it and read it off.

**Final gains → copy into `car_config.h`** (lines 91–95). They reset on reboot,
so this step is not optional:

```
SPEED_KP  ______    SPEED_KI  ______    SPEED_KD  ______
SPEED_KF  ______    SPEED_OFFSET_PCT  ______
```

Host-test cross-check — after editing `car_config.h`, run
`make -C tests/host`. `test_pid` re-simulates your new gains and asserts
settling within 5% and overshoot under 15%. If it fails, your gains are
unstable on the model even if the car looked fine; note that in the report.

---

## 3. Motion accuracy → calibration + evaluation

Flash `firmware/picocar_TEST_MOTION_usbcdc.uf2`. Each **GP20** press runs
forward 500 → left 90° → right 90° → backward 500 → U-turn 180°, printing
`odo=… heading=… ok=…` per move.

Run on a flat floor with a tape measure and protractor. Mark the start
position and heading with masking tape.

### Pass A — before calibration (`WHEEL_DIAMETER_MM` 65.0, `TRACK_WIDTH_MM` 130.0)

| Move | Commanded | Reported (odo/heading) | Run 1 | Run 2 | Run 3 | Run 4 | Run 5 | Mean | SD |
|---|---|---|---|---|---|---|---|---|---|
| Forward | 500 mm | | | | | | | | |
| Left turn | 90° | | | | | | | | |
| Right turn | 90° | | | | | | | | |
| Backward | 500 mm | | | | | | | | |
| U-turn | 180° | | | | | | | | |

### Compute the corrections

Use the **mean** of your 5 runs. `reported` is what the firmware printed.

**Wheel diameter** — from the forward/backward distance error:

```
WHEEL_DIAMETER_MM_new = 65.0 × (actual_mean_mm / reported_mm)
```
Worked example: commanded 500, firmware reports 500, tape says 520
→ 65.0 × (520 / 500) = **67.6 mm**

Average the forward and backward results; if they differ by more than a few
percent, suspect wheel slip rather than geometry.

New `WHEEL_DIAMETER_MM` = ______ mm

**Track width** — from the turn angle error:

```
TRACK_WIDTH_MM_new = 130.0 × (commanded_deg / actual_deg)
```
Worked example: commanded 90°, protractor says 85°
→ 130.0 × (90 / 85) = **137.6 mm**

Under-rotation means the real track is *wider* than assumed. Average the
left 90°, right 90° and 180° results. A persistent left/right asymmetry is
not a track-width problem — look at motor balance or `STRAIGHT_KP`.

New `TRACK_WIDTH_MM` = ______ mm

> Do the diameter first, reflash, then the track width. The turn arc
> `π·W·θ/360` is measured in encoder ticks, so a wrong diameter corrupts the
> track-width figure.

### Pass B — after calibration (this is the table that goes in the report)

| Move | Commanded | Measured (mean ± sd, n=5) | Error |
|---|---|---|---|
| Forward | 500 mm | | |
| Left turn | 90° | | |
| Right turn | 90° | | |
| Backward | 500 mm | | |
| U-turn | 180° | | |

Also record how many of the 25 moves reported `ok=0` (stall timeout): ______
A non-zero count is worth a sentence in the report — it is the stall
detection working, not a failure.

---

## 4. Transfer to the report

| This sheet | Goes to |
|---|---|
| §2 gains table, ≥3 rows | `docs/REPORT.md` §4.2 "PID tuning report" table |
| §2 final gains | `app_program/car_config.h` lines 91–95 |
| §3 Pass B table | `docs/REPORT.md` §4.2 "Motion accuracy evaluation" table |
| §3 calibrated geometry | `app_program/car_config.h` lines 72, 74 |
| Best step-response plot | report figure (dashboard **Save PNG**) |

Commit `car_config.h` and `REPORT.md` together once both are filled, so the
gains in the table always match the gains in the code.
