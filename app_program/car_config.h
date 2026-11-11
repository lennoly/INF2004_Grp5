/** @file car_config.h
 *
 * @brief Single source of truth for the PicoCar pin map and tunables.
 *
 * Every hardware pin and every calibration constant lives here so that the
 * five subsystems never hard-code a number (Barr-C: no magic numbers).
 * Values marked [CALIBRATE] must be measured on your own car.
 *
 * Board : Raspberry Pi Pico W on a Cytron Robo Pico carrier.
 * Pins reserved by the template: GP0/GP1 UART console, GP16 liveness LED,
 * GP23/24/25/29 CYW43 radio.
 */

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* ------------------------------------------------------------------------ */
/* Application mode (selected at build time: make APP_MODE=TEST_IMU ...)    */
/* ------------------------------------------------------------------------ */
#define APP_MODE_MISSION            (0)  /* Full integrated robot          */
#define APP_MODE_TEST_MOTOR         (1)  /* Buddy 2: open loop + PID step  */
#define APP_MODE_TEST_MOTION        (2)  /* Buddy 2: distance/turn accuracy*/
#define APP_MODE_TEST_IR            (3)  /* Buddy 3: line sensors          */
#define APP_MODE_TEST_BARCODE       (4)  /* Buddy 3: barcode decoder       */
#define APP_MODE_TEST_IMU           (5)  /* Buddy 4: IMU / hump / events   */
#define APP_MODE_TEST_ULTRASONIC    (6)  /* Buddy 5: scan + profile + plan */
#define APP_MODE_TEST_TELEMETRY     (7)  /* Buddy 1: MQTT telemetry        */
#define APP_MODE_TEST_WHEELS        (8)  /* Buddy 2: raw motor pin check   */
#define APP_MODE_TEST_OBSTACLE      (10) /* Buddy 5: MISSION, straight     */

#ifndef APP_MODE
#define APP_MODE                    APP_MODE_MISSION
#endif

/* ------------------------------------------------------------------------ */
/* Pin map (see WIRING.md)                                                  */
/* ------------------------------------------------------------------------ */
/* Motors - fixed on the Robo Pico (MxA high + MxB low = forward).          */
#define PIN_MOTOR_L_A               (8u)   /* M1A, PWM4 A                   */
#define PIN_MOTOR_L_B               (9u)   /* M1B, PWM4 B                   */
#define PIN_MOTOR_R_A               (10u)  /* M2A, PWM5 A                   */
#define PIN_MOTOR_R_B               (11u)  /* M2B, PWM5 B                   */

/* Wheel encoders (LM393 slotted opto, D0 output) - Grove port 2.           */
#define PIN_ENCODER_L               (2u)
#define PIN_ENCODER_R               (3u)

/* IMU LSM303D (0x1D) on I2C0 - Grove port 3.                               */
#define PIN_IMU_SDA                 (4u)
#define PIN_IMU_SCL                 (5u)

/* Ultrasonic HC-SR04: TRIG on Grove 5 (GP6), ECHO on Grove 7 (GP7).        */
#define PIN_US_TRIG                 (6u)
#define PIN_US_ECHO                 (7u)

/* Servo carrying the ultrasonic sensor - Robo Pico servo port GP12.        */
#define PIN_SERVO                   (12u)

/* IR reflective sensors, analog output (A0) -> ADC.                        */
#define PIN_IR_LEFT                 (26u)  /* ADC0, Grove 6                 */
#define PIN_IR_RIGHT                (27u)  /* ADC1, Grove 6                 */
#define PIN_IR_BARCODE              (28u)  /* ADC2, Grove 7                 */
#define ADC_CH_IR_LEFT              (0u)
#define ADC_CH_IR_RIGHT             (1u)
#define ADC_CH_IR_BARCODE           (2u)

/* On-board user buttons (active low).                                      */
#define PIN_BTN_START               (20u)
#define PIN_BTN_STOP                (21u)

/* ------------------------------------------------------------------------ */
/* Chassis geometry [CALIBRATE]                                             */
/* ------------------------------------------------------------------------ */
/* [CALIBRATE] effective, not the 64 mm tyre: 65 x 450 / 497 from the  */
/* TEST_MOTION runs on 28 Oct (encoders 497 mm, tape 450 mm).         */
#define WHEEL_DIAMETER_MM           (58.9f)
#define ENCODER_SLOTS               (559u)    /* slots on the encoder disc   */
/* [CALIBRATE] effective (tyres scrub in turns): 110 under, 147 over.   */
/* 28 Oct, after the 58.9 diameter: 138 turned ~93 / ~190 deg for an   */
/* encoder 87 / 177.5, so the real track is ~129; 132 also covers the */
/* turn stopping ~3 deg early (expect ~89 and ~182 deg).              */
#define TRACK_WIDTH_MM              (132.0f) /* wheel centre to wheel centre*/
#define CAR_WIDTH_MM                (160.0f)
#define CAR_LENGTH_MM               (200.0f)
#define LINE_SENSOR_AHEAD_MM        (140.0f) /* IR row ahead of the axle    */

/* Set to 1 if a motor spins backwards for a "forward" command.             */
#define MOTOR_L_INVERT              (0)
#define MOTOR_R_INVERT              (0)

/* ------------------------------------------------------------------------ */
/* Motion control (Buddy 2)                                                 */
/* ------------------------------------------------------------------------ */
#define MOTOR_PWM_FREQ_HZ           (10000u)  /* Robo Pico max is 20 kHz    */
#define CONTROL_PERIOD_MS           (20u)     /* 50 Hz control loop         */
/* 100 let motor noise through on the floor (fake speed spikes).          */
#define ENCODER_GLITCH_US           (180u)    /* ignore faster edges        */
/* [CALIBRATE] fastest wheel speed (TEST_MOTOR sweep at 100 % duty); the
   glitch filter must stay below the shortest half-period at this speed.   */
#define SPEED_WHEEL_MAX_MM_S        (900.0f)
#define ENCODER_TIMEOUT_US          (200000u) /* no edge => wheel stopped   */

#define SPEED_KP                    (0.104f)  /* [CALIBRATE] %duty/(mm/s)   */
#define SPEED_KI                    (0.60f)
#define SPEED_KD                    (0.0f)
#define SPEED_KF                    (0.12f)   /* feed-forward %duty/(mm/s)  */
#define SPEED_OFFSET_PCT            (2.3f)    /* static friction offset     */
#define STRAIGHT_KP                 (2.0f)    /* (mm/s) per mm L-R mismatch */

#define SPEED_CRUISE_MM_S           (75.0f)   /* [CALIBRATE] 4 Nov (was 50) */
#define SPEED_HUMP_UP_MM_S          (100.0f)  /* [CALIBRATE] climbing     */
#define SPEED_HUMP_DOWN_MM_S        (75.0f)   /* [CALIBRATE] descending    */
/* Everything but the humps at the cruise speed, so the car moves the same */
/* way throughout (4 Nov: 75; were 160, 120 and 300 mm/s, then 50).       */
#define SPEED_SLOW_MM_S             (75.0f)   /* settling cap              */
#define SPEED_TURN_MM_S             (75.0f)   /* motion_turn_*()           */
#define SPEED_BYPASS_MM_S           (75.0f)   /* [CALIBRATE] obstacle pass */
/* Range of the speed= command.  NFR2: at most 300 mm/s, so the barcode
   sensor still gets >= 10 samples on a 3 mm bar (1 kHz sampler).          */
#define SPEED_CMD_MIN_MM_S          (50)
#define SPEED_CMD_MAX_MM_S          (300)
#define MOTION_BRAKE_MARGIN_MM      (4.0f)

/* ------------------------------------------------------------------------ */
/* Line following + barcode (Buddy 3)                                       */
/* ------------------------------------------------------------------------ */
#define IR_SAMPLE_PERIOD_US         (1000u)   /* 1 kHz sampler ISR          */
#define IR_BLACK_IS_HIGH            (1)       /* A0 rises over black        */
#define IR_DEFAULT_WHITE            (200u)    /* [CALIBRATE] raw ADC counts */
#define IR_DEFAULT_BLACK            (2400u)   /* [CALIBRATE] TEST_IR 22 Oct */

#define LINE_KP                     (18.0f)   /* (mm/s) per unit error; 6 Nov: 45 -> 35 -> 18, jitter (200324) */
#define LINE_KI                     (0.0f)
#define LINE_KD                     (0.0f)
#define LINE_LOST_LEVEL             (0.15f)   /* both below => off line     */
#define LINE_JUNCTION_LEVEL         (0.75f)   /* both above => junction     */
#define LINE_JUNCTION_MIN_MM        (10.0f)
#define LINE_JUNCTION_REARM_MM      (100.0f)  /* [CALIBRATE] off black, rearm */
#define LINE_LOST_CONFIRM_MS        (150u)

/* 14 cm barcode (S3-U03): narrow bars peak at 190-810 permille driving at
   100 mm/s, white paper reads 0-21. */
/* 6 Nov (TEST_IR 174231, sensor at running height): white 187-270 raw
   (max 318), held on black 2300-2900; narrow bars passing at speed peaked
   at 484 and 564 raw, 13 and 16 %, so at 15 % some were missed and the
   rest read as 7-20 ms tips.  Was 0.15 / 0.08; 0.08 is 376 raw, 0.05 310. */
#define BARCODE_BLACK_LEVEL         (0.08f)   /* [CALIBRATE] hysteresis     */
#define BARCODE_WHITE_LEVEL         (0.05f)
/* White gap ending a frame.  6 Nov (180900): wide spaces of about 20-25 mm
   took 184-250+ ms at 100 mm/s, so 250 split the frame in two (7 + 3
   elements); at the 75 mm/s cruise they take up to 330 ms.  500 ms is 37 mm
   at 75.  Above 254 ms only the idle timer ends a frame (element widths
   saturate at PACKED_WIDTH_OVER). */
#define BARCODE_QUIET_MS            (500u)
#define BARCODE_MIN_WIDE_RATIO      (1.6f)    /* wide/narrow sanity check   */
/* No default letter per command: none turns the car until map= sets it
   (dashboard Demo barcode letters), and every reset clears them. */
#define NAV_EXECUTE_AT_JUNCTION     (1)       /* 0 = act immediately        */
#define NAV_PENDING_MAX_MM          (400.0f)  /* no junction => act anyway  */
/* Junctions seen by the barcode sensor: the junction line (18 mm) crosses */
/* under it; barcode bars are at most about 8 mm.  The turn starts once    */
/* the sensor is past the line.                                            */
#define NAV_JUNCTION_BAR_MIN_MM     (12.0f)   /* [CALIBRATE]                */
#define NAV_JUNCTION_BAR_MAX_MM     (30.0f)   /* [CALIBRATE]                */
#define NAV_LINE_JUNCTION           (0)       /* 1 = line junctions too   */
/* LEFT / RIGHT: creep NAV_TURN_AHEAD_MM, then pivot, both at             */
/* SPEED_CRUISE_MM_S (a 140 mm drive at 160 mm/s, then 120 mm/s, went past */
/* the line; no creep at all worked, 4 Nov).  Short of 90 deg; the line    */
/* search's first sweep finishes it on the line.                           */
#define NAV_TURN_AHEAD_MM           (60.0f)   /* [CALIBRATE] 4 Nov: was 45   */
#define NAV_TURN_DEG                (75.0f)   /* [CALIBRATE]                */
/* After a junction turn or a line search: follow at SPEED_SLOW_MM_S for    */
/* this distance so the steering lines the car up, then cruise.             */
#define LINE_SETTLE_MM              (150.0f)
#define LINE_SETTLE_KP              (18.0f)   /* steering (mm/s) meanwhile; 10 Nov: was 40, shook at every start (185006), now = LINE_KP */
#define LINE_SETTLE_MAX_MS          (1500u)   /* ...but never longer        */

/* ------------------------------------------------------------------------ */
/* IMU / terrain (Buddy 4)                                                  */
/* ------------------------------------------------------------------------ */
#define IMU_PERIOD_MS               (20u)
#define IMU_I2C_HZ                  (400000u)
#define IMU_FORWARD_SIGN            (1.0f)    /* +1 if chip X points ahead  */
#define IMU_CALIB_SAMPLES           (100u)
#define HUMP_PITCH_ON_DEG           (6.0f)
#define HUMP_PITCH_OFF_DEG          (3.0f)
#define HUMP_MIN_HEIGHT_MM          (5.0f)
/* DESCENDING once this far below the peak, CLIMBING again only at a new
   peak: roll noise moves the height about 1 mm either way.               */
#define HUMP_STATE_HYST_MM          (2.0f)
/* One-wheel humps (the barcode sensor leaves room for only one wheel):
   the car rolls instead of pitching; 3 deg is about 5 mm under a wheel. */
#define HUMP_ROLL_ON_DEG            (3.0f)
#define HUMP_ROLL_OFF_DEG           (1.5f)
/* [CALIBRATE] height = lever * tan(roll): 1 cm rolled 5.6 deg, 2 cm 11.5. */
#define HUMP_ROLL_LEVER_MM          (100.0f)
/* A 1 cm hump leans past the trigger for about 25 mm, a hand jolt 15. */
#define HUMP_ROLL_MIN_MM            (20.0f)   /* rolled this far: not a jolt */
/* Encoders and compass must both turn: straight pushes read up to 12 deg/s
   on the encoders and 9 on the compass, a hand turn 12-39 and 26-81.     */
#define TURN_RATE_EVENT_DPS         (10.0f)
/* Lifted or overturned (ends a run, 11 Nov requirement): pitch or roll past
   LIFT_TILT_DEG, or the vertical acceleration LIFT_AZ_MG away from 1 g (a
   lift straight up; upside down reads -1 g), for LIFT_CONFIRM_STEPS vehicle
   steps (20 ms each) in a row.  Driving reached 20 deg on humps and 26.5
   deg once (telemetry logs to 10 Nov).  [CALIBRATE] not yet tried on the
   car: the stop prints the pitch, roll and az that caused it. */
#define LIFT_TILT_DEG               (30.0f)
#define LIFT_AZ_MG                  (300.0f)
#define LIFT_CONFIRM_STEPS          (3u)
/* A level lift barely moves the accelerometer (11 Nov, 141046: tilts
   stopped the car, lifting it straight up did not).  In the air all three
   IR sensors read 1.0, as on black, and the wheels keep turning: no floor
   for LIFT_NO_FLOOR_MM of wheel travel is a lift too.  The only place all
   three are black on the track is a junction line, at most 30 mm. */
#define LIFT_NO_FLOOR_LEVEL         (0.9f)    /* line sensors above this   */
#define LIFT_NO_FLOOR_MM            (40.0f)   /* [CALIBRATE] 0.5 s at 75   */
#define ACCEL_EVENT_MM_S2           (250.0f)

/* ------------------------------------------------------------------------ */
/* Ultrasonic + servo (Buddy 5)                                             */
/* ------------------------------------------------------------------------ */
#define SERVO_MIN_US                (500u)    /* pulse at 0 deg [CALIBRATE] */
#define SERVO_MAX_US                (2500u)   /* pulse at 180 deg           */
#define SERVO_CENTRE_DEG            (90)      /* straight ahead             */
/* [CALIBRATE] Pulse trim: at 1500 us the sensor looks 2.5 deg left, so a
   box on the right was seen 40 mm later than one on the left (31 Oct). */
#define SERVO_TRIM_US               (-28)
/* [CALIBRATE] The mount stopped the sensor at about 60 and 120 deg (bench,
   22 Oct); fixed on 3 Nov to turn 45 deg each way, scanned 40 each way
   since 6 Nov (Kent's choice: a 20 mm-off 20 cm box at 200 mm still shows
   both edges).  Never command past these, so the servo cannot push on it. */
#define SERVO_MIN_DEG               (50)      /* furthest right             */
#define SERVO_MAX_DEG               (130)     /* furthest left              */
/* [CALIBRATE] 1 if servo 0 deg points the sensor left (mirrored mount).
   0: with 1 the sweep started on the car's right, not its left (seen on
   the car, 3 Nov), and a box 60 mm right read 32 mm left.                 */
#define SERVO_INVERT                (0)
#define SERVO_SETTLE_MS             (120u)
#define SERVO_STEP_DEG              (5)       /* [CALIBRATE] per 20 ms step */
#define US_TIMEOUT_MS               (40u)
#define US_SEM_TIMEOUT_MS           (100u)    /* NFR9: sensor wait limit    */
#define US_MIN_MM                   (20)
#define US_MAX_MM                   (4000)
/* Start profiling below this (31 Oct: at 200 scans found the box 5 of 9
   and 6 of 9, at 150 3 of 9).  detect= changes it until the next restart. */
#define OBST_DETECT_MM              (200)
/* The floor echoes at about 300-380 mm (the sensor is low and cannot tilt
   up): the profile treats echoes this far or further as no echo.  The
   front sensor still watches the pass for real objects. */
#define OBST_FLOOR_MM               (280)     /* [CALIBRATE]               */
#define OBST_CONFIRM_READINGS       (2u)      /* close front pings in a row */
/* 4 Nov: 2 stopped the car on short dropouts while pivoting (194058,    */
/* 194455); 4 pings at ~70 ms: a dead sensor still stops in ~0.3 s.      */
#define OBST_FAULT_LIMIT            (4u)      /* FAULT pings in a row: stop */
#define OBST_STALE_MS               (300u)    /* no front reading: stop     */
#define OBST_TOO_CLOSE_MM           (90)      /* reverse below this         */
#define OBST_CLUSTER_DEPTH_MM       (150)
#define OBST_MARGIN_MM              (60.0f)
#define OBST_MAX_CLEARANCE_MM       (1000)
#define OBST_FINE_STEP_DEG          (5)
#define OBST_DEPTH_MAX_MM           (150.0f)  /* [CALIBRATE] deepest box  */
#define OBST_MAX_ATTEMPTS           (2u)
/* Pass rules only: the width and its estimate are never limited to these. */
/* Box size assumed for a blind pass (no edge seen): the widest expected,  */
/* 10-20 cm, up to 60 mm off the line (28 Oct).                            */
#define OBST_BLIND_WIDTH_MM         (200.0f)
/* Seen narrower than this: the scan may have missed part of the box (a
   centred 150 mm box read 94 mm, one side only, 28 Oct), so its edges are
   not trusted and the car passes wide by OBST_BLIND_OFFSET_MM. */
#define OBST_PARTIAL_SCAN_MM        (100.0f)
/* Neither edge seen (a flat face echoes at every servo angle, S5-U07):
   pass this far to one side, enough for the widest obstacle on the line
   (100 + corridor 140 = 240).  Blind and near-centre passes go to
   OBST_BLIND_LEFT's side.  Ceiling: the widest box 60 mm off the line on
   the side passed leaves no gap. */
#define OBST_BLIND_OFFSET_MM                                            \
    ((OBST_BLIND_WIDTH_MM / 2.0f) + (CAR_WIDTH_MM / 2.0f) + OBST_MARGIN_MM)
/* An edge counts as seen only with this many scan points past it.  6 Nov:
   1 (was 2).  On the 40 deg sweep the cone still echoes off a box's corner
   35 deg out, on the second-last point, so a 10 cm box centred read cl=0
   (140935).  Ceiling: one dropout on the end point now counts as free. */
#define OBST_EDGE_POINTS            (1u)
/* Side vote: echoes of the box right of straight ahead minus those left of
   it.  This many or more: the box is on that side, pass on the other,
   whatever the edges say (a 10-20 cm box at 200 mm echoes out to the
   sweep's ends, so "edge seen" is often one stray point; the six 4 Nov
   bypass scans vote -4..+3 with the right sign each time).  Fewer: the
   edge rules below decide.  A large value turns the vote off. */
#define OBST_LEAN_POINTS            (2)
/* Sonar cone: each seen edge is pulled in by closest x tan(beam).  0: a
   flat face only echoes near square-on, so the seen box can sit 60-80 mm
   from the real one (25 Oct); 8 deg cut a pass to 53 mm.  Keep the edges
   as seen and the cone as margin. */
#define OBST_BEAM_HALF_DEG          (0.0f)    /* [CALIBRATE]               */
/* Width estimate only (est= in the scan lines), not the pass: the same
   pull-in at this cone.  6 was fitted to a 230 mm box read 286 mm wide,
   but a far echo inside the box set that edge; with only valid echoes it
   reads 231 (est 191), and a 150 mm box reads 142 at 148 mm (est 111) and
   166-200 at 211 mm (est 121-156) (3 Nov, 45 deg sweep).  Not refitted:
   re-run 150/200/250 first. */
#define OBST_WIDTH_BEAM_DEG         (6.0f)    /* [CALIBRATE]               */
/* Seen centre this close to the car's centre line: the side cannot be
   trusted (a box 60 mm left read 10 mm right, or 20 mm left), so pass
   wide by OBST_BLIND_OFFSET_MM on OBST_BLIND_LEFT's side.  40 sent a box
   well right of the line (read +30 mm, 4 Nov 190538) right, into it; a
   centred box read -11..+23 and clears on either side.  0 (6 Nov, Kent):
   always pass away from the side the box reads on, even -9 mm. */
#define OBST_CENTRE_UNSURE_MM       (0.0f)    /* [CALIBRATE] 6 Nov: off; was 20, 40 */
/* Smallest sideways shift of a side pass: the seen edge can be 60-80 mm
   off, so a pass sized to it alone could clip the box (53 mm, 25 Oct). */
#define OBST_MIN_OFFSET_MM          (240.0f)  /* [CALIBRATE] 11 Nov: was 150; the side of a 90 mm box cannot be told (141912), 240 clears it on either side */
/* Side of a blind pass and of any box within OBST_CENTRE_UNSURE_MM of the */
/* line: 1 = left (4 Nov, Kent's choice; was right).                      */
#define OBST_BLIND_LEFT             (1)       /* 0: pass on the right      */
/* Diagonal bypass: the car's inner side on the 45 deg legs keeps this    */
/* clearance from the box's near and far corners (deepest box,            */
/* OBST_DEPTH_MAX_MM); the car backs off first if the box is too close.   */
#define OBST_CORNER_MARGIN_MM       (15.0f)   /* [CALIBRATE] was 30        */
#define OBST_SENSOR_AHEAD_MM        (70.0f)   /* sonar face to axle, measured */
/* TEST_OBSTACLE build: MISSION with line only after the obstacle.  The   */
/* car drives straight (no steering, line search or barcode turns) until   */
/* it has bypassed an obstacle, then rejoins the line and follows it.      */
#define OBST_TEST_STRAIGHT          (APP_MODE == APP_MODE_TEST_OBSTACLE)

/* ------------------------------------------------------------------------ */
/* Vehicle controller (Buddy 5)                                             */
/* ------------------------------------------------------------------------ */
#define WATCHDOG_TIMEOUT_MS         (250u)    /* Week 6; 0 = off            */

/* ------------------------------------------------------------------------ */
/* Telemetry (Buddy 1)                                                      */
/* ------------------------------------------------------------------------ */
#define TELEMETRY_PERIOD_MS         (200u)    /* 5 Hz over MQTT             */
#define TELEMETRY_CONSOLE_PERIOD_MS (1000u)   /* 1 Hz when printing to USB  */
#define HEARTBEAT_PERIOD_MS         (1000u)
#define COMMAND_POLL_MS             (20u)     /* console/MQTT command check */
#define EVENT_CHECK_MS              (100u)

#endif /* CAR_CONFIG_H */

/*** end of file ***/
