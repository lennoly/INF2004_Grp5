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
#define APP_MODE_TEST_CRASH         (9)  /* Buddy 4: drive into a wall     */
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

#define SPEED_CRUISE_MM_S           (50.0f)   /* [CALIBRATE] 27 Oct: bends */
#define SPEED_HUMP_UP_MM_S          (100.0f)  /* [CALIBRATE] climbing     */
#define SPEED_HUMP_DOWN_MM_S        (75.0f)   /* [CALIBRATE] descending    */
#define SPEED_SLOW_MM_S             (160.0f)  /* 100 stick-slipped (floor) */
#define SPEED_TURN_MM_S             (120.0f)
#define SPEED_BYPASS_MM_S           (300.0f)  /* [CALIBRATE] NFR2 max     */
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

#define LINE_KP                     (45.0f)   /* (mm/s) per unit error      */
#define LINE_KI                     (0.0f)
#define LINE_KD                     (0.0f)
#define LINE_LOST_LEVEL             (0.15f)   /* both below => off line     */
#define LINE_JUNCTION_LEVEL         (0.75f)   /* both above => junction     */
#define LINE_JUNCTION_MIN_MM        (10.0f)
#define LINE_JUNCTION_REARM_MM      (100.0f)  /* [CALIBRATE] off black, rearm */
#define LINE_LOST_CONFIRM_MS        (150u)

/* 14 cm barcode (S3-U03): narrow bars peak at 190-810 permille driving at
   100 mm/s, white paper reads 0-21. */
#define BARCODE_BLACK_LEVEL         (0.15f)   /* [CALIBRATE] hysteresis     */
#define BARCODE_WHITE_LEVEL         (0.08f)
#define BARCODE_QUIET_MS            (250u)    /* white gap ending a frame   */
#define BARCODE_MIN_WIDE_RATIO      (1.6f)    /* wide/narrow sanity check   */
/* No letter is built in: the PC sends the four with map= before START,
   and START is refused until all four are set and differ. */
#define NAV_EXECUTE_AT_JUNCTION     (1)       /* 0 = act immediately        */
#define NAV_PENDING_MAX_MM          (400.0f)  /* no junction => drop it     */
/* Junctions seen by the barcode sensor: the junction line (18 mm) crosses */
/* under it; barcode bars are at most about 8 mm.  The turn starts once    */
/* the sensor is past the line.                                            */
#define NAV_JUNCTION_BAR_MIN_MM     (12.0f)   /* [CALIBRATE]                */
#define NAV_JUNCTION_BAR_MAX_MM     (30.0f)   /* [CALIBRATE]                */
#define NAV_LINE_JUNCTION           (0)       /* 1 = line junctions too   */
/* After a junction turn or a line search: follow at SPEED_SLOW_MM_S for    */
/* this distance so the steering lines the car up, then cruise.             */
#define LINE_SETTLE_MM              (150.0f)
#define LINE_SETTLE_KP              (40.0f)   /* steering (mm/s) meanwhile  */
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
/* [CALIBRATE] Driving, only the jolt against the direction of travel
   counts (TEST_CRASH): by size alone, wall hits (1014-2885 mg) overlapped
   motor starts (up to 1453 forward, 1797 reversing).  Parked: any jolt.  */
#define IMPACT_JERK_MG              (800.0f)  /* sample-to-sample change    */
#define IMPACT_DRIVE_PCT            (5.0f)    /* duty above this: driving   */
/* Braking (up to 1011 mg) and the rock after a reverse start (1074) also
   jolt against travel: ignore jolts just after the car's own duty jumps. */
#define IMPACT_DUTY_STEP_PCT        (15.0f)   /* mean duty change per step  */
#define IMPACT_BLANK_MS             (300u)    /* then ignore jolts this long*/
#define IMPACT_HOLDOFF_MS           (500u)
/* Encoders and compass must both turn: straight pushes read up to 12 deg/s
   on the encoders and 9 on the compass, a hand turn 12-39 and 26-81.     */
#define TURN_RATE_EVENT_DPS         (10.0f)
#define ACCEL_EVENT_MM_S2           (250.0f)

/* ------------------------------------------------------------------------ */
/* Ultrasonic + servo (Buddy 5)                                             */
/* ------------------------------------------------------------------------ */
#define SERVO_MIN_US                (500u)    /* pulse at 0 deg [CALIBRATE] */
#define SERVO_MAX_US                (2500u)   /* pulse at 180 deg           */
#define SERVO_CENTRE_DEG            (90)      /* straight ahead             */
/* [CALIBRATE] The mount stops the sensor at about 60 and 120 deg (bench,
   22 Oct): never command past these, so the servo cannot push on it.      */
#define SERVO_MIN_DEG               (65)      /* furthest right             */
#define SERVO_MAX_DEG               (115)     /* furthest left              */
/* [CALIBRATE] 1 if servo 0 deg points the sensor left (mirrored mount)     */
#define SERVO_INVERT                (0)
#define SERVO_SETTLE_MS             (120u)
#define SERVO_STEP_DEG              (5)       /* [CALIBRATE] per 20 ms step */
#define US_TIMEOUT_MS               (40u)
#define US_SEM_TIMEOUT_MS           (100u)    /* NFR9: sensor wait limit    */
#define US_MIN_MM                   (20)
#define US_MAX_MM                   (4000)
/* Start profiling below this (25 Oct: 200 was too close to see the box
   edges; 30 Oct: 150 asked for, the +/-25 deg scan then covers +/-70 mm). */
#define OBST_DETECT_MM              (150)
/* The floor echoes at about 300-380 mm (the sensor is low and cannot tilt
   up): the profile treats echoes this far or further as no echo.  The
   front sensor still watches the pass for real objects. */
#define OBST_FLOOR_MM               (280)     /* [CALIBRATE]               */
/* 1: an IMU impact also starts the obstacle routine.  0 while tuning line   */
/* following: impacts are still counted and reported, but do not stop.     */
#define OBST_ON_IMPACT              (1)
#define OBST_CONFIRM_READINGS       (2u)      /* close front pings in a row */
#define OBST_FAULT_LIMIT            (2u)      /* FAULT pings in a row: stop */
#define OBST_STALE_MS               (300u)    /* no front reading: stop     */
#define OBST_TOO_CLOSE_MM           (90)      /* reverse below this         */
#define OBST_CLUSTER_DEPTH_MM       (150)
#define OBST_MARGIN_MM              (60.0f)
#define OBST_MAX_CLEARANCE_MM       (1000)
#define OBST_FINE_STEP_DEG          (5)
#define OBST_DEPTH_MAX_MM           (150.0f)  /* [CALIBRATE] deepest box  */
#define OBST_MAX_ATTEMPTS           (2u)
/* Obstacles are 10-20 cm wide, up to 60 mm off the line (28 Oct). */
#define OBST_WIDTH_MAX_MM           (200.0f)
/* Seen narrower than this: the scan missed part of the box (a centred
   150 mm box read 94 mm, one side only, 28 Oct), so its edges are not
   trusted and the car passes wide by OBST_BLIND_OFFSET_MM. */
#define OBST_WIDTH_MIN_MM           (100.0f)
#define OBST_LINE_OFFSET_MAX_MM     (60.0f)
/* Neither edge seen (a flat face echoes at every servo angle, S5-U07):
   pass this far to one side, enough for the widest obstacle off the line
   the other way (100 + 60 + corridor 140 = 300). */
#define OBST_BLIND_OFFSET_MM                                            \
    ((OBST_WIDTH_MAX_MM / 2.0f) + OBST_LINE_OFFSET_MAX_MM               \
     + (CAR_WIDTH_MM / 2.0f) + OBST_MARGIN_MM)
/* An edge counts as seen only with this many scan points past it, so a
   single dropout does not make one. */
#define OBST_EDGE_POINTS            (2u)
/* Sonar cone: each seen edge is pulled in by closest x tan(beam).  0: a
   flat face only echoes near square-on, so the seen box can sit 60-80 mm
   from the real one (25 Oct); 8 deg cut a pass to 53 mm.  Keep the edges
   as seen and the cone as margin. */
#define OBST_BEAM_HALF_DEG          (0.0f)    /* [CALIBRATE]               */
/* Seen centre this close to the car's centre line: the side cannot be
   trusted (a box 60 mm left read 10 mm right, or 20 mm left), so pass
   wide by OBST_BLIND_OFFSET_MM, away from the way it leans. */
#define OBST_CENTRE_UNSURE_MM       (40.0f)   /* [CALIBRATE]               */
/* Smallest sideways shift of a side pass: the seen edge can be 60-80 mm
   off, so a pass sized to it alone could clip the box (53 mm, 25 Oct). */
#define OBST_MIN_OFFSET_MM          (150.0f)  /* [CALIBRATE]               */
#define OBST_BLIND_LEFT             (1)       /* 0: blind pass on the right*/
/* Diagonal bypass: the car's inner side on the 45 deg legs keeps this    */
/* clearance from the box's near and far corners (deepest box,            */
/* OBST_DEPTH_MAX_MM); the car backs off first if the box is too close.   */
#define OBST_CORNER_MARGIN_MM       (30.0f)   /* [CALIBRATE]               */
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
