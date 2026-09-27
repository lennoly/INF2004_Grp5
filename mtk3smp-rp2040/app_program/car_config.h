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

/* IMU GY-511 (LSM303DLHC) on I2C0 - Grove port 3.                          */
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
#define WHEEL_DIAMETER_MM           (65.0f)
#define ENCODER_SLOTS               (20u)    /* slots on the encoder disc   */
#define TRACK_WIDTH_MM              (130.0f) /* wheel centre to wheel centre*/
#define CAR_WIDTH_MM                (160.0f)
#define CAR_LENGTH_MM               (200.0f)
#define LINE_SENSOR_AHEAD_MM        (70.0f)  /* IR row ahead of the axle    */

/* Set to 1 if a motor spins backwards for a "forward" command.             */
#define MOTOR_L_INVERT              (0)
#define MOTOR_R_INVERT              (0)

/* ------------------------------------------------------------------------ */
/* Motion control (Buddy 2)                                                 */
/* ------------------------------------------------------------------------ */
#define MOTOR_PWM_FREQ_HZ           (10000u)  /* Robo Pico max is 20 kHz    */
#define CONTROL_PERIOD_MS           (20u)     /* 50 Hz control loop         */
#define ENCODER_GLITCH_US           (300u)    /* ignore faster edges        */
#define ENCODER_TIMEOUT_US          (200000u) /* no edge => wheel stopped   */

#define SPEED_KP                    (0.08f)   /* [CALIBRATE] %duty/(mm/s)   */
#define SPEED_KI                    (0.60f)
#define SPEED_KD                    (0.0f)
#define SPEED_KF                    (0.10f)   /* feed-forward %duty/(mm/s)  */
#define SPEED_OFFSET_PCT            (18.0f)   /* static friction offset     */
#define STRAIGHT_KP                 (2.0f)    /* (mm/s) per mm L-R mismatch */

#define SPEED_CRUISE_MM_S           (180.0f)
#define SPEED_SLOW_MM_S             (100.0f)
#define SPEED_TURN_MM_S             (120.0f)
#define MOTION_BRAKE_MARGIN_MM      (4.0f)

/* ------------------------------------------------------------------------ */
/* Line following + barcode (Buddy 3)                                       */
/* ------------------------------------------------------------------------ */
#define IR_SAMPLE_PERIOD_US         (1000u)   /* 1 kHz sampler ISR          */
#define IR_BLACK_IS_HIGH            (1)       /* A0 rises over black        */
#define IR_DEFAULT_WHITE            (400u)    /* [CALIBRATE] raw ADC counts */
#define IR_DEFAULT_BLACK            (3000u)   /* [CALIBRATE]                */

#define LINE_KP                     (160.0f)  /* (mm/s) per unit error      */
#define LINE_KI                     (0.0f)
#define LINE_KD                     (12.0f)
#define LINE_LOST_LEVEL             (0.15f)   /* both below => off line     */
#define LINE_JUNCTION_LEVEL         (0.75f)   /* both above => junction     */
#define LINE_JUNCTION_MIN_MM        (10.0f)
#define LINE_LOST_CONFIRM_MS        (150u)

#define BARCODE_BLACK_LEVEL         (0.60f)   /* hysteresis thresholds      */
#define BARCODE_WHITE_LEVEL         (0.40f)
#define BARCODE_QUIET_MS            (250u)    /* white gap ending a frame   */
#define BARCODE_MIN_WIDE_RATIO      (1.6f)    /* wide/narrow sanity check   */
#define NAV_EXECUTE_AT_JUNCTION     (1)       /* 0 = act immediately        */
#define NAV_PENDING_MAX_MM          (400.0f)  /* no junction => act anyway  */

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
#define IMPACT_JERK_MG              (700.0f)  /* sample-to-sample change    */
#define IMPACT_HOLDOFF_MS           (500u)
#define TURN_RATE_EVENT_DPS         (40.0f)
#define ACCEL_EVENT_MM_S2           (250.0f)

/* ------------------------------------------------------------------------ */
/* Ultrasonic + servo (Buddy 5)                                             */
/* ------------------------------------------------------------------------ */
#define SERVO_MIN_US                (500u)    /* pulse at 0 deg [CALIBRATE] */
#define SERVO_MAX_US                (2500u)   /* pulse at 180 deg           */
#define SERVO_CENTRE_DEG            (90)      /* straight ahead             */
#define SERVO_SETTLE_MS             (120u)
#define US_TIMEOUT_MS               (40u)
#define US_MIN_MM                   (20)
#define US_MAX_MM                   (4000)
#define OBST_DETECT_MM              (300)     /* start profiling below this */
#define OBST_TOO_CLOSE_MM           (90)      /* reverse below this         */
#define OBST_CLUSTER_DEPTH_MM       (150)
#define OBST_MARGIN_MM              (60.0f)
#define OBST_MAX_CLEARANCE_MM       (1000)
#define OBST_FINE_STEP_DEG          (5)
#define OBST_FINE_PAD_DEG           (20)
#define OBST_SIDE_SERVO_DEG         (15)      /* 15 = right, 165 = left     */
#define OBST_PASS_MAX_MM            (600.0f)
#define OBST_MAX_ATTEMPTS           (2u)

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
