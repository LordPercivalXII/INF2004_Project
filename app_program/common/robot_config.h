/** @file robot_config.h
 *
 * @brief Robot-wide configuration: pins, periods and tuning constants.
 *        Port of include/config.h from the FreeRTOS project.
 *
 * Every pin here is claimed in TEAM.md section 2.  Timing notes for this
 * kernel: the system tick is CNF_TIMER_PERIOD = 10 ms (FreeRTOS ran at
 * 1 ms), so every task period and timeout below is a multiple of 10 ms or
 * is rounded up to one.  Anything faster (1 kHz IR sampling, barcode edge
 * timestamps, ultrasonic echo width) runs in an interrupt handler timed by
 * the RP2040 TIMER in microseconds.
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Set by "make SIM=0|1" (build_make/pico_rp2040.mk); 1 if not given. */
#ifndef ROBOT_SIMULATION
#define ROBOT_SIMULATION                (1)
#endif

/* ---- Message buffer depths (were FreeRTOS queue lengths) ---- */
#define ROBOT_QUEUE_LENGTH              (12U)
#define ROBOT_EVENT_QUEUE_LENGTH        (24U)

/* ---- Task periods, ms ---- */
#define ROBOT_TELEMETRY_PERIOD_MS       (1000U)
#define ROBOT_LINE_PERIOD_MS            (10U)
#define ROBOT_MOTION_PERIOD_MS          (20U)
#define ROBOT_IMU_PERIOD_MS             (10U)
#define ROBOT_CONTROLLER_WAIT_MS        (10U)   /* was 2 ms at 1 kHz tick */

#ifndef ROBOT_BASE_SPEED
#define ROBOT_BASE_SPEED                (0.05f)
#endif
#define ROBOT_MAX_SPEED                 (0.70f)
#define ROBOT_MIN_SPEED                 (0.05f)

/* ---- Cytron Robo Pico pin map (Group 19 Design Review) ----
 * Measured on the car: motor channel GP10/GP11 drives the physical left
 * wheel and GP8/GP9 the physical right wheel, so they are swapped here. */
#define PIN_MOTOR_LEFT_PWM              (10U)
#define PIN_MOTOR_LEFT_DIR              (11U)
#define PIN_MOTOR_RIGHT_PWM             (8U)
#define PIN_MOTOR_RIGHT_DIR             (9U)
#define PIN_ENCODER_LEFT_A              (12U)
#define PIN_ENCODER_LEFT_B              (13U)
#define PIN_ENCODER_RIGHT_A             (14U)
#define PIN_ENCODER_RIGHT_B             (19U)
#define PIN_IR_LEFT                     (26U)
#define PIN_IR_CENTER                   (27U)
#define PIN_IR_BARCODE                  (28U)
#define PIN_I2C_SDA                     (4U)    /* set in i2c_cnf_sysdep.h */
#define PIN_I2C_SCL                     (5U)    /* set in i2c_cnf_sysdep.h */
#define PIN_IMU_INTERRUPT               (2U)    /* reserved, not used yet */
#define PIN_ULTRASONIC_TRIGGER          (17U)
#define PIN_ULTRASONIC_ECHO             (16U)
#define PIN_SCAN_SERVO                  (15U)

/* ---- Buddy 5: ultrasonic scanner ---- */
#define ULTRASONIC_THRESHOLD_CM         (30.0f)
#define ULTRASONIC_MIN_DISTANCE_CM      (2.0f)
#define ULTRASONIC_MAX_DISTANCE_CM      (400.0f)
#define ULTRASONIC_ECHO_TIMEOUT_MS      (30U)
#define ULTRASONIC_MIN_PING_INTERVAL_MS (66U)
#define ULTRASONIC_ECHO_QUEUE_LENGTH    (4U)
#define ULTRASONIC_SERVO_SETTLE_MS      (50U)
#define ULTRASONIC_COARSE_LEFT_DEG      (60U)   /* 30 deg left of centre: cabling limit */
#define ULTRASONIC_COARSE_CENTER_DEG    (90U)
#define ULTRASONIC_COARSE_RIGHT_DEG     (120U)  /* 30 deg right of centre: cabling limit */
#define ULTRASONIC_FINE_STEP_DEG        (5U)
#define ULTRASONIC_FINE_MARGIN_DEG      (15U)   /* up to the next coarse angle */
#define ULTRASONIC_BEAM_WIDTH_DEG       (15U)

/* ---- Buddy 4: LSM303DLHC on I2C0 through kernel "iica" ---- */
#define IMU_I2C_DEVICE_NAME             "iica"
#define IMU_ACCEL_ADDRESS               (0x19U)
#define IMU_MAG_ADDRESS                 (0x1EU)
#define IMU_SAMPLE_RATE_HZ              (100U)
#define IMU_GRAVITY_M_S2                (9.80665f)
#define IMU_SHOCK_THRESHOLD_G           (3.0f)      /* FR10                 */
#define IMU_HUMP_THRESHOLD_G            (0.08f)     /* world vertical       */
#define IMU_FILTER_WINDOW_SIZE          (5U)        /* moving average taps  */
/* Temporary terrain commissioning trace, up to 50 Hz while moving.
 * Printed by telemetry_task, never by the sensor/motion event producers. */
#ifndef IMU_HEIGHT_TRACE_ENABLED
#define IMU_HEIGHT_TRACE_ENABLED        (0)
#endif

/* Serial console: 1 = one readable status line per second instead of the
 * long JSON record (MQTT still gets JSON); S1_SERIAL_DIAG = 1 brings back
 * the heartbeat and timing_diag lines. */
#ifndef S1_SERIAL_COMPACT
#define S1_SERIAL_COMPACT               (1)
#endif
#ifndef S1_SERIAL_DIAG
#define S1_SERIAL_DIAG                  (0)
#endif

/* ---- Buddy 2: motors and encoders ---- */
#define MOTOR_ENCODER_TICKS_PER_REV     (630.0f)
#define MOTOR_WHEEL_DIAMETER_M          (0.066f)
/* 100 us still permits 3.29 m/s with the geometry above, well above the
 * 0.70 m/s drive limit. Suppress faster duplicate electrical edges. */
#define MOTOR_ENCODER_MIN_EDGE_US       (100U)
#define MOTOR_TRACK_WIDTH_M             (0.125f)  /* Measured wheel-center spacing: 12.5 cm */
/* 1 = motors off and raw encoder ticks printed (build TEST=1 SIM=0). */
#ifndef S2_ENCODER_CALIBRATION
#define S2_ENCODER_CALIBRATION          (0)
#endif
/* With calibration on: 1 = also spin each wheel alone to check direction. */
#ifndef S2_MOTOR_DIR_TEST
#define S2_MOTOR_DIR_TEST               (0)
#endif
/* Motor direction: 1.0 = normal, -1.0 = wired/mounted the other way round. */
#define MOTOR_LEFT_DIR_SIGN             (1.0f)
#define MOTOR_RIGHT_DIR_SIGN            (-1.0f)
/* Cap on motor PWM duty (0..1); lower = slower wheels. */
#define MOTOR_MAX_DUTY                  (0.70f)
/* Smallest duty that still overcomes motor friction (0 = disabled). */
#define MOTOR_MIN_DUTY                  (0.0f)
/* TEST ONLY.  Milliseconds after power-up at which a SIM=0 car starts
 * following the line by itself, as if it had received CONTROL_START;
 * 0 = off.  Nothing else sends START yet.  Set back to 0 before
 * committing: with it on, the car drives off whenever it is powered. */
#ifndef CTRL_AUTO_START_MS
#define CTRL_AUTO_START_MS              (0U)
#endif
/* 1 = after boot, run straight/turn bench drive and print results. */
#ifndef S2_BENCH_DRIVE
#define S2_BENCH_DRIVE                  (0)
#endif
#define MOTOR_ENCODER_TIMEOUT_MS        (500U)
#define MOTOR_PID_KP                    (0.55f)
#define MOTOR_PID_KI                    (0.08f)
#define MOTOR_PID_KD                    (0.00f)
#define MOTOR_TURN_SPEED_M_S            (0.18f)
#define MOTOR_TURN_MIN_SPEED_M_S        (0.07f)
#define MOTOR_TURN_SLOWDOWN_DEG         (25.0f)

/* ---- Buddy 3: line tracking (FR7, NFR6) ---- */
#define IR_RING_SIZE                    (64U)    /* power of two          */
#define ADC_CHANNEL_IR_LEFT             (0U)     /* GP26 = ADC0           */
#define ADC_CHANNEL_IR_CENTER           (1U)     /* GP27 = ADC1           */
#define IR_SAMPLE_RATE_HZ               (1000U)  /* TIMER alarm interrupt */
#define IR_ANALOG_BLACK_IS_HIGH         (1)      /* verify on hardware    */
#define IR_CALIBRATION_MS               (1000U)
#define IR_CALIBRATION_STEP_MS          (10U)    /* was 5 ms; 10 ms tick  */
#define IR_MIN_CONTRAST_COUNTS          (400U)
#define IR_DEFAULT_WHITE_COUNTS         (230U)
#define IR_DEFAULT_BLACK_COUNTS         (2200U)

/* Line position with TWO analog sensors, left (GP26) and middle (GP27),
 * each sitting over one EDGE of the ~18 mm line (sensors ~18-20 mm
 * apart).  Normalised readings (0 = white, 1 = black):
 *   centred  both grey (~0.5)  error 0, continuous for a few mm
 *   drifted  one up, one down  error -1..+1 (middle - left)
 *   lost     both white        steer towards the side last seen
 *   junction both black        a crossing line covers both sensors
 * The right sensor (GP28) is for barcodes only. */
#define LINE_LOST_LEVEL                 (0.06f)  /* both below: lost      */
#define LINE_JUNCTION_LEVEL             (0.80f)  /* both above: junction  */
#define LINE_JUNCTION_RELEASE           (0.65f)  /* re-arm when one below:
                                                    > 0.5 so a centred car
                                                    (L + M ~ 1) re-arms */
#define LINE_JUNCTION_MIN_CYCLES        (2U)     /* x 10 ms: an 18 mm
                                                    crossing at 0.6 m/s
                                                    gives ~3 cycles     */
#define LINE_ERROR_SIGN                 (1.0f)   /* -1.0f if the car steers
                                                    away from the line  */

/* Junction turns (controller.c).  A junction is detected when the LINE
 * SENSORS reach the crossing line, but the car pivots about its WHEEL
 * AXLE.  Before a left/right/U-turn it drives straight this far, so the
 * pivot lands on the centre of the crossing line:
 *     JUNCTION_APPROACH_M = (sensor-to-axle distance) + ~10 mm
 * Measure the sensor-to-axle distance along the driving direction on the
 * car (the sensors look close to the axle in the photos: ~0 mm). */
#define JUNCTION_APPROACH_M             (0.010f)
#define JUNCTION_APPROACH_SPEED         (0.20f)  /* m/s, slow = accurate  */
#define JUNCTION_APPROACH_TIMEOUT_MS    (2000U)  /* turn anyway if stuck  */

/* 1 = read the REAL line sensors (ADC, boot calibration) even in a
 * SIM=1 build, to check polarity and steering sign by hand while the
 * motors stay simulated.  Default: real sensors only in SIM=0 builds. */
#ifndef S3_LINE_HW
#define S3_LINE_HW                      (!ROBOT_SIMULATION)
#endif
/* 1 = print L, M (raw and %), error and state every 200 ms. */
#ifndef S3_LINE_DEBUG
#define S3_LINE_DEBUG                   (0)
#endif
#define LINE_DEBUG_PERIOD_MS            (100U)
#if (!ROBOT_SIMULATION) && (!S3_LINE_HW)
#error "SIM=0 needs the real line sensors (S3_LINE_HW = 1)"
#endif

/* ---- Buddy 3: Code 39 barcode (FR8, FR9, NFR5) ---- */
#define IR_BARCODE_BLACK_LEVEL          (1U)     /* verify on hardware    */
#define BARCODE_GLITCH_US               (500U)
#define BARCODE_END_GAP_MIN_MS          (150U)
#define BARCODE_END_GAP_FACTOR          (4U)
#define BARCODE_END_GAP_MAX_MS          (1000U)  /* cap: slow scans end */
#define BARCODE_EDGE_QUEUE_LENGTH       (512U)   /* absorbs comparator chatter */

/* ---- Buddy 3: junction routing from MQTT (subsystem3/route.c) ----
 * Which character means left/right/straight/U-turn is NOT set here: it
 * arrives at run time on MQTT_ROUTE_TOPIC (config/wifi_secrets.h). */
#define ROUTE_CONFIG_MAX_LEN            (128U)  /* = mqtt_utk RX buffer  */
#define ROUTE_CONFIG_QUEUE_LENGTH       (4U)
#define ROUTE_HISTORY_LENGTH            (8U)    /* decoded chars kept    */

/* 1 = read the REAL barcode sensor (GP28) and stop the scripted A..E
 * barcodes, even in a SIM=1 build (motors, IMU, ultrasonic and the line
 * stay simulated; a junction is simulated 3 s after each real read).
 * Default: real sensor only in SIM=0 builds. */
#ifndef S3_BARCODE_HW
#define S3_BARCODE_HW                   (1)
#endif
/* 1 = print every scan's bar/space widths and the decode result. */
/* 1 = print every bar/space width as it is read (very verbose). */
#ifndef S3_BARCODE_EDGE_DEBUG
#define S3_BARCODE_EDGE_DEBUG           (0)
#endif
#ifndef S3_BARCODE_DEBUG
#define S3_BARCODE_DEBUG                (1)
#endif
#if (!ROBOT_SIMULATION) && (!S3_BARCODE_HW)
#error "SIM=0 needs the real barcode sensor (S3_BARCODE_HW = 1)"
#endif

/* 1 = use the REAL ultrasonic sensor and scan servo even in a SIM=1
 * build, to bench-test the scanner: motors, IMU and the line stay
 * simulated, so the wheels never turn.
 * Default: real scanner only in SIM=0 builds. */
#ifndef S5_ULTRASONIC_HW
#define S5_ULTRASONIC_HW                (!ROBOT_SIMULATION)
#endif
#if (!ROBOT_SIMULATION) && (!S5_ULTRASONIC_HW)
#error "SIM=0 needs the real ultrasonic scanner (S5_ULTRASONIC_HW = 1)"
#endif

/* ---- Spare constants kept from the original configuration ---- */
#define OBSTACLE_TRIGGER_CM             (35.0f)
#define ULTRASONIC_MAX_CM               (250.0f)
#define SERVO_MIN_DEG                   (25U)
#define SERVO_CENTER_DEG                (90U)
#define SERVO_MAX_DEG                   (155U)

#endif /* ROBOT_CONFIG_H */

/*** end of file ***/
