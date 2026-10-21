#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

#ifndef ROBOT_SIMULATION
#define ROBOT_SIMULATION 1
#endif

#define ROBOT_TASK_STACK_WORDS       512U
#define ROBOT_QUEUE_LENGTH           12U
#define ROBOT_EVENT_QUEUE_LENGTH     24U
#define ROBOT_TELEMETRY_PERIOD_MS    1000U
#define ROBOT_LINE_PERIOD_MS         10U
#define ROBOT_MOTION_PERIOD_MS       20U
#define ROBOT_IMU_PERIOD_MS          10U
#define ROBOT_BASE_SPEED             0.35f
#define ROBOT_MAX_SPEED              0.70f

/* Cytron Robo Pico pin map from the Group 19 Design Review. */
#define PIN_MOTOR_LEFT_PWM           8U
#define PIN_MOTOR_RIGHT_PWM          10U
#define PIN_MOTOR_LEFT_DIR           9U
#define PIN_MOTOR_RIGHT_DIR          11U
#define PIN_ENCODER_LEFT_A           12U
#define PIN_ENCODER_LEFT_B           13U
#define PIN_ENCODER_RIGHT_A          14U
#define PIN_ENCODER_RIGHT_B          19U
#define PIN_IR_LEFT                  26U
#define PIN_IR_CENTER                27U
#define PIN_IR_BARCODE               28U
#define PIN_I2C_SDA                  4U
#define PIN_I2C_SCL                  5U
#define PIN_IMU_INTERRUPT            2U
#define PIN_ULTRASONIC_TRIGGER       17U
#define PIN_ULTRASONIC_ECHO          16U
#define PIN_SCAN_SERVO               15U

#define ULTRASONIC_THRESHOLD_CM      30.0f
#define ULTRASONIC_MIN_DISTANCE_CM   2.0f
#define ULTRASONIC_MAX_DISTANCE_CM   400.0f
#define ULTRASONIC_ECHO_TIMEOUT_MS   30U
#define ULTRASONIC_MIN_PING_INTERVAL_MS 66U
#define ULTRASONIC_ECHO_QUEUE_LENGTH 4U
#define ULTRASONIC_SERVO_SETTLE_MS   50U
#define ULTRASONIC_COARSE_LEFT_DEG   45U
#define ULTRASONIC_COARSE_CENTER_DEG 90U
#define ULTRASONIC_COARSE_RIGHT_DEG  135U
#define ULTRASONIC_FINE_STEP_DEG     5U
#define ULTRASONIC_FINE_MARGIN_DEG   20U
#define ULTRASONIC_BEAM_WIDTH_DEG    15U

#define I2C_PORT                     i2c0
#define IMU_I2C_ADDRESS              0x68U
#define IMU_SAMPLE_RATE_HZ           100U
#define IMU_GRAVITY_M_S2             9.80665f
#define IMU_ACCEL_COUNTS_PER_G       8192.0f      /* ±4g full-scale range (AFS_SEL=1) */
#define IMU_GYRO_COUNTS_PER_DPS      65.5f        /* ±1000 DPS full-scale range (FS_SEL=1) */
#define IMU_SHOCK_THRESHOLD_G        3.0f         /* FR10: Detect shocks ±3g */
#define IMU_HUMP_THRESHOLD_G         0.25f        /* FR11: Threshold for bump detection */
#define IMU_FILTER_WINDOW_SIZE       5U           /* 5-tap moving average filter */

#define MOTOR_ENCODER_TICKS_PER_REV  360.0f
#define MOTOR_WHEEL_DIAMETER_M       0.065f
#define MOTOR_PID_KP                 0.55f
#define MOTOR_PID_KI                 0.08f
#define MOTOR_PID_KD                 0.00f

/* ---- Buddy 3: line tracking (FR7, NFR6) ---- */
#define IR_RING_SIZE                64U   /* power-of-two ISR-to-task sample buffer */
#define ADC_CHANNEL_IR_LEFT         0U    /* GP26 = ADC0 */
#define ADC_CHANNEL_IR_CENTER       1U    /* GP27 = ADC1 */
#define IR_SAMPLE_RATE_HZ           1000U /* Buddy 3 IR sampling timer */
#define IR_ANALOG_BLACK_IS_HIGH      1     /* AO voltage rises over black on most TCRT5000 modules: verify */
#define IR_CALIBRATION_MS            3000U /* sweep sensors across line and floor during this window */
#define IR_MIN_CONTRAST_COUNTS       400U  /* 12-bit ADC; below this calibration is rejected */
#define IR_DEFAULT_WHITE_COUNTS      600U  /* fallback if calibration fails */
#define IR_DEFAULT_BLACK_COUNTS      3000U
#define IR_ON_LINE_THRESHOLD         0.5f  /* normalised 0 = white, 1 = black */
#define JUNCTION_MIN_MS              150U  /* all three sensors black this long = junction */

/* ---- Buddy 3: Code 39 barcode (FR8, FR9, NFR5) ---- */
#define IR_BARCODE_BLACK_LEVEL       1U    /* gpio level on black bar (DO high on black): verify */
#define BARCODE_GLITCH_US            500U  /* debounce: pulses shorter than this are noise */
#define BARCODE_END_GAP_MIN_MS       150U  /* quiet time that ends a barcode read */
#define BARCODE_END_GAP_FACTOR       4U    /* ...or 4x the widest element seen, if longer */
#define BARCODE_EDGE_QUEUE_LENGTH    128U
#define BARCODE_LEFT_CHAR            'L'   /* set these to the track's barcode convention */
#define BARCODE_RIGHT_CHAR           'R'

#define OBSTACLE_TRIGGER_CM          35.0f
#define ULTRASONIC_MAX_CM            250.0f
#define SERVO_MIN_DEG                25U
#define SERVO_CENTER_DEG             90U
#define SERVO_MAX_DEG                155U

#endif