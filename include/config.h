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
#define ROBOT_OBSTACLE_PERIOD_MS     30U
#define ROBOT_BASE_SPEED             0.35f
#define ROBOT_MAX_SPEED              0.70f

/* Default pin map is an integration starting point; verify against the actual Robo Pico carrier. */
#define PIN_MOTOR_LEFT_PWM           16U
#define PIN_MOTOR_RIGHT_PWM          17U
#define PIN_MOTOR_LEFT_DIR           18U
#define PIN_MOTOR_RIGHT_DIR          19U
#define PIN_ENCODER_LEFT_A           20U
#define PIN_ENCODER_LEFT_B           21U
#define PIN_ENCODER_RIGHT_A          22U
#define PIN_ENCODER_RIGHT_B          23U
#define PIN_IR_LEFT                  6U
#define PIN_IR_CENTER                7U
#define PIN_IR_BARCODE               8U
#define PIN_I2C_SDA                  4U
#define PIN_I2C_SCL                  5U
#define PIN_ULTRASONIC_TRIGGER       14U
#define PIN_ULTRASONIC_ECHO          15U
#define PIN_SCAN_SERVO               13U

#define I2C_PORT                     i2c0
#define IMU_I2C_ADDRESS              0x68U
#define IMU_SAMPLE_RATE_HZ           100U
#define IMU_GRAVITY_M_S2             9.80665f
#define IMU_ACCEL_COUNTS_PER_G       16384.0f
#define IMU_GYRO_COUNTS_PER_DPS      131.0f
#define IMU_SHOCK_THRESHOLD_G        2.2f
#define IMU_HUMP_THRESHOLD_G         0.25f

#define MOTOR_ENCODER_TICKS_PER_REV  360.0f
#define MOTOR_WHEEL_DIAMETER_M       0.065f
#define MOTOR_PID_KP                 0.55f
#define MOTOR_PID_KI                 0.08f
#define MOTOR_PID_KD                 0.00f

#define OBSTACLE_TRIGGER_CM          35.0f
#define ULTRASONIC_MAX_CM            250.0f
#define SERVO_MIN_DEG                25U
#define SERVO_CENTER_DEG             90U
#define SERVO_MAX_DEG                155U

#endif