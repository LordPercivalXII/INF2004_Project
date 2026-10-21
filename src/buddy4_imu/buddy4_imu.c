#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "common_types.h"
#include "interfaces/buddy4_imu.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* MPU6050 Register Definitions */
#define MPU6050_REG_PWR_MGMT_1       0x6BU     /* Power management */
#define MPU6050_REG_ACCEL_CONFIG     0x1CU     /* Accelerometer configuration */
#define MPU6050_REG_GYRO_CONFIG      0x1BU     /* Gyroscope configuration */
#define MPU6050_REG_SMPLRT_DIV       0x19U     /* Sample rate divider */
#define MPU6050_REG_WHO_AM_I         0x75U     /* Device ID (should be 0x68 or 0x69) */
#define MPU6050_REG_ACCEL_XOUT_H     0x3BU     /* Start of accelerometer data */

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gz;
} ImuRaw;

typedef struct {
    float samples[IMU_FILTER_WINDOW_SIZE];
    uint8_t index;
    uint8_t count;
} MovingAverageFilter;

static bool imu_write_register(uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = { reg, value };
    return i2c_write_blocking(I2C_PORT, IMU_I2C_ADDRESS, bytes, sizeof(bytes), false) == (int)sizeof(bytes);
}

static bool imu_read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_write_blocking(I2C_PORT, IMU_I2C_ADDRESS, &reg, 1U, true) == 1 &&
           i2c_read_blocking(I2C_PORT, IMU_I2C_ADDRESS, data, length, false) == (int)length;
}

static void filter_init(MovingAverageFilter *filter)
{
    filter->index = 0;
    filter->count = 0;
    for (uint8_t i = 0; i < IMU_FILTER_WINDOW_SIZE; i++) {
        filter->samples[i] = 0.0f;
    }
}

static float filter_update(MovingAverageFilter *filter, float value)
{
    filter->samples[filter->index] = value;
    filter->index = (filter->index + 1) % IMU_FILTER_WINDOW_SIZE;
    if (filter->count < IMU_FILTER_WINDOW_SIZE) {
        filter->count++;
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < filter->count; i++) {
        sum += filter->samples[i];
    }
    return sum / (float)filter->count;
}

static bool imu_init(void)
{
#if ROBOT_SIMULATION
    return true;
#else
    i2c_init(I2C_PORT, 400000U);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
    
    sleep_ms(100);
    
    uint8_t identity = 0U;
    if (!imu_read_registers(MPU6050_REG_WHO_AM_I, &identity, 1U) || 
        (identity != 0x68U && identity != 0x69U)) {
        return false;
    }
    
    /* Configure MPU6050:
     * - Wake up from sleep (PWR_MGMT_1 = 0x00)
     * - Set accelerometer to ±4g (AFS_SEL=1)
     * - Set gyroscope to ±1000 DPS (FS_SEL=1)
     * - Set sample rate divider for 100 Hz sampling
     */
    return imu_write_register(MPU6050_REG_PWR_MGMT_1, 0x00U) &&    /* Wake up */
           imu_write_register(MPU6050_REG_ACCEL_CONFIG, 0x08U) &&  /* ±4g */
           imu_write_register(MPU6050_REG_GYRO_CONFIG, 0x08U) &&   /* ±1000 DPS */
           imu_write_register(MPU6050_REG_SMPLRT_DIV, 0x09U);      /* ~100 Hz */
#endif
}

static bool imu_read(ImuRaw *raw, uint32_t sample)
{
#if ROBOT_SIMULATION
    raw->ax = 0;
    raw->ay = (int16_t)((sample % 40U) < 4U ? 16384 : 0);  /* Simulate ~2g spike */
    raw->az = (int16_t)IMU_ACCEL_COUNTS_PER_G;
    raw->gz = 0;
    return true;
#else
    uint8_t data[14];
    if (!imu_read_registers(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data))) {
        return false;
    }
    raw->ax = (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
    raw->ay = (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
    raw->az = (int16_t)(((uint16_t)data[4] << 8U) | data[5]);
    raw->gz = (int16_t)(((uint16_t)data[12] << 8U) | data[13]);
    return true;
#endif
}

/* Classify motion state from acceleration and gyro data */
static MotionClassification classify_motion(float ax_g, float ay_g, float az_g, float gz_dps)
{
    float total_accel = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    float abs_gz = fabsf(gz_dps);

    /* Priority order: Impact > Turning > Climbing/Descending > Accelerating > Stationary */
    
    /* MOTION_IMPACT: Sudden shock detection (|a_total| > 3.0g) */
    if (total_accel > 3.0f) {
        return MOTION_IMPACT;
    }
    
    /* MOTION_TURNING: High yaw rate indicates turning (|gz| > 20 DPS) */
    if (abs_gz > 20.0f) {
        return MOTION_TURNING;
    }
    
    /* MOTION_CLIMBING: Strong upward acceleration (az > 1.3g) */
    if (az_g > 1.3f) {
        return MOTION_CLIMBING;
    }
    
    /* MOTION_DESCENDING: Reduced vertical acceleration (az < 0.7g) indicates downward slope */
    if (az_g < 0.7f) {
        return MOTION_DESCENDING;
    }
    
    /* MOTION_ACCELERATING: Significant horizontal acceleration (|a_total| > 1.5g) */
    if (total_accel > 1.5f) {
        return MOTION_ACCELERATING;
    }
    
    /* MOTION_STATIONARY: Default when none of above conditions met */
    return MOTION_STATIONARY;
}

/* Calculate tilt angles from acceleration vector (assumes acceleration ≈ gravity at rest) */
static void calculate_tilt_angles(float ax_g, float ay_g, float az_g, 
                                  float *roll_deg, float *pitch_deg)
{
    /* Roll = atan2(ay, az) */
    float roll_rad = atan2f(ay_g, az_g);
    *roll_deg = roll_rad * 180.0f / M_PI;
    
    /* Pitch = atan2(-ax, sqrt(ay² + az²)) */
    float pitch_rad = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));
    *pitch_deg = pitch_rad * 180.0f / M_PI;
}

static void imu_task(void *argument)
{
    (void)argument;
    const bool initialized = imu_init();
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t sample = 0U;
    
    /* Bump height estimation */
    float vertical_velocity = 0.0f;
    float max_vertical_velocity = 0.0f;
    bool in_hump = false;
    
    /* Moving average filter for vertical acceleration */
    MovingAverageFilter az_filter;
    filter_init(&az_filter);
    
    for (;;) {
        ImuRaw raw = { 0 };
        const bool valid = initialized && imu_read(&raw, sample++);
        
        /* Convert raw counts to g units */
        const float ax_g = (float)raw.ax / IMU_ACCEL_COUNTS_PER_G;
        const float ay_g = (float)raw.ay / IMU_ACCEL_COUNTS_PER_G;
        const float az_g = (float)raw.az / IMU_ACCEL_COUNTS_PER_G;
        
        /* FR11: Remove gravity component from vertical axis (assumes flat terrain) */
        const float vertical_g = az_g - 1.0f;
        
        /* Apply 5-tap moving average filter to vertical acceleration */
        const float vertical_g_filtered = filter_update(&az_filter, vertical_g);
        
        const float dt = (float)ROBOT_IMU_PERIOD_MS / 1000.0f;
        
        /* FR11: Integrate vertical acceleration to estimate bump height (h = v²/2g) */
        if (valid && fabsf(vertical_g_filtered) > IMU_HUMP_THRESHOLD_G) {
            in_hump = true;
            /* Trapezoidal integration for velocity */
            vertical_velocity += vertical_g_filtered * IMU_GRAVITY_M_S2 * dt;
            /* Track maximum velocity magnitude */
            if (fabsf(vertical_velocity) > max_vertical_velocity) {
                max_vertical_velocity = fabsf(vertical_velocity);
            }
        }
        
        float estimated_height = 0.0f;
        /* Bump ends when acceleration returns to baseline */
        if (valid && in_hump && fabsf(vertical_g_filtered) <= IMU_HUMP_THRESHOLD_G) {
            /* h = v_max² / (2g) formula from FR11 */
            estimated_height = (max_vertical_velocity * max_vertical_velocity) / (2.0f * IMU_GRAVITY_M_S2);
            /* Reset for next bump */
            vertical_velocity = 0.0f;
            max_vertical_velocity = 0.0f;
            in_hump = false;
        }
        
        /* FR10: Detect shocks as total acceleration magnitude > ±3g threshold */
        float total_accel_g = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
        
        /* Post event to event queue */
        SystemEvent event = { .type = SYSTEM_EVENT_IMU, .timestamp_ms = Robot_Millis() };
        event.data.imu.acceleration_x_g = ax_g;
        event.data.imu.acceleration_y_g = ay_g;
        event.data.imu.acceleration_z_g = az_g;
        event.data.imu.turn_rate_dps = (float)raw.gz / IMU_GYRO_COUNTS_PER_DPS;
        event.data.imu.estimated_hump_height_m = estimated_height;
        event.data.imu.shock_detected = valid && (total_accel_g > IMU_SHOCK_THRESHOLD_G);
        event.data.imu.sensor_valid = valid;
        
        /* Classify motion state from sensor data */
        event.data.imu.motion_state = classify_motion(ax_g, ay_g, az_g, 
                                                       event.data.imu.turn_rate_dps);
        
        /* Calculate tilt angles from acceleration vector */
        calculate_tilt_angles(ax_g, ay_g, az_g, 
                            &event.data.imu.tilt_roll_deg, 
                            &event.data.imu.tilt_pitch_deg);
        
        (void)Robot_PostEvent(&event);
        
        /* NFR7: Sample at 10 ms intervals (100 Hz) */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_IMU_PERIOD_MS));
    }
}


void Buddy4Imu_CreateTask(void)
{
    BaseType_t result = xTaskCreate(imu_task, "IMUMonitor", ROBOT_TASK_STACK_WORDS,
                                    NULL, tskIDLE_PRIORITY + 3U, NULL);
    configASSERT(result == pdPASS);
}