#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "common_types.h"
#include "interfaces/buddy4_imu.h"

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gz;
} ImuRaw;

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
    uint8_t identity = 0U;
    if (!imu_read_registers(0x75U, &identity, 1U) || (identity != 0x68U && identity != 0x69U)) return false;
    return imu_write_register(0x6BU, 0x00U) && imu_write_register(0x1CU, 0x00U) &&
           imu_write_register(0x1BU, 0x00U) && imu_write_register(0x19U, 0x09U);
#endif
}

static bool imu_read(ImuRaw *raw, uint32_t sample)
{
#if ROBOT_SIMULATION
    raw->ax = 0;
    raw->ay = (int16_t)((sample % 40U) < 4U ? 500 : 0);
    raw->az = (int16_t)IMU_ACCEL_COUNTS_PER_G;
    raw->gz = 0;
    return true;
#else
    uint8_t data[14];
    if (!imu_read_registers(0x3BU, data, sizeof(data))) return false;
    raw->ax = (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
    raw->ay = (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
    raw->az = (int16_t)(((uint16_t)data[4] << 8U) | data[5]);
    raw->gz = (int16_t)(((uint16_t)data[12] << 8U) | data[13]);
    return true;
#endif
}

static void imu_task(void *argument)
{
    (void)argument;
    const bool initialized = imu_init();
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t sample = 0U;
    float vertical_velocity = 0.0f;
    float max_vertical_velocity = 0.0f;
    bool in_hump = false;
    for (;;) {
        ImuRaw raw = { 0 };
        const bool valid = initialized && imu_read(&raw, sample++);
        const float ax_g = (float)raw.ax / IMU_ACCEL_COUNTS_PER_G;
        const float ay_g = (float)raw.ay / IMU_ACCEL_COUNTS_PER_G;
        const float az_g = (float)raw.az / IMU_ACCEL_COUNTS_PER_G;
        const float vertical_g = az_g - 1.0f;
        const float dt = (float)ROBOT_IMU_PERIOD_MS / 1000.0f;
        if (valid && fabsf(vertical_g) > IMU_HUMP_THRESHOLD_G) {
            in_hump = true;
            vertical_velocity += vertical_g * IMU_GRAVITY_M_S2 * dt;
            if (fabsf(vertical_velocity) > max_vertical_velocity) max_vertical_velocity = fabsf(vertical_velocity);
        }
        float estimated_height = 0.0f;
        if (valid && in_hump && fabsf(vertical_g) <= IMU_HUMP_THRESHOLD_G) {
            estimated_height = (max_vertical_velocity * max_vertical_velocity) / (2.0f * IMU_GRAVITY_M_S2);
            vertical_velocity = 0.0f;
            max_vertical_velocity = 0.0f;
            in_hump = false;
        }
        SystemEvent event = { .type = SYSTEM_EVENT_IMU, .timestamp_ms = Robot_Millis() };
        event.data.imu.acceleration_x_g = ax_g;
        event.data.imu.acceleration_y_g = ay_g;
        event.data.imu.acceleration_z_g = az_g;
        event.data.imu.turn_rate_dps = (float)raw.gz / IMU_GYRO_COUNTS_PER_DPS;
        event.data.imu.estimated_hump_height_m = estimated_height;
        event.data.imu.shock_detected = valid && sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g) > IMU_SHOCK_THRESHOLD_G;
        event.data.imu.sensor_valid = valid;
        (void)Robot_PostEvent(&event);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_IMU_PERIOD_MS));
    }
}

void Buddy4Imu_CreateTask(void)
{
    BaseType_t result = xTaskCreate(imu_task, "IMUMonitor", ROBOT_TASK_STACK_WORDS,
                                    NULL, tskIDLE_PRIORITY + 3U, NULL);
    configASSERT(result == pdPASS);
}