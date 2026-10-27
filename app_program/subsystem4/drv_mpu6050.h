/** @file drv_mpu6050.h
 *
 * @brief MPU6050 accelerometer/gyro over the kernel I2C driver.
 */

#ifndef DRV_MPU6050_H
#define DRV_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gz;
} mpu6050_raw_t;

bool mpu6050_init(void);
bool mpu6050_read(mpu6050_raw_t * p_raw);

#endif /* DRV_MPU6050_H */

/*** end of file ***/
