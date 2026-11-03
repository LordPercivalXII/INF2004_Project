/** @file drv_lsm303dlhc.h
 * @brief LSM303DLHC accelerometer/magnetometer on kernel I2C0.
 */
#ifndef DRV_LSM303DLHC_H
#define DRV_LSM303DLHC_H

#include "imu_processing.h"

typedef enum
{
    LSM303_READ_ERROR = -1,
    LSM303_READ_WAIT = 0,
    LSM303_READ_READY = 1
} lsm303_read_result_t;

bool lsm303_init(void);
lsm303_read_result_t lsm303_read(imu_sample_t * p_sample);

/* Register byte decoding is exposed for deterministic tests. */
void lsm303_decode_accel(uint8_t const * p_bytes, float * p_acceleration);
bool lsm303_decode_mag(uint8_t const * p_bytes, float * p_magnetic_ut);

#endif /* DRV_LSM303DLHC_H */
