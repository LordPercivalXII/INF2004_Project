/** @file drv_mpu6050.c
 *
 * @brief MPU6050 driver.  Called only from imu_task (owner of I2C0).
 *
 *   Pico SDK                              micro T-Kernel device driver
 *   ------------------------------------  -------------------------------
 *   i2c_init(i2c0, 400 kHz) + pin setup   "iica" started by the kernel
 *                                         (config_device.h, pins in
 *                                         i2c_cnf_sysdep.h, 400 kHz)
 *   i2c_write_blocking(addr, {reg, v})    tk_swri_dev(dd, addr, ..)
 *   write reg + i2c_read_blocking         tk_swri_dev(dd, TDN_I2C_EXEC,
 *                                         T_I2C_EXEC) (repeated start)
 */

#include <tk/tkernel.h>
#include "../../device/include/dev_i2c.h"

#include "drv_mpu6050.h"
#include "../common/robot_config.h"

#define MPU6050_REG_SMPLRT_DIV      (0x19U)
#define MPU6050_REG_GYRO_CONFIG     (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG    (0x1CU)
#define MPU6050_REG_ACCEL_XOUT_H    (0x3BU)
#define MPU6050_REG_PWR_MGMT_1      (0x6BU)
#define MPU6050_REG_WHO_AM_I        (0x75U)

#define MPU6050_ID_A                (0x68U)
#define MPU6050_ID_B                (0x69U)
#define MPU6050_WAKE                (0x00U)
#define MPU6050_ACCEL_4G            (0x08U)
#define MPU6050_GYRO_1000DPS        (0x08U)
#define MPU6050_DIV_100HZ           (0x09U)
#define MPU6050_POWER_UP_MS         (100)
#define MPU6050_BURST_BYTES         (14U)

static ID g_h_i2c = E_ID;

static bool
mpu6050_write_register (uint8_t reg, uint8_t value)
{
    UB  bytes[2];
    SZ  asz = 0;
    ER  er;

    bytes[0] = reg;
    bytes[1] = value;
    er = tk_swri_dev(g_h_i2c, (D) IMU_I2C_ADDRESS, bytes,
                     (SZ) sizeof(bytes), &asz);

    return ((E_OK == er) && ((SZ) sizeof(bytes) == asz));
}

static bool
mpu6050_read_registers (uint8_t reg, uint8_t * p_data, uint32_t length)
{
    T_I2C_EXEC  exec;
    UB          reg_byte = reg;
    SZ          asz      = 0;
    ER          er;

    exec.sadr     = IMU_I2C_ADDRESS;
    exec.snd_size = 1;
    exec.snd_data = &reg_byte;
    exec.rcv_size = (SZ) length;
    exec.rcv_data = p_data;
    er = tk_swri_dev(g_h_i2c, (D) TDN_I2C_EXEC, &exec,
                     (SZ) sizeof(exec), &asz);

    return (E_OK == er);
}

/*!
 * @brief Open "iica", check WHO_AM_I and configure +-4 g, +-1000 dps,
 *        ~100 Hz.
 *
 * @return true if the sensor answered and accepted the configuration.
 */
bool
mpu6050_init (void)
{
    uint8_t identity = 0U;
    bool    b_ok     = false;

    g_h_i2c = tk_opn_dev((UB *) IMU_I2C_DEVICE_NAME, TD_UPDATE);

    if (g_h_i2c > 0)
    {
        (void) tk_dly_tsk(MPU6050_POWER_UP_MS);

        b_ok = mpu6050_read_registers(MPU6050_REG_WHO_AM_I, &identity, 1U)
               && ((MPU6050_ID_A == identity) ||
                   (MPU6050_ID_B == identity));

        b_ok = b_ok &&
               mpu6050_write_register(MPU6050_REG_PWR_MGMT_1,
                                      MPU6050_WAKE) &&
               mpu6050_write_register(MPU6050_REG_ACCEL_CONFIG,
                                      MPU6050_ACCEL_4G) &&
               mpu6050_write_register(MPU6050_REG_GYRO_CONFIG,
                                      MPU6050_GYRO_1000DPS) &&
               mpu6050_write_register(MPU6050_REG_SMPLRT_DIV,
                                      MPU6050_DIV_100HZ);
    }

    return (b_ok);
}

/*!
 * @brief Burst-read accelerometer XYZ and gyro Z.
 *
 * @return true on a successful bus transfer.
 */
bool
mpu6050_read (mpu6050_raw_t * p_raw)
{
    uint8_t data[MPU6050_BURST_BYTES];
    bool    b_ok = false;

    if (g_h_i2c > 0)
    {
        b_ok = mpu6050_read_registers(MPU6050_REG_ACCEL_XOUT_H, data,
                                      MPU6050_BURST_BYTES);
    }

    if (b_ok)
    {
        p_raw->ax = (int16_t) (((uint16_t) data[0] << 8U) | data[1]);
        p_raw->ay = (int16_t) (((uint16_t) data[2] << 8U) | data[3]);
        p_raw->az = (int16_t) (((uint16_t) data[4] << 8U) | data[5]);
        p_raw->gz = (int16_t) (((uint16_t) data[12] << 8U) | data[13]);
    }

    return (b_ok);
}

/*** end of file ***/
