/** @file drv_lsm303dlhc.c
 * @brief ST LSM303DLHC: 100 Hz high-resolution +-4 g, 75 Hz +-1.3 gauss.
 * Register definitions/scales: ST LSM303DLHC datasheet, DocID018771.
 */
#include "drv_lsm303dlhc.h"

#if !defined(IMU_HOST_TEST)
#include "../app.h"
#include "../../device/include/dev_i2c.h"

#define LSM303_ACCEL_CTRL1       (0x20U)
#define LSM303_ACCEL_CTRL4       (0x23U)
#define LSM303_ACCEL_STATUS      (0x27U)
#define LSM303_ACCEL_OUT         (0xA8U) /* OUT_X_L | auto-increment */
#define LSM303_MAG_STATUS        (0x09U)
#define LSM303_MAG_OUT           (0x03U)

static ID g_h_i2c = E_ID;
static bool g_b_error_reported;

static bool
lsm303_transfer_ok (ER er, SZ actual, SZ expected, uint8_t address,
                    uint8_t reg)
{
    bool const b_ok = (E_OK == er) && (actual == expected);
    if ((!b_ok) && (!g_b_error_reported))
    {
        tm_printf((UB *) "S4: LSM303 I2C addr=0x%02x reg=0x%02x "
                  "error=%d size=%d expected=%d\n",
                  (unsigned int) address, (unsigned int) reg,
                  (int) er, (int) actual, (int) expected);
        g_b_error_reported = true;
    }
    return (b_ok);
}

static bool
lsm303_read_registers (uint8_t address, uint8_t reg, uint8_t * p_data,
                       uint32_t length)
{
    T_I2C_EXEC exec = {0};
    SZ actual = 0;
    ER er;
    exec.sadr = address;
    exec.snd_size = 1;
    exec.snd_data = &reg;
    exec.rcv_size = (SZ) length;
    exec.rcv_data = p_data;
    er = tk_swri_dev(g_h_i2c, TDN_I2C_EXEC, &exec, sizeof(exec), &actual);
    return (lsm303_transfer_ok(er, actual, sizeof(exec), address, reg));
}

static bool
lsm303_configure (uint8_t address, uint8_t reg, uint8_t value)
{
    UB bytes[2] = {reg, value};
    SZ actual = 0;
    uint8_t readback = 0;
    ER const er = tk_swri_dev(g_h_i2c, address, bytes, sizeof(bytes), &actual);
    bool b_ok = lsm303_transfer_ok(er, actual, sizeof(bytes), address, reg);
    b_ok = b_ok && lsm303_read_registers(address, reg, &readback, 1U);
    if (b_ok && (readback != value))
    {
        tm_printf((UB *) "S4: LSM303 config mismatch addr=0x%02x "
                  "reg=0x%02x read=0x%02x expected=0x%02x\n",
                  (unsigned int) address, (unsigned int) reg,
                  (unsigned int) readback, (unsigned int) value);
        b_ok = false;
    }
    return (b_ok);
}

bool
lsm303_init (void)
{
    uint8_t accel_id = 0U;
    uint8_t mag_id[3] = {0};
    bool b_ok;
    g_b_error_reported = false;
    if (g_h_i2c <= 0)
    {
        g_h_i2c = tk_opn_dev((UB *) IMU_I2C_DEVICE_NAME, TD_UPDATE);
    }
    if (g_h_i2c <= 0)
    {
        tm_printf((UB *) "S4: cannot open iica: error=%d\n", (int) g_h_i2c);
        return (false);
    }
    b_ok = lsm303_read_registers(IMU_ACCEL_ADDRESS, 0x0FU, &accel_id, 1U) &&
           lsm303_read_registers(IMU_MAG_ADDRESS, 0x0AU, mag_id, 3U);
    if (b_ok)
    {
        tm_printf((UB *) "S4: LSM303 IDs accel=0x%02x mag=%02x,%02x,%02x\n",
                  (unsigned int) accel_id, (unsigned int) mag_id[0],
                  (unsigned int) mag_id[1], (unsigned int) mag_id[2]);
        b_ok = (0x33U == accel_id) && (0x48U == mag_id[0]) &&
               (0x34U == mag_id[1]) && (0x33U == mag_id[2]);
        if (!b_ok)
        {
            tm_printf((UB *) "S4: unexpected LSM303DLHC identity\n");
        }
    }
    /* BDU, little endian, +-4 g, high resolution; enable XYZ at 100 Hz.
       Magnetometer: 75 Hz, gain +-1.3 gauss, continuous conversion. */
    b_ok = b_ok &&
        lsm303_configure(IMU_ACCEL_ADDRESS, LSM303_ACCEL_CTRL4, 0x98U) &&
        lsm303_configure(IMU_ACCEL_ADDRESS, LSM303_ACCEL_CTRL1, 0x57U) &&
        lsm303_configure(IMU_MAG_ADDRESS, 0x00U, 0x18U) &&
        lsm303_configure(IMU_MAG_ADDRESS, 0x01U, 0x20U) &&
        lsm303_configure(IMU_MAG_ADDRESS, 0x02U, 0x00U);
    if (!b_ok)
    {
        app_check(tk_cls_dev(g_h_i2c, 0), "S4 close iica");
        g_h_i2c = E_ID;
    }
    else
    {
        (void) tk_dly_tsk(30);
        tm_printf((UB *) "S4: LSM303DLHC ready: accel 100 Hz, mag 75 Hz; "
                  "NO gyro, turn/height are estimates\n");
    }
    return (b_ok);
}

lsm303_read_result_t
lsm303_read (imu_sample_t * p_sample)
{
    uint8_t status = 0;
    uint8_t bytes[6];
    *p_sample = (imu_sample_t) {0};
    if ((g_h_i2c <= 0) ||
        (!lsm303_read_registers(IMU_ACCEL_ADDRESS, LSM303_ACCEL_STATUS,
                                 &status, 1U)))
    {
        return (LSM303_READ_ERROR);
    }
    if (0U == (status & 0x08U))
    {
        return (LSM303_READ_WAIT);
    }
    if (!lsm303_read_registers(IMU_ACCEL_ADDRESS, LSM303_ACCEL_OUT, bytes, 6U))
    {
        return (LSM303_READ_ERROR);
    }
    lsm303_decode_accel(bytes, p_sample->acceleration);
    p_sample->b_magnetic_valid =
        lsm303_read_registers(IMU_MAG_ADDRESS, LSM303_MAG_STATUS, &status, 1U);
    if (p_sample->b_magnetic_valid && (0U != (status & 0x03U)))
    {
        /* Reading all six bytes also releases LOCK after a partial read. */
        p_sample->b_magnetic_valid =
            lsm303_read_registers(IMU_MAG_ADDRESS, LSM303_MAG_OUT, bytes, 6U);
        if (p_sample->b_magnetic_valid)
        {
            p_sample->b_magnetic_valid =
                lsm303_decode_mag(bytes, p_sample->magnetic_ut);
            p_sample->b_magnetic_fresh = p_sample->b_magnetic_valid;
        }
    }
    return (LSM303_READ_READY);
}
#endif /* !IMU_HOST_TEST */

void
lsm303_decode_accel (uint8_t const * p_bytes, float * p_acceleration)
{
    uint32_t axis;
    for (axis = 0U; axis < 3U; axis++)
    {
        int16_t const raw = (int16_t) (((uint16_t) p_bytes[axis * 2U + 1U]
                                     << 8U) | p_bytes[axis * 2U]);
        /* 12-bit left-aligned two's complement, 2 mg/digit at +-4 g. */
        p_acceleration[axis] = (float) (raw / 16) * 0.002f;
    }
}

bool
lsm303_decode_mag (uint8_t const * p_bytes, float * p_magnetic_ut)
{
    int16_t const x = (int16_t) (((uint16_t) p_bytes[0] << 8U) | p_bytes[1]);
    int16_t const z = (int16_t) (((uint16_t) p_bytes[2] << 8U) | p_bytes[3]);
    int16_t const y = (int16_t) (((uint16_t) p_bytes[4] << 8U) | p_bytes[5]);
    /* Magnetometer output is big endian X,Z,Y, with a different Z gain. */
    if ((-4096 == x) || (-4096 == y) || (-4096 == z))
    {
        return (false);
    }
    p_magnetic_ut[0] = (float) x * (100.0f / 1100.0f);
    p_magnetic_ut[1] = (float) y * (100.0f / 1100.0f);
    p_magnetic_ut[2] = (float) z * (100.0f / 980.0f);
    return (true);
}
