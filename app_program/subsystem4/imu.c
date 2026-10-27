/** @file imu.c
 *
 * @brief IMU task, every 10 ms (NFR7): reads the MPU6050, estimates hump
 *        height (FR11), detects shocks (FR10), classifies motion and
 *        posts SYSTEM_EVENT_IMU.
 */

#include "subsystem4.h"
#include "drv_mpu6050.h"
#include "../common/drv_board.h"
#include "../common/rmath.h"
#include "../controller/controller.h"

#define IMU_SIM_SPIKE_PERIOD    (40U)
#define IMU_SIM_SPIKE_SAMPLES   (4U)
#define IMU_SIM_SPIKE_COUNTS    (16384)     /* ~2 g */

typedef struct
{
    float vertical_velocity;
    float max_vertical_velocity;
    bool  b_in_hump;
} imu_hump_t;

static bool
imu_sample (bool b_initialized, mpu6050_raw_t * p_raw, uint32_t sample)
{
    bool b_valid = false;

#if ROBOT_SIMULATION
    (void) b_initialized;
    p_raw->ax = 0;
    p_raw->ay = (int16_t) (((sample % IMU_SIM_SPIKE_PERIOD) <
                            IMU_SIM_SPIKE_SAMPLES) ? IMU_SIM_SPIKE_COUNTS
                                                   : 0);
    p_raw->az = (int16_t) IMU_ACCEL_COUNTS_PER_G;
    p_raw->gz = 0;
    b_valid = true;
#else
    (void) sample;
    b_valid = b_initialized && mpu6050_read(p_raw);
#endif

    return (b_valid);
}

/*!
 * @brief FR11: integrate filtered vertical acceleration while above the
 *        threshold; when it returns to baseline, h = v_max^2 / (2 g).
 *
 * @return Estimated hump height (m) at the end of a hump, else 0.
 */
static float
imu_update_hump (imu_hump_t * p_hump, bool b_valid, float vertical_g)
{
    float const dt     = (float) ROBOT_IMU_PERIOD_MS / 1000.0f;
    float       height = 0.0f;

    if (b_valid && (rmath_absf(vertical_g) > IMU_HUMP_THRESHOLD_G))
    {
        p_hump->b_in_hump = true;
        p_hump->vertical_velocity += vertical_g * IMU_GRAVITY_M_S2 * dt;
        p_hump->max_vertical_velocity =
            rmath_maxf(p_hump->max_vertical_velocity,
                       rmath_absf(p_hump->vertical_velocity));
    }
    else if (b_valid && p_hump->b_in_hump)
    {
        height = (p_hump->max_vertical_velocity *
                  p_hump->max_vertical_velocity) /
                 (2.0f * IMU_GRAVITY_M_S2);
        p_hump->vertical_velocity     = 0.0f;
        p_hump->max_vertical_velocity = 0.0f;
        p_hump->b_in_hump             = false;
    }
    else
    {
        /* Flat, or no valid sample. */
    }

    return (height);
}

/*!
 * @brief IMU task (10 ms, woken by its cyclic handler).
 */
void
imu_task (INT stacd, void * p_exinf)
{
    imu_filter_t   az_filter;
    imu_hump_t     hump    = {0.0f, 0.0f, false};
    mpu6050_raw_t  raw;
    system_event_t event;
    uint32_t       sample  = 0U;
    bool           b_initialized;
    bool           b_valid;
    float          total_g;

    (void) stacd;
    (void) p_exinf;

#if ROBOT_SIMULATION
    b_initialized = true;
#else
    b_initialized = mpu6050_init();

    if (!b_initialized)
    {
        tm_printf((UB *) "S4: MPU6050 not found on iica\n");
    }
#endif

    imu_filter_init(&az_filter);
    (void) tk_can_wup(TSK_SELF);    /* drop wake-ups queued during init */

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);    /* woken every 10 ms */

        raw.ax = 0;
        raw.ay = 0;
        raw.az = 0;
        raw.gz = 0;
        b_valid = imu_sample(b_initialized, &raw, sample);
        sample++;

        event = (system_event_t) {0};
        event.type         = SYSTEM_EVENT_IMU;
        event.timestamp_ms = board_millis();
        event.data.imu.acceleration_x_g = (float) raw.ax /
                                          IMU_ACCEL_COUNTS_PER_G;
        event.data.imu.acceleration_y_g = (float) raw.ay /
                                          IMU_ACCEL_COUNTS_PER_G;
        event.data.imu.acceleration_z_g = (float) raw.az /
                                          IMU_ACCEL_COUNTS_PER_G;
        event.data.imu.turn_rate_dps    = (float) raw.gz /
                                          IMU_GYRO_COUNTS_PER_DPS;

        /* FR11: gravity removed from the vertical axis (flat terrain). */
        event.data.imu.estimated_hump_height_m =
            imu_update_hump(&hump, b_valid,
                            imu_filter_update(&az_filter,
                                event.data.imu.acceleration_z_g - 1.0f));

        /* FR10: shock = total acceleration above the threshold. */
        total_g = rmath_sqrtf(
            (event.data.imu.acceleration_x_g *
             event.data.imu.acceleration_x_g) +
            (event.data.imu.acceleration_y_g *
             event.data.imu.acceleration_y_g) +
            (event.data.imu.acceleration_z_g *
             event.data.imu.acceleration_z_g));
        event.data.imu.b_shock_detected =
            b_valid && (total_g > IMU_SHOCK_THRESHOLD_G);
        event.data.imu.b_sensor_valid = b_valid;
        event.data.imu.motion_state =
            imu_classify_motion(event.data.imu.acceleration_x_g,
                                event.data.imu.acceleration_y_g,
                                event.data.imu.acceleration_z_g,
                                event.data.imu.turn_rate_dps);
        imu_tilt_angles(event.data.imu.acceleration_x_g,
                        event.data.imu.acceleration_y_g,
                        event.data.imu.acceleration_z_g,
                        &event.data.imu.tilt_roll_deg,
                        &event.data.imu.tilt_pitch_deg);

        (void) controller_post_event(&event);
    }
}

/*** end of file ***/
