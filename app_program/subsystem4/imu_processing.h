/** @file imu_processing.h
 * @brief Hardware-independent IMU processing, also used by the S4 tests.
 *
 * Mount with sensor X forward, Y left, Z up. Calibrate at rest on level
 * ground after each boot. Positive pitch here means nose DOWN.
 *
 * LSM303DLHC has NO gyro. Tilt uses low-pass acceleration; acceleration
 * and tilt cannot be separated during dynamic motion. Height is a bounded
 * accelerometer-only test/reference approximation for mild humps returning to
 * the same level with zero vertical speed. It is not a step-height or
 * absolute-altitude measurement. Check b_hump_height_valid and counts;
 * a rejected window leaves the last accepted peak unchanged. Quiet
 * constant-speed travel is indistinguishable from stationary by IMU alone.
 * Robot firmware disables that reference estimator and uses imu_height.h:
 * wheel travel plus pitch, fused by the controller receiving both events.
 * Turn rate is a relative magnetic-heading derivative, NOT gyro rate.
 * No hard/soft-iron calibration is assumed: motors/magnets can corrupt it
 * even when field magnitude checks pass. Never use it to control turns.
 */
#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include "../common/robot_types.h"
#include "../common/robot_config.h"

#define IMU_CALIBRATION_SAMPLES      (200U)
#define IMU_HUMP_MAX_SAMPLES         (400U)
#define IMU_HUMP_PRE_SAMPLES         (10U)
#define IMU_CLASSIFY_HOLD_MS         (80U)
#define IMU_IMPACT_HOLD_MS           (100U)
#define IMU_NORMAL_INTERVAL_MAX_MS   (30U)
#define IMU_RECOVERABLE_INTERVAL_MS  (100U)

typedef struct
{
    float   samples[IMU_FILTER_WINDOW_SIZE];
    uint8_t index;
    uint8_t count;
} imu_filter_t;

typedef struct
{
    float acceleration[3];    /* g */
    float magnetic_ut[3];
    bool b_magnetic_valid;
    bool b_magnetic_fresh;
} imu_sample_t;

typedef struct
{
    float acceleration;       /* world vertical, m/s^2 */
    float interval_s;
} imu_hump_sample_t;

typedef struct
{
    float cal_mean[3];        /* calibration window: running mean, g    */
    float cal_m2[3];          /* calibration window: sum of squared
                                 deviations (Welford), g^2              */
    float bias[3];
    uint32_t calibration_count;
    float gravity[3];
    bool b_attitude_initialized;
    bool b_have_time;
    bool b_odometry_height;
    bool b_was_shock;
    uint32_t last_ms;
    uint32_t candidate_ms;
    uint32_t impact_ms;
    motion_class_t candidate;
    motion_class_t motion;
    imu_filter_t vertical_filter;
    imu_filter_t horizontal_filter;
    imu_hump_sample_t hump[IMU_HUMP_MAX_SAMPLES];
    uint32_t hump_count;
    uint32_t hump_trigger_count;
    uint32_t hump_quiet_count;
    bool b_in_hump;
    float hump_velocity;
    float hump_duration_s;
    bool b_have_heading;
    bool b_have_magnetic_reference;
    uint32_t heading_ms;
    float previous_heading;
    float reference_field_ut;
    imu_event_t output;
} imu_processor_t;

void imu_filter_init(imu_filter_t * p_filter);
float imu_filter_update(imu_filter_t * p_filter, float value);
void imu_tilt_angles(float ax_g, float ay_g, float az_g,
                     float * p_roll_deg, float * p_pitch_deg);
void imu_processor_init(imu_processor_t * p_processor);
/* Largest per-axis std dev of the calibration window, g (diagnostics). */
float imu_calibration_noise_g(imu_processor_t const * p_processor);
void imu_processor_update(imu_processor_t * p_processor,
                           imu_sample_t const * p_sample, bool b_valid,
                           uint32_t timestamp_ms);

#endif /* IMU_PROCESSING_H */
