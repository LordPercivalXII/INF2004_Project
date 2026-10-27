/** @file imu_analysis.c
 *
 * @brief IMU signal processing: moving average, motion classification,
 *        tilt angles.  Pure functions (unit-tested in subsystem4_test.c).
 */

#include "subsystem4.h"
#include "../common/rmath.h"

#define IMU_IMPACT_G            (3.0f)
#define IMU_TURNING_DPS         (20.0f)
#define IMU_CLIMBING_G          (1.3f)
#define IMU_DESCENDING_G        (0.7f)
#define IMU_ACCELERATING_G      (1.5f)

void
imu_filter_init (imu_filter_t * p_filter)
{
    uint32_t idx;

    p_filter->index = 0U;
    p_filter->count = 0U;

    for (idx = 0U; idx < IMU_FILTER_WINDOW_SIZE; idx++)
    {
        p_filter->samples[idx] = 0.0f;
    }
}

/*!
 * @brief Add a sample; return the average of the samples so far (up to
 *        IMU_FILTER_WINDOW_SIZE).
 */
float
imu_filter_update (imu_filter_t * p_filter, float value)
{
    float    sum = 0.0f;
    uint32_t idx;

    p_filter->samples[p_filter->index] = value;
    p_filter->index = (uint8_t) ((p_filter->index + 1U) %
                                 IMU_FILTER_WINDOW_SIZE);

    if (p_filter->count < IMU_FILTER_WINDOW_SIZE)
    {
        p_filter->count++;
    }

    for (idx = 0U; idx < p_filter->count; idx++)
    {
        sum += p_filter->samples[idx];
    }

    return (sum / (float) p_filter->count);
}

/*!
 * @brief Classify motion.  Priority: impact > turning > climbing /
 *        descending > accelerating > stationary.
 */
motion_class_t
imu_classify_motion (float ax_g, float ay_g, float az_g, float gz_dps)
{
    float const    total = rmath_sqrtf((ax_g * ax_g) + (ay_g * ay_g) +
                                       (az_g * az_g));
    motion_class_t result = MOTION_STATIONARY;

    if (total > IMU_IMPACT_G)
    {
        result = MOTION_IMPACT;
    }
    else if (rmath_absf(gz_dps) > IMU_TURNING_DPS)
    {
        result = MOTION_TURNING;
    }
    else if (az_g > IMU_CLIMBING_G)
    {
        result = MOTION_CLIMBING;
    }
    else if (az_g < IMU_DESCENDING_G)
    {
        result = MOTION_DESCENDING;
    }
    else if (total > IMU_ACCELERATING_G)
    {
        result = MOTION_ACCELERATING;
    }
    else
    {
        result = MOTION_STATIONARY;
    }

    return (result);
}

/*!
 * @brief Roll = atan2(ay, az); pitch = atan2(-ax, sqrt(ay^2 + az^2)).
 *        Valid when acceleration is roughly gravity (at rest).
 */
void
imu_tilt_angles (float ax_g, float ay_g, float az_g, float * p_roll_deg,
                 float * p_pitch_deg)
{
    *p_roll_deg  = rmath_atan2f(ay_g, az_g) * RMATH_RAD_TO_DEG;
    *p_pitch_deg = rmath_atan2f(-ax_g,
                                rmath_sqrtf((ay_g * ay_g) + (az_g * az_g)))
                   * RMATH_RAD_TO_DEG;
}

/*** end of file ***/
