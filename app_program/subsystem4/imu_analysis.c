/** @file imu_analysis.c
 * @brief LSM303DLHC calibration, acceleration-only terrain, magnetic turns.
 */
#include "imu_processing.h"
#include "../common/rmath.h"

#define IMU_CAL_ACCEL_SPAN_G     (0.04f)
#define IMU_ATTITUDE_TAU_S       (0.15f)
#define IMU_MAX_INTERVAL_MS      (30U)
#define IMU_HUMP_TRIGGER_COUNT   (3U)
#define IMU_HUMP_QUIET_COUNT     (15U)
#define IMU_HUMP_MIN_SAMPLES     (30U)
#define IMU_HUMP_MIN_HEIGHT_M    (0.005f)
#define IMU_HUMP_MAX_HEIGHT_M    (0.50f)
#define IMU_HUMP_MAX_DRIFT_M_S2  (0.30f)
#define IMU_HUMP_MAX_DURATION_S  (4.0f)

void
imu_filter_init (imu_filter_t * p_filter)
{
    *p_filter = (imu_filter_t) {0};
}

float
imu_filter_update (imu_filter_t * p_filter, float value)
{
    float sum = 0.0f;
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

void
imu_tilt_angles (float ax_g, float ay_g, float az_g, float * p_roll_deg,
                 float * p_pitch_deg)
{
    *p_roll_deg = rmath_atan2f(ay_g, az_g) * RMATH_RAD_TO_DEG;
    *p_pitch_deg = rmath_atan2f(-ax_g,
        rmath_sqrtf((ay_g * ay_g) + (az_g * az_g))) * RMATH_RAD_TO_DEG;
}

void
imu_processor_init (imu_processor_t * p_processor)
{
    *p_processor = (imu_processor_t) {0};
    p_processor->gravity[2] = 1.0f;
    p_processor->motion = MOTION_UNKNOWN;
    p_processor->candidate = MOTION_UNKNOWN;
    p_processor->output.motion_state = MOTION_UNKNOWN;
    p_processor->output.b_turn_rate_estimated = true;
    p_processor->output.b_height_approximate = true;
}

static float
imu_norm (float const * p_vector)
{
    return (rmath_sqrtf((p_vector[0] * p_vector[0]) +
                        (p_vector[1] * p_vector[1]) +
                        (p_vector[2] * p_vector[2])));
}

static void
imu_reset_tracking (imu_processor_t * p_processor)
{
    if (p_processor->b_in_hump)
    {
        p_processor->output.b_hump_rejected = true;
        p_processor->output.rejected_hump_count++;
    }
    p_processor->b_in_hump = false;
    p_processor->hump_count = 0U;
    p_processor->hump_trigger_count = 0U;
    p_processor->hump_quiet_count = 0U;
    p_processor->hump_velocity = 0.0f;
    p_processor->hump_duration_s = 0.0f;
}

static bool
imu_calibrate (imu_processor_t * p_processor, imu_sample_t const * p_sample)
{
    float values[3];
    bool b_still = (rmath_absf(p_sample->acceleration[0]) < 0.15f) &&
                   (rmath_absf(p_sample->acceleration[1]) < 0.15f) &&
                   (p_sample->acceleration[2] > 0.85f) &&
                   (p_sample->acceleration[2] < 1.15f);
    uint32_t idx;

    for (idx = 0U; idx < 3U; idx++)
    {
        values[idx] = p_sample->acceleration[idx];
    }
    for (idx = 0U; idx < 3U; idx++)
    {
        if (0U == p_processor->calibration_count)
        {
            p_processor->minimum[idx] = values[idx];
            p_processor->maximum[idx] = values[idx];
            p_processor->sum[idx] = 0.0f;
        }
        p_processor->minimum[idx] =
            rmath_minf(p_processor->minimum[idx], values[idx]);
        p_processor->maximum[idx] =
            rmath_maxf(p_processor->maximum[idx], values[idx]);
        b_still = b_still &&
            ((p_processor->maximum[idx] - p_processor->minimum[idx]) <=
             IMU_CAL_ACCEL_SPAN_G);
        p_processor->sum[idx] += values[idx];
    }
    if (!b_still)
    {
        p_processor->calibration_count = 0U;
    }
    else
    {
        p_processor->calibration_count++;
    }
    if (p_processor->calibration_count == IMU_CALIBRATION_SAMPLES)
    {
        for (idx = 0U; idx < 3U; idx++)
        {
            p_processor->bias[idx] =
                p_processor->sum[idx] / (float) IMU_CALIBRATION_SAMPLES;
        }
        p_processor->bias[2] -= 1.0f;
        p_processor->output.b_calibrated = true;
    }
    return (p_processor->output.b_calibrated);
}

static void
imu_update_attitude (imu_processor_t * p_processor,
                     imu_sample_t const * p_sample, float dt)
{
    float const norm = imu_norm(p_sample->acceleration);
    float * p_gravity = p_processor->gravity;
    float length;
    float correction = 0.0f;
    uint32_t idx;

    if (!p_processor->b_attitude_initialized)
    {
        for (idx = 0U; idx < 3U; idx++)
        {
            p_gravity[idx] = p_sample->acceleration[idx] / norm;
        }
        p_processor->b_attitude_initialized = true;
    }
    /* With no gyro this is only quasi-static tilt. Freeze during a
       candidate hump so its acceleration is not absorbed as gravity. */
    if ((!p_processor->b_in_hump) &&
        (0U == p_processor->hump_trigger_count) &&
        (rmath_absf(norm - 1.0f) < 0.08f))
    {
        correction = dt / (IMU_ATTITUDE_TAU_S + dt);
    }
    for (idx = 0U; idx < 3U; idx++)
    {
        p_gravity[idx] = p_gravity[idx] * (1.0f - correction) +
            (p_sample->acceleration[idx] / norm) * correction;
    }
    length = imu_norm(p_gravity);
    for (idx = 0U; idx < 3U; idx++)
    {
        p_gravity[idx] /= length;
    }
}

static void
imu_invalidate_heading (imu_processor_t * p_processor, bool b_disturbed)
{
    p_processor->b_have_heading = false;
    p_processor->output.b_heading_valid = false;
    p_processor->output.b_turn_rate_valid = false;
    p_processor->output.b_magnetic_disturbed = b_disturbed;
    p_processor->output.turn_rate_dps = 0.0f;
}

static void
imu_update_heading (imu_processor_t * p_processor,
                     imu_sample_t const * p_sample, float total,
                     float horizontal_g, uint32_t now_ms)
{
    imu_event_t * p_output = &p_processor->output;
    float field;
    float roll;
    float pitch;
    float hx;
    float hy;
    float heading;
    float delta;
    float rate;
    uint32_t elapsed;
    uint32_t idx;
    bool b_valid = p_sample->b_magnetic_valid;

    if (!b_valid)
    {
        imu_invalidate_heading(p_processor, true);
        return;
    }
    /* Heading is unsafe during a hump or significant acceleration. */
    if ((!p_output->b_tilt_valid) || p_processor->b_in_hump ||
        (horizontal_g > 0.08f) || (rmath_absf(total - 1.0f) > 0.08f) ||
        (rmath_absf(p_output->tilt_roll_deg) > 20.0f) ||
        (rmath_absf(p_output->tilt_pitch_deg) > 20.0f))
    {
        imu_invalidate_heading(p_processor, false);
        return;
    }
    if (!p_sample->b_magnetic_fresh)
    {
        if ((now_ms - p_processor->heading_ms) > 150U)
        {
            imu_invalidate_heading(p_processor, false);
        }
        return;
    }
    for (idx = 0U; idx < 3U; idx++)
    {
        b_valid = b_valid && (p_sample->magnetic_ut[idx] >= -130.0f) &&
                            (p_sample->magnetic_ut[idx] <= 130.0f);
    }
    field = b_valid ? imu_norm(p_sample->magnetic_ut) : 0.0f;
    if ((!b_valid) || (field < 10.0f) || (field > 100.0f))
    {
        imu_invalidate_heading(p_processor, true);
        return;
    }
    if (!p_processor->b_have_magnetic_reference)
    {
        p_processor->reference_field_ut = field;
        p_processor->b_have_magnetic_reference = true;
    }
    if (rmath_absf(field - p_processor->reference_field_ut) >
        (p_processor->reference_field_ut * 0.25f))
    {
        imu_invalidate_heading(p_processor, true);
        return;
    }
    roll = p_output->tilt_roll_deg * RMATH_DEG_TO_RAD;
    pitch = p_output->tilt_pitch_deg * RMATH_DEG_TO_RAD;
    hx = p_sample->magnetic_ut[0] * rmath_cosf(pitch) +
         p_sample->magnetic_ut[1] * rmath_sinf(roll) * rmath_sinf(pitch) +
         p_sample->magnetic_ut[2] * rmath_cosf(roll) * rmath_sinf(pitch);
    hy = p_sample->magnetic_ut[1] * rmath_cosf(roll) -
         p_sample->magnetic_ut[2] * rmath_sinf(roll);
    if ((hx * hx + hy * hy) < 25.0f)
    {
        imu_invalidate_heading(p_processor, true);
        return;
    }
    heading = rmath_atan2f(-hy, hx) * RMATH_RAD_TO_DEG;
    if (heading < 0.0f)
    {
        heading += 360.0f;
    }
    elapsed = now_ms - p_processor->heading_ms;
    if (p_processor->b_have_heading && (elapsed >= 10U) && (elapsed <= 150U))
    {
        delta = heading - p_processor->previous_heading;
        if (delta > 180.0f)
        {
            delta -= 360.0f;
        }
        else if (delta < -180.0f)
        {
            delta += 360.0f;
        }
        rate = delta * 1000.0f / (float) elapsed;
        if (rmath_absf(rate) > 180.0f)
        {
            imu_invalidate_heading(p_processor, true);
            return;
        }
        p_output->turn_rate_dps += ((float) elapsed / (100.0f + (float) elapsed)) *
                                  (rate - p_output->turn_rate_dps);
        p_output->b_turn_rate_valid = true;
    }
    else
    {
        p_output->turn_rate_dps = 0.0f;
        p_output->b_turn_rate_valid = false;
    }
    p_output->magnetic_heading_deg = heading;
    p_output->b_heading_valid = true;
    p_output->b_magnetic_disturbed = false;
    p_processor->previous_heading = heading;
    p_processor->heading_ms = now_ms;
    p_processor->b_have_heading = true;
}

static void
imu_finish_hump (imu_processor_t * p_processor)
{
    float duration = 0.0f;
    float end_velocity = 0.0f;
    float drift;
    float velocity = 0.0f;
    float height = 0.0f;
    float peak = 0.0f;
    bool b_descended = false;
    uint32_t idx;

    for (idx = 0U; idx < p_processor->hump_count; idx++)
    {
        duration += p_processor->hump[idx].interval_s;
        end_velocity += p_processor->hump[idx].acceleration *
                        p_processor->hump[idx].interval_s;
    }
    /* A completed hump starts/ends at zero vertical speed. Remove the
       constant acceleration bias satisfying that boundary condition. */
    drift = end_velocity / duration;
    for (idx = 0U; idx < p_processor->hump_count; idx++)
    {
        float const dt = p_processor->hump[idx].interval_s;
        float const acceleration =
            p_processor->hump[idx].acceleration - drift;
        height += velocity * dt + 0.5f * acceleration * dt * dt;
        velocity += acceleration * dt;
        peak = rmath_maxf(peak, height);
        b_descended = b_descended || (velocity < -0.01f);
    }
    if ((rmath_absf(drift) <= IMU_HUMP_MAX_DRIFT_M_S2) &&
        b_descended && (peak >= IMU_HUMP_MIN_HEIGHT_M) &&
        (peak <= IMU_HUMP_MAX_HEIGHT_M) &&
        (rmath_absf(height) <= rmath_maxf(0.015f, peak * 0.5f)))
    {
        p_processor->output.estimated_hump_height_m = peak;
        p_processor->output.b_hump_height_valid = true;
        p_processor->output.b_hump_complete = true;
        p_processor->output.hump_count++;
        p_processor->b_in_hump = false;
    }
    imu_reset_tracking(p_processor);
}

static void
imu_update_hump (imu_processor_t * p_processor, float vertical_g,
                 float dt)
{
    uint32_t idx;

    if ((p_processor->hump_count == IMU_HUMP_MAX_SAMPLES) ||
        (p_processor->b_in_hump &&
         ((p_processor->hump_duration_s + dt) > IMU_HUMP_MAX_DURATION_S)))
    {
        imu_reset_tracking(p_processor);
        return;
    }
    if ((!p_processor->b_in_hump) &&
        (p_processor->hump_count == IMU_HUMP_PRE_SAMPLES))
    {
        for (idx = 1U; idx < IMU_HUMP_PRE_SAMPLES; idx++)
        {
            p_processor->hump[idx - 1U] = p_processor->hump[idx];
        }
        p_processor->hump_count--;
    }
    p_processor->hump[p_processor->hump_count].acceleration =
        vertical_g * IMU_GRAVITY_M_S2;
    p_processor->hump[p_processor->hump_count].interval_s = dt;
    p_processor->hump_count++;

    if (!p_processor->b_in_hump)
    {
        p_processor->hump_trigger_count =
            (vertical_g > IMU_HUMP_THRESHOLD_G)
            ? p_processor->hump_trigger_count + 1U : 0U;
        if (p_processor->hump_trigger_count >= IMU_HUMP_TRIGGER_COUNT)
        {
            p_processor->b_in_hump = true;
            p_processor->hump_velocity = 0.0f;
            p_processor->hump_duration_s = 0.0f;
            for (idx = 0U; idx < p_processor->hump_count; idx++)
            {
                p_processor->hump_velocity +=
                    p_processor->hump[idx].acceleration *
                    p_processor->hump[idx].interval_s;
                p_processor->hump_duration_s +=
                    p_processor->hump[idx].interval_s;
            }
        }
    }
    else
    {
        p_processor->hump_velocity += vertical_g * IMU_GRAVITY_M_S2 * dt;
        p_processor->hump_duration_s += dt;
        p_processor->hump_quiet_count =
            ((rmath_absf(vertical_g) < 0.04f) &&
             (rmath_absf(p_processor->output.tilt_pitch_deg) < 3.0f) &&
             (rmath_absf(p_processor->output.tilt_roll_deg) < 5.0f))
            ? p_processor->hump_quiet_count + 1U : 0U;
        if ((p_processor->hump_count >= IMU_HUMP_MIN_SAMPLES) &&
            (p_processor->hump_quiet_count >= IMU_HUMP_QUIET_COUNT))
        {
            imu_finish_hump(p_processor);
        }
    }
}

static void
imu_update_motion (imu_processor_t * p_processor, float horizontal_g,
                   uint32_t now_ms)
{
    imu_event_t * p_output = &p_processor->output;
    motion_class_t candidate = MOTION_STATIONARY;
    float const turn_limit =
        (MOTION_TURNING == p_processor->motion) ? 15.0f : 20.0f;
    float const accel_limit =
        (MOTION_ACCELERATING == p_processor->motion) ? 0.08f : 0.12f;

    if (p_output->b_shock_detected)
    {
        p_processor->motion = MOTION_IMPACT;
        p_processor->impact_ms = now_ms;
    }
    if ((MOTION_IMPACT == p_processor->motion) &&
        ((now_ms - p_processor->impact_ms) < IMU_IMPACT_HOLD_MS))
    {
        candidate = MOTION_IMPACT;
    }
    else if (p_processor->b_in_hump &&
             ((p_processor->hump_velocity < -0.01f) ||
              (p_output->tilt_pitch_deg > 3.0f)))
    {
        candidate = MOTION_DESCENDING;
    }
    else if (p_processor->b_in_hump)
    {
        candidate = MOTION_CLIMBING;
    }
    else if (p_output->b_turn_rate_valid &&
             (rmath_absf(p_output->turn_rate_dps) > turn_limit))
    {
        candidate = MOTION_TURNING;
    }
    else if (horizontal_g > accel_limit)
    {
        candidate = MOTION_ACCELERATING;
    }
    if (candidate != p_processor->candidate)
    {
        p_processor->candidate = candidate;
        p_processor->candidate_ms = now_ms;
    }
    if ((MOTION_IMPACT == candidate) ||
        ((now_ms - p_processor->candidate_ms) >= IMU_CLASSIFY_HOLD_MS))
    {
        p_processor->motion = candidate;
    }
    p_output->motion_state = p_processor->motion;
}

void
imu_processor_update (imu_processor_t * p_processor,
                       imu_sample_t const * p_sample, bool b_valid,
                       uint32_t timestamp_ms)
{
    imu_sample_t corrected = {0};
    imu_event_t * p_output = &p_processor->output;
    uint32_t const interval_ms = timestamp_ms - p_processor->last_ms;
    float dt = (float) interval_ms / 1000.0f;
    float total;
    float vertical_g = 0.0f;
    float horizontal_g = 0.0f;
    uint32_t idx;

    p_output->b_hump_complete = false;
    p_output->b_hump_rejected = false;
    p_output->b_shock_detected = false;
    p_output->b_saturated = false;
    p_output->b_tilt_valid = false;
    for (idx = 0U; idx < 3U; idx++)
    {
        /* Comparisons also reject NaN and infinity. */
        b_valid = b_valid &&
            (p_sample->acceleration[idx] >= -4.10f) &&
            (p_sample->acceleration[idx] <= 4.10f);
        p_output->b_saturated = p_output->b_saturated ||
            (rmath_absf(p_sample->acceleration[idx]) >= 4.05f);
    }
    if (p_processor->b_have_time)
    {
        b_valid = b_valid && (interval_ms >= 5U) &&
                  (interval_ms <= IMU_MAX_INTERVAL_MS);
    }
    else
    {
        dt = (float) ROBOT_IMU_PERIOD_MS / 1000.0f;
    }
    p_processor->last_ms = timestamp_ms;
    p_processor->b_have_time = true;
    p_output->b_sensor_valid = b_valid;

    if (!b_valid)
    {
        imu_reset_tracking(p_processor);
        imu_filter_init(&p_processor->vertical_filter);
        imu_filter_init(&p_processor->horizontal_filter);
        p_processor->calibration_count = 0U;
        p_processor->b_attitude_initialized = false;
        p_processor->motion = MOTION_UNKNOWN;
        p_processor->candidate = MOTION_UNKNOWN;
        p_processor->b_was_shock = false;
        p_output->motion_state = MOTION_UNKNOWN;
        p_output->b_hump_active = false;
        p_output->acceleration_x_g = 0.0f;
        p_output->acceleration_y_g = 0.0f;
        p_output->acceleration_z_g = 0.0f;
        p_output->vertical_acceleration_g = 0.0f;
        p_output->turn_rate_dps = 0.0f;
        p_output->tilt_roll_deg = 0.0f;
        p_output->tilt_pitch_deg = 0.0f;
        imu_invalidate_heading(p_processor, false);
        return;
    }
    if ((!p_output->b_calibrated) && (!imu_calibrate(p_processor, p_sample)))
    {
        return;
    }
    for (idx = 0U; idx < 3U; idx++)
    {
        corrected.acceleration[idx] =
            p_sample->acceleration[idx] - p_processor->bias[idx];
    }
    p_output->acceleration_x_g = corrected.acceleration[0];
    p_output->acceleration_y_g = corrected.acceleration[1];
    p_output->acceleration_z_g = corrected.acceleration[2];
    total = imu_norm(corrected.acceleration);
    p_output->b_shock_detected = (total > IMU_SHOCK_THRESHOLD_G);
    if (p_output->b_shock_detected && (!p_processor->b_was_shock))
    {
        p_output->shock_count++;
    }
    p_processor->b_was_shock = p_output->b_shock_detected;
    if ((total < 0.20f) || p_output->b_saturated)
    {
        imu_reset_tracking(p_processor);
        imu_filter_init(&p_processor->vertical_filter);
        imu_filter_init(&p_processor->horizontal_filter);
        p_processor->b_attitude_initialized = false;
        p_processor->motion = MOTION_UNKNOWN;
        p_processor->candidate = MOTION_UNKNOWN;
        p_output->motion_state = MOTION_UNKNOWN;
        p_output->b_hump_active = false;
        p_output->vertical_acceleration_g = 0.0f;
        imu_invalidate_heading(p_processor, false);
        if (p_output->b_shock_detected)
        {
            imu_update_motion(p_processor, 0.0f, timestamp_ms);
        }
        return;
    }
    if ((!p_processor->b_attitude_initialized) &&
        (rmath_absf(total - 1.0f) >= 0.08f))
    {
        p_output->motion_state = MOTION_UNKNOWN;
        if (p_output->b_shock_detected)
        {
            imu_update_motion(p_processor, 0.0f, timestamp_ms);
        }
        return;
    }
    imu_update_attitude(p_processor, &corrected, dt);
    imu_tilt_angles(p_processor->gravity[0], p_processor->gravity[1],
                     p_processor->gravity[2], &p_output->tilt_roll_deg,
                     &p_output->tilt_pitch_deg);
    p_output->b_tilt_valid = (rmath_absf(total - 1.0f) < 0.08f);
    for (idx = 0U; idx < 3U; idx++)
    {
        float const linear = corrected.acceleration[idx] -
                             p_processor->gravity[idx];
        horizontal_g += linear * linear;
    }
    /* Magnitude-based vertical approximation: only mild terrain, with
       predominantly vertical acceleration. It is not gyro compensation. */
    vertical_g = total - 1.0f;
    horizontal_g = rmath_sqrtf(rmath_maxf(0.0f,
        horizontal_g - (vertical_g * vertical_g)));
    p_output->vertical_acceleration_g =
        imu_filter_update(&p_processor->vertical_filter, vertical_g);
    horizontal_g = imu_filter_update(&p_processor->horizontal_filter,
                                      horizontal_g);
    p_output->b_tilt_valid = p_output->b_tilt_valid && (horizontal_g < 0.08f);
    if (p_output->b_shock_detected ||
        (rmath_absf(p_output->tilt_pitch_deg) > 15.0f) ||
        (rmath_absf(p_output->tilt_roll_deg) > 15.0f) ||
        (horizontal_g > 0.15f))
    {
        imu_reset_tracking(p_processor);
        imu_filter_init(&p_processor->vertical_filter);
        imu_filter_init(&p_processor->horizontal_filter);
    }
    else if (!p_processor->b_odometry_height)
    {
        imu_update_hump(p_processor, p_output->vertical_acceleration_g, dt);
    }
    p_output->b_hump_active = p_processor->b_in_hump;
    imu_update_heading(p_processor, p_sample, total, horizontal_g, timestamp_ms);
    imu_update_motion(p_processor, horizontal_g, timestamp_ms);
}

/*** end of file ***/
