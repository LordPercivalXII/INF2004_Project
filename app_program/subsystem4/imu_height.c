/** @file imu_height.c
 * @brief Buddy 4 odometry-assisted ramp/hump height estimator.
 */
#include "imu_height.h"
#include "../common/rmath.h"

#define HEIGHT_MAX_GAP_MS        (100U)
#define HEIGHT_FLAT_SAMPLES      (10U)
#define HEIGHT_LEVEL_DEG         (1.0f)
#define HEIGHT_MIN_PEAK_M        (0.005f)

void
imu_height_init (imu_height_t * p_height)
{
    *p_height = (imu_height_t) {0};
}

void
imu_height_invalidate (imu_height_t * p_height, imu_height_rejection_t reason)
{
    p_height->last_rejection = reason;
    if (p_height->b_active)
    {
        p_height->rejected_count++;
        p_height->b_rejected = true;
    }
    p_height->b_active = false;
    p_height->b_complete = false;
    p_height->b_ready = false;
    p_height->b_valid = false;
    p_height->b_have_distance = false;
    p_height->b_descended = false;
    p_height->flat_samples = 0U;
    p_height->rise_m = 0.0f;
    p_height->peak_m = 0.0f;
    p_height->path_m = 0.0f;
    p_height->level_path_m = 0.0f;
    p_height->last_dz = 0.0f;
    p_height->b_gap = false;
    p_height->gap_segments = 0U;
    p_height->gap_path_m = 0.0f;
    p_height->bridged_path_m = 0.0f;
}

imu_height_rejection_t
imu_height_imu_rejection (imu_event_t const * p_imu)
{
    if (!p_imu->b_sensor_valid) return HEIGHT_REJECT_SENSOR;
    if (!p_imu->b_calibrated) return HEIGHT_REJECT_CALIBRATION;
    if (p_imu->b_saturated) return HEIGHT_REJECT_SATURATION;
    if (p_imu->b_shock_detected) return HEIGHT_REJECT_SHOCK;
    if (!p_imu->b_tilt_valid) return HEIGHT_REJECT_TILT;
    return HEIGHT_REJECT_NONE;
}

char const *
imu_height_rejection_name (imu_height_rejection_t reason)
{
    switch (reason)
    {
        case HEIGHT_REJECT_NONE: return "none";
        case HEIGHT_REJECT_SENSOR: return "sensor";
        case HEIGHT_REJECT_CALIBRATION: return "calibration";
        case HEIGHT_REJECT_SATURATION: return "saturation";
        case HEIGHT_REJECT_SHOCK: return "shock";
        case HEIGHT_REJECT_TILT: return "tilt";
        case HEIGHT_REJECT_IMU_AGE: return "imu_age";
        case HEIGHT_REJECT_PITCH: return "pitch_range";
        case HEIGHT_REJECT_ROLL: return "roll_range";
        case HEIGHT_REJECT_DISTANCE: return "distance_range";
        case HEIGHT_REJECT_SPEED: return "wheel_speed";
        case HEIGHT_REJECT_TURN: return "wheel_speed_difference";
        case HEIGHT_REJECT_INTERVAL: return "motion_interval";
        case HEIGHT_REJECT_DISTANCE_JUMP: return "distance_jump";
        case HEIGHT_REJECT_DOWNHILL_START: return "downhill_start";
        case HEIGHT_REJECT_PATH: return "path_limit";
        case HEIGHT_REJECT_RISE: return "rise_limit";
        case HEIGHT_REJECT_PEAK: return "peak_limit";
        case HEIGHT_REJECT_ENDPOINT: return "return_level";
        case HEIGHT_REJECT_WHEEL_AGE: return "wheel_age";
        case HEIGHT_REJECT_GAP_TIMEOUT: return "tilt_gap_timeout";
        case HEIGHT_REJECT_GAP_DISTANCE: return "tilt_gap_distance";
        case HEIGHT_REJECT_GAP_PITCH: return "tilt_gap_pitch_change";
        case HEIGHT_REJECT_GAP_BUDGET: return "tilt_gap_travel_budget";
        case HEIGHT_REJECT_GAP_BUFFER: return "tilt_gap_buffer";
        case HEIGHT_REJECT_GAP_ACCELERATION: return "tilt_gap_acceleration";
        default: return "invalid_reason";
    }
}

void
imu_height_observe_imu (imu_height_t * p_height,
                        imu_event_t const * p_imu, uint32_t imu_ms)
{
    imu_height_rejection_t const reason = imu_height_imu_rejection(p_imu);
    if ((HEIGHT_REJECT_NONE != reason) && (HEIGHT_REJECT_TILT != reason))
    {
        imu_height_invalidate(p_height, reason);
        return;
    }
    if ((HEIGHT_REJECT_TILT == reason) || p_height->b_gap)
    {
        float const ax = p_imu->acceleration_x_g;
        float const ay = p_imu->acceleration_y_g;
        float const az = p_imu->acceleration_z_g;
        float const norm_squared = ax * ax + ay * ay + az * az;
        if (!(norm_squared >= 0.85f * 0.85f &&
              norm_squared <= 1.15f * 1.15f))
        {
            imu_height_invalidate(p_height, HEIGHT_REJECT_GAP_ACCELERATION);
            return;
        }
    }
    if ((HEIGHT_REJECT_TILT == reason) && !p_height->b_gap)
    {
        if (!p_height->b_ready || !p_height->b_have_distance)
        {
            imu_height_invalidate(p_height, reason);
            return;
        }
        p_height->b_gap = true;
        p_height->gap_anchor_ms = p_height->last_ms;
        p_height->gap_slope = p_height->last_slope;
        p_height->gap_segments = 0U;
        p_height->gap_path_m = 0.0f;
    }
    if (p_height->b_gap)
    {
        p_height->b_valid = false;
        p_height->b_complete = false;
        if (((imu_ms - p_height->gap_anchor_ms) > 100U) &&
            ((p_height->gap_anchor_ms - imu_ms) > 60U))
        {
            imu_height_invalidate(p_height, HEIGHT_REJECT_GAP_TIMEOUT);
        }
    }
}

static void
imu_height_integrate (imu_height_t * p_height, float ds, float slope,
                       bool b_level)
{
    float const dz = ds * (slope + p_height->last_slope) * 0.5f;
    p_height->last_dz = dz;
    p_height->last_slope = slope;
    if (!p_height->b_active)
    {
        if ((!b_level) && (dz < -0.00001f))
        {
            imu_height_invalidate(p_height, HEIGHT_REJECT_DOWNHILL_START);
            return;
        }
        if (b_level || (dz <= 0.00001f))
        {
            return;
        }
        p_height->b_active = true;
    }
    p_height->rise_m += dz;
    p_height->peak_m = rmath_maxf(p_height->peak_m, p_height->rise_m);
    p_height->path_m += rmath_absf(ds);
    p_height->b_descended = p_height->b_descended ||
        ((p_height->peak_m - p_height->rise_m) > HEIGHT_MIN_PEAK_M);
    p_height->level_path_m = b_level
        ? p_height->level_path_m + rmath_absf(ds) : 0.0f;
    if ((p_height->path_m > 10.0f) ||
        (p_height->rise_m < -0.05f) || (p_height->peak_m > 1.0f))
    {
        imu_height_rejection_t const reason =
            (p_height->path_m > 10.0f) ? HEIGHT_REJECT_PATH
            : ((p_height->rise_m < -0.05f) ? HEIGHT_REJECT_RISE
               : HEIGHT_REJECT_PEAK);
        imu_height_invalidate(p_height, reason);
    }
}

void
imu_height_update (imu_height_t * p_height, motion_event_t const * p_motion,
                    imu_event_t const * p_imu, uint32_t imu_ms,
                    uint32_t motion_ms)
{
    uint32_t const interval = motion_ms - p_height->last_ms;
    uint32_t const age = motion_ms - imu_ms;
    float const pitch = p_imu->tilt_pitch_deg;
    float const distance = p_motion->distance_m;
    float const left = p_motion->left_speed_m_s;
    float const right = p_motion->right_speed_m_s;
    float slope;
    float ds;
    bool b_level;
    bool const b_had_distance = p_height->b_have_distance;
    imu_height_rejection_t reason = imu_height_imu_rejection(p_imu);

    p_height->b_complete = false;
    p_height->b_rejected = false;
    imu_height_observe_imu(p_height, p_imu, imu_ms);
    if ((b_had_distance && !p_height->b_have_distance) ||
        ((HEIGHT_REJECT_TILT == reason) && !p_height->b_gap)) return;
    if (HEIGHT_REJECT_TILT == reason && p_height->b_gap)
    {
        reason = HEIGHT_REJECT_NONE;
    }
    /* Accept a nearby newer IMU sample as well as a nearby older one:
       both event producers run independently, so enqueue order may differ. */
    if (HEIGHT_REJECT_NONE == reason)
    {
        if ((age > 60U) && ((imu_ms - motion_ms) > 60U))
            reason = HEIGHT_REJECT_IMU_AGE;
        else if (!(pitch >= -35.0f && pitch <= 35.0f))
            reason = HEIGHT_REJECT_PITCH;
        else if (!(p_imu->tilt_roll_deg >= -10.0f &&
                   p_imu->tilt_roll_deg <= 10.0f))
            reason = HEIGHT_REJECT_ROLL;
        else if (!(distance >= -10000.0f && distance <= 10000.0f))
            reason = HEIGHT_REJECT_DISTANCE;
        else if (!(left >= -1.0f && left <= 1.0f) ||
                 !(right >= -1.0f && right <= 1.0f))
            reason = HEIGHT_REJECT_SPEED;
        else if (rmath_absf(left - right) > 0.20f)
            reason = HEIGHT_REJECT_TURN;
    }
    if (HEIGHT_REJECT_NONE != reason)
    {
        imu_height_invalidate(p_height, reason);
        return;
    }
    slope = -rmath_sinf(pitch * RMATH_DEG_TO_RAD);
    b_level = (rmath_absf(pitch) <= HEIGHT_LEVEL_DEG);
    if (!p_height->b_have_distance)
    {
        p_height->b_have_distance = true;
        p_height->last_distance_m = distance;
        p_height->last_ms = motion_ms;
        p_height->last_slope = slope;
        p_height->gap_pitch = pitch;
        return;
    }
    ds = distance - p_height->last_distance_m;
    p_height->last_distance_m = distance;
    p_height->last_ms = motion_ms;
    if ((interval == 0U) || (interval > HEIGHT_MAX_GAP_MS))
    {
        imu_height_invalidate(p_height, HEIGHT_REJECT_INTERVAL);
        return;
    }
    if (rmath_absf(ds) > (0.003f + (float) interval * 0.001f))
    {
        imu_height_invalidate(p_height, HEIGHT_REJECT_DISTANCE_JUMP);
        return;
    }
    if (p_height->b_gap)
    {
        uint32_t const duration = motion_ms - p_height->gap_anchor_ms;
        uint32_t idx;
        if (duration > 100U)
            reason = HEIGHT_REJECT_GAP_TIMEOUT;
        else if (p_height->gap_segments >= IMU_HEIGHT_GAP_SEGMENTS)
            reason = HEIGHT_REJECT_GAP_BUFFER;
        else if (p_height->gap_path_m + rmath_absf(ds) > 0.01f)
            reason = HEIGHT_REJECT_GAP_DISTANCE;
        else if (p_height->bridged_path_m + p_height->gap_path_m +
                 rmath_absf(ds) > 0.03f)
            reason = HEIGHT_REJECT_GAP_BUDGET;
        if (HEIGHT_REJECT_NONE != reason)
        {
            imu_height_invalidate(p_height, reason);
            return;
        }
        p_height->gap_ds[p_height->gap_segments] = ds;
        p_height->gap_ms[p_height->gap_segments++] = duration;
        p_height->gap_path_m += rmath_absf(ds);
        if (!p_imu->b_tilt_valid)
        {
            return;
        }
        if (rmath_absf(pitch - p_height->gap_pitch) > 3.0f)
        {
            imu_height_invalidate(p_height, HEIGHT_REJECT_GAP_PITCH);
            return;
        }
        /* Replay only after a valid endpoint exists; never integrate the
           invalid tilt values or count interpolated travel as level runout. */
        for (idx = 0U; idx < p_height->gap_segments; idx++)
        {
            float const interpolated = p_height->gap_slope +
                (slope - p_height->gap_slope) *
                ((float) p_height->gap_ms[idx] / (float) duration);
            bool const b_interpolated_level =
                rmath_absf(interpolated) <=
                rmath_sinf(HEIGHT_LEVEL_DEG * RMATH_DEG_TO_RAD);
            imu_height_integrate(p_height, p_height->gap_ds[idx],
                                  interpolated, b_interpolated_level);
            p_height->level_path_m = 0.0f;
            if (!p_height->b_ready) return;
        }
        p_height->bridged_path_m += p_height->gap_path_m;
        p_height->gap_count++;
        p_height->b_gap = false;
        p_height->gap_segments = 0U;
        p_height->b_valid = true;
        p_height->gap_pitch = pitch;
        return;
    }
    p_height->gap_pitch = pitch;
    if (!p_height->b_ready)
    {
        p_height->last_slope = slope;
        p_height->flat_samples = b_level ? p_height->flat_samples + 1U : 0U;
        p_height->b_ready = (p_height->flat_samples >= HEIGHT_FLAT_SAMPLES);
        p_height->b_valid = p_height->b_ready;
        return;
    }
    p_height->b_valid = true;
    imu_height_integrate(p_height, ds, slope, b_level);
    if (p_height->b_valid && p_height->b_active &&
        p_height->b_descended && (p_height->level_path_m >= 0.03f))
    {
        if ((p_height->peak_m >= HEIGHT_MIN_PEAK_M) &&
            (rmath_absf(p_height->rise_m) <=
             rmath_maxf(0.003f, p_height->peak_m * 0.35f)))
        {
            p_height->last_peak_m = p_height->peak_m;
            p_height->b_peak_valid = true;
            p_height->count++;
            p_height->b_active = false;
            p_height->b_descended = false;
            p_height->rise_m = 0.0f;
            p_height->peak_m = 0.0f;
            p_height->path_m = 0.0f;
            p_height->level_path_m = 0.0f;
            p_height->b_complete = true;
            p_height->bridged_path_m = 0.0f;
        }
        else
        {
            imu_height_invalidate(p_height, HEIGHT_REJECT_ENDPOINT);
        }
    }
}

void
imu_height_apply (imu_height_t const * p_height, imu_event_t * p_imu)
{
    p_imu->estimated_hump_height_m = p_height->last_peak_m;
    p_imu->b_hump_height_valid = p_height->b_peak_valid;
    p_imu->b_hump_active = p_height->b_active;
    p_imu->b_hump_complete = p_height->b_complete;
    p_imu->b_hump_rejected = p_height->b_rejected;
    p_imu->hump_count = p_height->count;
    p_imu->rejected_hump_count = p_height->rejected_count;
    p_imu->terrain_rise_m = p_height->rise_m;
    p_imu->terrain_peak_m = p_height->peak_m;
    p_imu->b_height_tracking_valid = p_height->b_valid;
    p_imu->b_height_uses_odometry = true;
    p_imu->b_height_gap_pending = p_height->b_gap;
    p_imu->height_gap_count = p_height->gap_count;
    p_imu->b_height_approximate = true;
    if ((MOTION_CLIMBING == p_imu->motion_state) ||
        (MOTION_DESCENDING == p_imu->motion_state))
    {
        p_imu->motion_state = MOTION_STATIONARY;
    }
    if (p_height->b_active && p_height->b_valid &&
        (rmath_absf(p_height->last_slope) >
         rmath_sinf(HEIGHT_LEVEL_DEG * RMATH_DEG_TO_RAD)) &&
        (MOTION_IMPACT != p_imu->motion_state))
    {
        if (p_height->last_dz < -0.00001f)
        {
            p_imu->motion_state = MOTION_DESCENDING;
        }
        else if (p_height->last_dz > 0.00001f)
        {
            p_imu->motion_state = MOTION_CLIMBING;
        }
    }
}
