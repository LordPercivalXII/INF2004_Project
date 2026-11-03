/** @file subsystem4_test.c
 * @brief LSM303DLHC processing/byte-decoding tests: TEST=1 or IMU_HOST_TEST.
 */
#include "imu_processing.h"
#include "drv_lsm303dlhc.h"
#include "imu_height.h"
#include "../common/rmath.h"

#if defined(IMU_HOST_TEST)
#include <stdio.h>
static int32_t g_host_failures;
#define TEST_INIT() int32_t test_pass = 0; int32_t test_fail = 0
#define TEST_CHECK(id, cond, desc) do { \
    if (cond) { test_pass++; printf("[TEST] %s PASS %s\n", id, desc); } \
    else { test_fail++; printf("[TEST] %s FAIL %s\n", id, desc); } \
} while (0)
#define TEST_SUMMARY(name) do { \
    printf("[TEST] %s SUMMARY %d passed, %d failed\n", \
           name, (int) test_pass, (int) test_fail); \
    g_host_failures = test_fail; \
} while (0)
#else
#include "../test_report.h"
#endif

#if APP_TEST || defined(IMU_HOST_TEST)
static imu_processor_t g_processor;
static uint32_t g_ms;
static imu_height_t g_height;
static imu_event_t g_height_imu;
static motion_event_t g_height_motion;
static uint32_t g_height_ms;

static void
s4_height_feed (float pitch, float ds, uint32_t count)
{
    uint32_t idx;
    for (idx = 0U; idx < count; idx++)
    {
        g_height_ms += 20U;
        g_height_imu.tilt_pitch_deg = pitch;
        g_height_motion.distance_m += ds;
        g_height_motion.left_speed_m_s = ds / 0.02f;
        g_height_motion.right_speed_m_s = ds / 0.02f;
        imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                          g_height_ms, g_height_ms);
        imu_height_apply(&g_height, &g_height_imu);
    }
}

static void
s4_height_baseline (void)
{
    imu_height_init(&g_height);
    g_height_imu = (imu_event_t) {
        .b_sensor_valid = true, .b_calibrated = true, .b_tilt_valid = true,
        .acceleration_z_g = 1.0f
    };
    g_height_motion = (motion_event_t) {0};
    g_height_ms = 0U;
    s4_height_feed(0.0f, 0.0f, 12U);
}

static imu_sample_t
s4_flat (void)
{
    imu_sample_t const sample = {
        .acceleration = {0.0f, 0.0f, 1.0f},
        .magnetic_ut = {30.0f, 0.0f, 20.0f},
        .b_magnetic_valid = true,
        .b_magnetic_fresh = true
    };
    return (sample);
}

static void
s4_feed (imu_sample_t const * p_sample, uint32_t count)
{
    uint32_t idx;
    for (idx = 0U; idx < count; idx++)
    {
        g_ms += 10U;
        imu_processor_update(&g_processor, p_sample, true, g_ms);
    }
}

static void
s4_calibrate (void)
{
    imu_sample_t const flat = s4_flat();
    imu_processor_init(&g_processor);
    g_ms = 0U;
    s4_feed(&flat, IMU_CALIBRATION_SAMPLES + 30U);
}

static void
s4_rotate (float start_deg, float rate_dps, uint32_t count)
{
    imu_sample_t sample = s4_flat();
    uint32_t idx;
    for (idx = 0U; idx < count; idx++)
    {
        float const angle =
            (start_deg + (float) idx * rate_dps * 0.01f) * RMATH_DEG_TO_RAD;
        sample.magnetic_ut[0] = 30.0f * rmath_cosf(angle);
        sample.magnetic_ut[1] = -30.0f * rmath_sinf(angle);
        s4_feed(&sample, 1U);
    }
}

static bool
s4_hump (float acceleration_g, bool b_jitter, float direction)
{
    imu_sample_t sample = s4_flat();
    bool b_climbed = false;
    bool b_descended = false;
    uint32_t idx;
    for (idx = 0U; idx < 80U; idx++)
    {
        sample.acceleration[2] = 1.0f + direction *
            (((idx < 20U) || (idx >= 60U)) ? acceleration_g : -acceleration_g);
        g_ms += (b_jitter && ((idx % 2U) != 0U)) ? 20U : 10U;
        imu_processor_update(&g_processor, &sample, true, g_ms);
        b_climbed = b_climbed || (MOTION_CLIMBING == g_processor.output.motion_state);
        b_descended = b_descended ||
            (MOTION_DESCENDING == g_processor.output.motion_state);
    }
    sample.acceleration[2] = 1.0f;
    s4_feed(&sample, 25U);
    return (b_climbed && b_descended);
}

void
subsystem4_test (void)
{
    imu_filter_t filter;
    imu_sample_t sample = s4_flat();
    float vector[3];
    float average = 0.0f;
    float roll;
    float pitch;
    float height;
    uint32_t idx;
    bool b_rejected;
    uint8_t const accel_bytes[6] = {0x40U,0x1FU,0xC0U,0xE0U,0xF0U,0x7FU};
    uint8_t mag_bytes[6] = {0x04U,0x4CU,0x03U,0xD4U,0xFBU,0xB4U};
    union { uint32_t bits; float value; } nonfinite;

    TEST_INIT();
    imu_filter_init(&filter);
    for (idx = 0U; idx < 10U; idx++)
    {
        average = imu_filter_update(&filter, (float) idx);
    }
    TEST_CHECK("S4-U01", (rmath_absf(average - 7.0f) < 0.001f),
               "moving average and ring wrap");
    imu_tilt_angles(0.0f, 0.70711f, 0.70711f, &roll, &pitch);
    TEST_CHECK("S4-U02", ((rmath_absf(roll - 45.0f) < 0.1f) &&
                          (rmath_absf(pitch) < 0.1f)), "45 degree roll");
    imu_tilt_angles(-0.5f, 0.0f, 0.8660254f, &roll, &pitch);
    TEST_CHECK("S4-U03", (rmath_absf(pitch - 30.0f) < 0.1f),
               "30 degree pitch");
    lsm303_decode_accel(accel_bytes, vector);
    TEST_CHECK("S4-U04", ((rmath_absf(vector[0] - 1.0f) < 0.001f) &&
                          (rmath_absf(vector[1] + 1.0f) < 0.001f) &&
                          (rmath_absf(vector[2] - 4.094f) < 0.001f)),
               "12-bit left-aligned little-endian 4g acceleration scale");
    TEST_CHECK("S4-U05", (lsm303_decode_mag(mag_bytes, vector) &&
                          (rmath_absf(vector[0] - 100.0f) < 0.01f) &&
                          (rmath_absf(vector[1] + 100.0f) < 0.01f) &&
                          (rmath_absf(vector[2] - 100.0f) < 0.01f)),
               "magnetometer big-endian X,Z,Y and separate Z scale");
    mag_bytes[0] = 0xF0U;
    mag_bytes[1] = 0x00U;
    TEST_CHECK("S4-U06", !lsm303_decode_mag(mag_bytes, vector),
               "magnetometer overflow sentinel rejected");

    imu_processor_init(&g_processor);
    g_ms = 0U;
    sample.acceleration[0] = 0.02f;
    sample.acceleration[1] = -0.01f;
    sample.acceleration[2] = 1.03f;
    s4_feed(&sample, 199U);
    TEST_CHECK("S4-U07", (!g_processor.output.b_calibrated &&
                          (MOTION_UNKNOWN == g_processor.output.motion_state)),
               "requires 200 stable acceleration samples");
    s4_feed(&sample, 31U);
    TEST_CHECK("S4-U08", (g_processor.output.b_calibrated &&
        (rmath_absf(g_processor.output.acceleration_x_g) < 0.001f) &&
        (rmath_absf(g_processor.output.acceleration_z_g - 1.0f) < 0.001f)),
        "level accelerometer bias calibration");
    TEST_CHECK("S4-U09", (!g_processor.output.b_has_gyro &&
                          !g_processor.output.b_mag_calibrated &&
                          g_processor.output.b_turn_rate_estimated &&
                          g_processor.output.b_height_approximate),
               "no fabricated gyro or magnetic calibration claims");
    imu_processor_init(&g_processor);
    sample = s4_flat();
    s4_feed(&sample, 100U);
    sample.acceleration[0] = 0.3f;
    s4_feed(&sample, 1U);
    sample = s4_flat();
    s4_feed(&sample, 199U);
    TEST_CHECK("S4-U10", !g_processor.output.b_calibrated,
               "movement restarts calibration");
    s4_feed(&sample, 1U);
    TEST_CHECK("S4-U11", g_processor.output.b_calibrated,
               "calibration recovers after movement");

    s4_calibrate();
    s4_feed(&sample, 300U);
    TEST_CHECK("S4-U12", ((MOTION_STATIONARY == g_processor.output.motion_state) &&
                          !g_processor.output.b_hump_height_valid),
               "quiet flat data does not produce humps");
    s4_rotate(0.0f, 30.0f, 100U);
    TEST_CHECK("S4-U13", (g_processor.output.b_turn_rate_valid &&
                          (MOTION_TURNING == g_processor.output.motion_state) &&
                          (rmath_absf(g_processor.output.turn_rate_dps - 30.0f) < 0.2f)),
               "magnetic heading derivative detects sustained positive turn");
    s4_rotate(30.0f, -30.0f, 100U);
    TEST_CHECK("S4-U14", (g_processor.output.b_turn_rate_valid &&
                          (g_processor.output.turn_rate_dps < -29.8f)),
               "negative magnetic turn rate");
    s4_calibrate();
    s4_rotate(0.0f, -30.0f, 100U);
    TEST_CHECK("S4-U15", (g_processor.output.b_turn_rate_valid &&
                          (g_processor.output.turn_rate_dps < -29.8f)),
               "heading wrap 0 to 359 does not create spike");
    sample = s4_flat();
    sample.b_magnetic_fresh = false;
    s4_feed(&sample, 20U);
    TEST_CHECK("S4-U16", (!g_processor.output.b_heading_valid &&
                          !g_processor.output.b_turn_rate_valid &&
                          (0.0f == g_processor.output.turn_rate_dps)),
               "stale magnetometer invalidates rate rather than reporting zero turn");
    s4_calibrate();
    sample = s4_flat();
    sample.magnetic_ut[0] = 80.0f;
    s4_feed(&sample, 1U);
    TEST_CHECK("S4-U17", (g_processor.output.b_magnetic_disturbed &&
                          !g_processor.output.b_turn_rate_valid &&
                          g_processor.output.b_sensor_valid),
               "field disturbance disables turns, not acceleration");
    sample = s4_flat();
    s4_feed(&sample, 30U);
    TEST_CHECK("S4-U18", (g_processor.output.b_heading_valid &&
                          g_processor.output.b_turn_rate_valid &&
                          !g_processor.output.b_magnetic_disturbed),
               "heading recovers after field returns to baseline");
    sample.magnetic_ut[0] = 0.0f;
    sample.magnetic_ut[1] = -30.0f;
    s4_feed(&sample, 1U);
    TEST_CHECK("S4-U19", (g_processor.output.b_magnetic_disturbed &&
                          !g_processor.output.b_turn_rate_valid),
               "implausible heading jump rejected");
    sample = s4_flat();
    sample.b_magnetic_valid = false;
    s4_feed(&sample, 1U);
    TEST_CHECK("S4-U20", (!g_processor.output.b_heading_valid &&
                          g_processor.output.b_sensor_valid),
               "magnetic transfer failure cannot masquerade as fresh heading");

    s4_calibrate();
    sample = s4_flat();
    sample.acceleration[0] = 0.3f;
    s4_feed(&sample, 15U);
    TEST_CHECK("S4-U21", ((MOTION_ACCELERATING == g_processor.output.motion_state) &&
                          !g_processor.output.b_turn_rate_valid),
               "transient horizontal acceleration; magnetic tilt rejected");
    sample.acceleration[0] = 3.2f;
    s4_feed(&sample, 1U);
    TEST_CHECK("S4-U22", (g_processor.output.b_shock_detected &&
                          (MOTION_IMPACT == g_processor.output.motion_state)),
               "impact reported immediately without smoothing");
    sample.acceleration[0] = 0.0f;
    s4_feed(&sample, 5U);
    TEST_CHECK("S4-U23", (MOTION_IMPACT == g_processor.output.motion_state),
               "impact classification holds briefly");
    s4_calibrate();
    sample = s4_flat();
    sample.acceleration[1] = 0.7071068f;
    sample.acceleration[2] = 0.7071068f;
    s4_feed(&sample, 300U);
    TEST_CHECK("S4-U24", ((rmath_absf(g_processor.output.tilt_roll_deg - 45.0f) < 0.2f) &&
                          g_processor.output.b_tilt_valid &&
                          !g_processor.output.b_hump_height_valid &&
                          !g_processor.output.b_heading_valid),
               "static roll settles without false hump; steep magnetic tilt rejected");
    s4_calibrate();
    TEST_CHECK("S4-U25", s4_hump(0.25f, false, 1.0f),
               "complete hump includes climbing and descending classes");
    height = g_processor.output.estimated_hump_height_m;
    TEST_CHECK("S4-U26", (g_processor.output.b_hump_height_valid &&
                          (1U == g_processor.output.hump_count) &&
                          (rmath_absf(height - 0.0980665f) < 0.003f)),
               "ideal flat-axis 9.81 cm hump integration within 3 mm");
    sample = s4_flat();
    s4_feed(&sample, 300U);
    TEST_CHECK("S4-U27", ((1U == g_processor.output.hump_count) &&
                          (height == g_processor.output.estimated_hump_height_m)),
               "last completed peak retained, no duplicate hump events");
    (void) s4_hump(0.12f, false, 1.0f);
    TEST_CHECK("S4-U28", ((2U == g_processor.output.hump_count) &&
        (rmath_absf(g_processor.output.estimated_hump_height_m - 0.0470719f) < 0.003f)),
        "second hump measured separately");
    s4_calibrate();
    (void) s4_hump(0.25f, true, 1.0f);
    TEST_CHECK("S4-U29", (g_processor.output.b_hump_height_valid &&
        (rmath_absf(g_processor.output.estimated_hump_height_m - 0.2206496f) < 0.004f)),
        "actual sample intervals used in height estimate");
    s4_calibrate();
    (void) s4_hump(0.25f, false, -1.0f);
    TEST_CHECK("S4-U30", !g_processor.output.b_hump_height_valid,
               "pothole is not a positive hump");
    s4_calibrate();
    sample = s4_flat();
    sample.acceleration[2] = 1.2f;
    s4_feed(&sample, 20U);
    g_ms += 10U;
    imu_processor_update(&g_processor, &sample, false, g_ms);
    TEST_CHECK("S4-U31", (!g_processor.output.b_sensor_valid &&
                          g_processor.output.b_hump_rejected &&
                          !g_processor.output.b_heading_valid &&
                          (MOTION_UNKNOWN == g_processor.output.motion_state)),
               "I2C failure aborts terrain and heading");
    sample = s4_flat();
    s4_feed(&sample, 30U);
    TEST_CHECK("S4-U32", (g_processor.output.b_sensor_valid &&
                          !g_processor.output.b_hump_height_valid),
               "recovery does not invent a peak");
    sample.acceleration[2] = 1.2f;
    s4_feed(&sample, 20U);
    g_ms += 200U;
    imu_processor_update(&g_processor, &sample, true, g_ms);
    TEST_CHECK("S4-U33", (g_processor.output.b_sensor_valid &&
                          g_processor.output.b_timing_gap &&
                          !g_processor.output.b_tilt_valid &&
                          g_processor.output.b_hump_rejected),
               "missed sampling deadline aborts reference integration without claiming sensor failure");
    s4_calibrate();
    sample = s4_flat();
    sample.acceleration[2] = 1.2f;
    b_rejected = false;
    for (idx = 0U; idx < 410U; idx++)
    {
        s4_feed(&sample, 1U);
        b_rejected = b_rejected || g_processor.output.b_hump_rejected;
    }
    TEST_CHECK("S4-U34", (b_rejected && !g_processor.output.b_hump_height_valid),
               "unbounded acceleration times out without height");
    s4_calibrate();
    sample = s4_flat();
    sample.acceleration[0] = 4.094f;
    s4_feed(&sample, 1U);
    TEST_CHECK("S4-U35", (g_processor.output.b_saturated &&
                          g_processor.output.b_sensor_valid &&
                          g_processor.output.b_shock_detected &&
                          !g_processor.output.b_hump_active),
               "real LSM303 full-scale clipping flagged");
    nonfinite.bits = 0x7FC00000U;
    sample.acceleration[0] = nonfinite.value;
    s4_feed(&sample, 1U);
    TEST_CHECK("S4-U36", !g_processor.output.b_sensor_valid, "NaN rejected");
    s4_calibrate();
    sample = s4_flat();
    sample.magnetic_ut[0] = nonfinite.value;
    s4_feed(&sample, 1U);
    TEST_CHECK("S4-U37", (!g_processor.output.b_heading_valid &&
                          g_processor.output.b_sensor_valid),
               "magnetic NaN cannot poison acceleration");
    s4_calibrate();
    sample = s4_flat();
    g_ms = 0xFFFFFFF0U;
    g_processor.last_ms = g_ms;
    g_processor.heading_ms = g_ms;
    s4_feed(&sample, 10U);
    TEST_CHECK("S4-U38", (g_processor.output.b_sensor_valid &&
                          g_processor.output.b_turn_rate_valid),
               "millisecond timer wrap handled");
    s4_calibrate();
    sample.acceleration[0] = 3.2f;
    s4_feed(&sample, 5U);
    TEST_CHECK("S4-U39", (1U == g_processor.output.shock_count),
               "sustained impact counted once");
    sample = s4_flat();
    s4_feed(&sample, 30U);
    sample.acceleration[0] = 3.2f;
    s4_feed(&sample, 1U);
    TEST_CHECK("S4-U40", (2U == g_processor.output.shock_count),
               "second impact counted for low-rate telemetry");
    sample.acceleration[0] = -4.096f;
    s4_feed(&sample, 1U);
    TEST_CHECK("S4-U41", (g_processor.output.b_sensor_valid &&
                          g_processor.output.b_saturated &&
                          g_processor.output.b_shock_detected),
               "negative full-scale clipping treated symmetrically");
    s4_height_baseline();
    TEST_CHECK("S4-H01", g_height_imu.b_height_tracking_valid,
               "level-ground baseline establishes encoder height tracking");
    s4_height_feed(0.0f, 0.005f, 100U);
    TEST_CHECK("S4-H02", (!g_height.b_active && g_height.count == 0U &&
                          g_height.rise_m == 0.0f),
               "flat travel does not invent a hump");
    s4_height_feed(-20.0f, 0.0f, 100U);
    s4_height_feed(20.0f, 0.0f, 100U);
    TEST_CHECK("S4-H03", (!g_height.b_active && g_height.count == 0U),
               "manual tilt without wheel travel does not create height");
    s4_height_feed(0.0f, 0.0f, 1U);
    s4_height_feed(-10.0f, 0.005f, 100U);
    TEST_CHECK("S4-H04", (g_height.b_active && g_height.b_valid &&
                          rmath_absf(g_height.rise_m - 0.08639f) < 0.001f &&
                          g_height_imu.motion_state == MOTION_CLIMBING),
               "0.5 m surface travel at 10 degrees gives about 8.64 cm rise");
    {
        float const rise = g_height.rise_m;
        s4_height_feed(-10.0f, 0.0f, 100U);
        s4_height_feed(0.0f, 0.0f, 20U);
        s4_height_feed(0.0f, 0.005f, 20U);
        TEST_CHECK("S4-H05", (g_height.b_active && g_height.count == 0U &&
                              rmath_absf(g_height.rise_m - rise) < 0.00001f),
                   "stop or elevated plateau retains rise without false completion");
    }
    s4_height_feed(10.0f, 0.005f, 20U);
    TEST_CHECK("S4-H06", (g_height_imu.motion_state == MOTION_DESCENDING),
               "rolling down the opposite slope classified as descending");
    s4_height_feed(10.0f, 0.005f, 80U);
    s4_height_feed(0.0f, 0.005f, 10U);
    TEST_CHECK("S4-H07", (g_height.count == 1U && !g_height.b_active &&
                          g_height_imu.b_hump_height_valid &&
                          rmath_absf(g_height.last_peak_m - 0.08639f) < 0.001f),
               "completed up/down hump reports peak once after level runout");
    s4_height_feed(0.0f, 0.005f, 50U);
    TEST_CHECK("S4-H08", (g_height.count == 1U &&
                          g_height.last_peak_m > 0.085f),
               "last completed peak retained without duplicate counts");
    s4_height_feed(-10.0f, 0.005f, 100U);
    s4_height_feed(10.0f, 0.005f, 100U);
    s4_height_feed(0.0f, 0.005f, 10U);
    TEST_CHECK("S4-H09", g_height.count == 2U,
               "consecutive humps counted separately");
    s4_height_baseline();
    s4_height_feed(10.0f, -0.005f, 100U);
    TEST_CHECK("S4-H10", (g_height.rise_m > 0.085f &&
                          g_height_imu.motion_state == MOTION_CLIMBING),
               "signed odometry handles reversing uphill");
    s4_height_baseline();
    s4_height_feed(10.0f, 0.005f, 100U);
    TEST_CHECK("S4-H11", (!g_height.b_active && g_height.count == 0U),
               "initial downhill-only travel is not a positive hump");
    s4_height_baseline();
    s4_height_feed(-10.0f, 0.005f, 20U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(-10.0f, 0.0f, 13U);
    TEST_CHECK("S4-H12", (!g_height.b_valid && !g_height.b_active &&
                          g_height.rejected_count == 1U &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_TIMEOUT),
               "persistent invalid tilt rejects window and prevents restart");
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(-10.0f, 0.005f, 20U);
    TEST_CHECK("S4-H13", !g_height.b_ready,
               "cannot restart height partway up a slope after data loss");
    s4_height_feed(0.0f, 0.0f, 12U);
    TEST_CHECK("S4-H14", g_height.b_valid,
               "level-ground baseline can be re-established after rejection");
    s4_height_feed(-10.0f, 0.005f, 20U);
    g_height_ms += 101U;
    s4_height_feed(-10.0f, 0.005f, 1U);
    TEST_CHECK("S4-H15", (!g_height.b_valid && g_height.rejected_count == 2U &&
                          g_height.last_rejection == HEIGHT_REJECT_INTERVAL),
               "missing odometry interval rejects rather than integrating a gap");
    s4_height_baseline();
    imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                      g_height_ms - 101U, g_height_ms + 20U);
    TEST_CHECK("S4-H16", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_IMU_AGE),
               "stale tilt is not fused with wheel travel");
    s4_height_baseline();
    g_height_motion.left_speed_m_s = -0.3f;
    g_height_motion.right_speed_m_s = 0.3f;
    imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                      g_height_ms + 20U, g_height_ms + 20U);
    TEST_CHECK("S4-H17", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_TURN),
               "turn-in-place rejected");
    s4_height_baseline();
    s4_height_feed(-10.0f, 0.005f, 20U);
    s4_height_feed(-10.0f, 0.5f, 1U);
    TEST_CHECK("S4-H18", (!g_height.b_valid && g_height.rejected_count == 1U &&
                          g_height.last_rejection == HEIGHT_REJECT_SPEED),
               "implausible encoder jump/speed rejected");
    s4_height_baseline();
    g_height_imu.tilt_roll_deg = 11.0f;
    s4_height_feed(-10.0f, 0.005f, 1U);
    TEST_CHECK("S4-H19", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_ROLL),
               "excessive cross-slope rejected");
    s4_height_baseline();
    s4_height_feed(-36.0f, 0.005f, 1U);
    TEST_CHECK("S4-H20", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_PITCH),
               "steep pitch outside supported geometry rejected");
    s4_height_baseline();
    g_height_motion.distance_m = nonfinite.value;
    s4_height_feed(0.0f, 0.0f, 1U);
    TEST_CHECK("S4-H21", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_DISTANCE),
               "non-finite wheel distance rejected");
    s4_height_baseline();
    g_height_ms = 0xFFFFFFF0U;
    g_height.last_ms = g_height_ms;
    s4_height_feed(-10.0f, 0.005f, 10U);
    TEST_CHECK("S4-H22", (g_height.b_valid && g_height.rise_m > 0.0f),
               "odometry height handles millisecond wrap");
    s4_height_baseline();
    s4_height_feed(-10.0f, 0.005f, 100U);
    s4_height_feed(10.0f, 0.005f, 20U);
    s4_height_feed(0.0f, 0.005f, 10U);
    TEST_CHECK("S4-H23", (g_height.count == 0U &&
                          g_height.rejected_count == 1U),
               "partial descent onto a different plateau is not a complete hump");
    s4_height_baseline();
    s4_height_feed(-10.0f, 0.005f, 20U);
    imu_height_invalidate(&g_height, HEIGHT_REJECT_SENSOR);
    imu_height_invalidate(&g_height, HEIGHT_REJECT_SENSOR);
    TEST_CHECK("S4-H24", (g_height.rejected_count == 1U),
               "repeated invalidation counts one discarded window");
    s4_calibrate();
    g_processor.b_odometry_height = true;
    sample = s4_flat();
    sample.acceleration[2] = 1.25f;
    s4_feed(&sample, 10U);
    TEST_CHECK("S4-H25", (!g_processor.output.b_hump_active &&
                          g_processor.output.hump_count == 0U),
               "robot mode disables competing accelerometer-only hump estimator");
    s4_height_baseline();
    g_height_imu.b_sensor_valid = false;
    s4_height_feed(-10.0f, 0.005f, 1U);
    TEST_CHECK("S4-H26", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_SENSOR),
               "sensor failure disables height");
    s4_height_baseline();
    g_height_imu.b_saturated = true;
    s4_height_feed(-10.0f, 0.005f, 1U);
    TEST_CHECK("S4-H27", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_SATURATION),
               "clipped acceleration disables height");
    s4_height_baseline();
    g_height_imu.b_shock_detected = true;
    s4_height_feed(-10.0f, 0.005f, 1U);
    TEST_CHECK("S4-H28", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_SHOCK),
               "impact disables height");
    s4_height_baseline();
    g_height_imu.b_calibrated = false;
    s4_height_feed(-10.0f, 0.005f, 1U);
    TEST_CHECK("S4-H29", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_CALIBRATION),
               "uncalibrated tilt cannot provide height");
    s4_height_baseline();
    s4_height_feed(nonfinite.value, 0.005f, 1U);
    TEST_CHECK("S4-H30", !g_height.b_valid, "NaN pitch rejected");
    s4_height_baseline();
    imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                      g_height_ms + 40U, g_height_ms + 20U);
    TEST_CHECK("S4-H31", g_height.b_valid,
               "nearby newer sensor event accepted despite producer enqueue ordering");
    s4_height_baseline();
    s4_height_feed(10.0f, 0.005f, 50U);
    s4_height_feed(-10.0f, 0.005f, 50U);
    TEST_CHECK("S4-H32", (!g_height.b_active && !g_height.b_ready &&
                          g_height.count == 0U),
               "down/up dip cannot invent a positive hump above the original datum");
    s4_height_baseline();
    s4_calibrate();
    g_processor.b_odometry_height = true;
    {
        float truth = 0.0f;
        float truth_peak = 0.0f;
        float previous_slope = 0.0f;
        /* 0 -> 10 -> 0 -> -10 -> 0 degrees, with gentle transitions.
           Feed the actual accelerometer tilt filter before height fusion. */
        for (idx = 0U; idx < 600U; idx++)
        {
            float angle = 0.0f;
            float slope;
            if (idx < 100U) angle = (float) idx * 0.1f;
            else if (idx < 200U) angle = 10.0f;
            else if (idx < 400U) angle = 10.0f - (float) (idx - 200U) * 0.1f;
            else if (idx < 500U) angle = -10.0f;
            else angle = -10.0f + (float) (idx - 500U) * 0.1f;
            slope = rmath_sinf(angle * RMATH_DEG_TO_RAD);
            sample = s4_flat();
            sample.acceleration[0] = slope;
            sample.acceleration[2] = rmath_cosf(angle * RMATH_DEG_TO_RAD);
            s4_feed(&sample, 2U);
            g_height_imu = g_processor.output;
            s4_height_feed(g_processor.output.tilt_pitch_deg, 0.002f, 1U);
            truth += 0.002f * (previous_slope + slope) * 0.5f;
            truth_peak = rmath_maxf(truth_peak, truth);
            previous_slope = slope;
        }
        for (idx = 0U; idx < 100U; idx++)
        {
            sample = s4_flat();
            s4_feed(&sample, 2U);
            g_height_imu = g_processor.output;
            s4_height_feed(g_processor.output.tilt_pitch_deg, 0.002f, 1U);
        }
        TEST_CHECK("S4-H33", (g_height.count == 1U &&
                              g_height.rejected_count == 0U &&
                              rmath_absf(g_height.last_peak_m - truth_peak) <
                              truth_peak * 0.1f),
                   "real tilt-filter plus rolling-distance path completes gentle hump within 10 percent in ideal data");
    }
    s4_height_baseline();
    s4_height_feed(-2.94f, 0.00195f, 100U);
    TEST_CHECK("S4-H34", (g_height.b_active &&
                          g_height_imu.motion_state == MOTION_CLIMBING &&
                          rmath_absf(g_height.rise_m - 0.01f) < 0.0002f),
               "1 cm high 19.5 cm shallow ascent is not hidden by level deadband");
    s4_height_feed(2.94f, 0.00195f, 100U);
    TEST_CHECK("S4-H35", g_height_imu.motion_state == MOTION_DESCENDING,
               "shallow downhill segment is classified as descending");
    s4_height_feed(0.0f, 0.00195f, 30U);
    TEST_CHECK("S4-H36", (g_height.count == 1U &&
                          rmath_absf(g_height.last_peak_m - 0.01f) < 0.0002f),
               "1 cm up/down hump completes with measured peak near 0.010 m");
    s4_height_baseline();
    for (idx = 0U; idx < 200U; idx++)
    {
        s4_height_feed((idx % 2U) ? 0.6f : -0.6f, 0.002f, 1U);
    }
    TEST_CHECK("S4-H37", (!g_height.b_active && g_height.count == 0U),
               "sub-degree level noise does not create terrain events");
    s4_height_baseline();
    s4_height_feed(-2.94f, 0.00195f, 100U);
    s4_height_feed(2.94f, 0.00195f, 55U);
    s4_height_feed(0.0f, 0.00195f, 30U);
    TEST_CHECK("S4-H38", (g_height.count == 0U &&
                          g_height.rejected_count == 1U &&
                          g_height.last_rejection == HEIGHT_REJECT_ENDPOINT),
               "1 cm hump cannot pass endpoint check after only partial descent");
    s4_height_baseline();
    s4_calibrate();
    g_processor.b_odometry_height = true;
    for (idx = 0U; idx < 500U; idx++)
    {
        float const angle = (idx < 100U) ? 2.94f
                            : ((idx < 200U) ? -2.94f : 0.0f);
        sample = s4_flat();
        sample.acceleration[0] = rmath_sinf(angle * RMATH_DEG_TO_RAD);
        sample.acceleration[2] = rmath_cosf(angle * RMATH_DEG_TO_RAD);
        s4_feed(&sample, 2U);
        g_height_imu = g_processor.output;
        s4_height_feed(g_processor.output.tilt_pitch_deg, 0.00195f, 1U);
    }
    TEST_CHECK("S4-H39", (g_height.count == 1U &&
                          g_height.rejected_count == 0U &&
                          rmath_absf(g_height.last_peak_m - 0.01f) < 0.002f),
               "actual tilt filter resolves 1 cm shallow hump within 2 mm in ideal data");
    s4_height_baseline();
    imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                      g_height_ms, g_height_ms);
    TEST_CHECK("S4-H40", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_INTERVAL),
               "duplicate motion timestamp identified without inventing distance");
    s4_height_baseline();
    g_height_motion.distance_m += 0.03f;
    imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                      g_height_ms + 20U, g_height_ms + 20U);
    TEST_CHECK("S4-H41", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_DISTANCE_JUMP),
               "distance jump identified independently of wheel speed gate");
    s4_height_baseline();
    s4_height_feed(10.0f, 0.005f, 2U);
    TEST_CHECK("S4-H42",
               g_height.last_rejection == HEIGHT_REJECT_DOWNHILL_START,
               "downhill-start datum loss has a distinct reason");
    s4_height_baseline();
    s4_height_feed(-10.0f, 0.005f, 1U);
    g_height.path_m = 10.0f;
    s4_height_feed(-10.0f, 0.005f, 1U);
    TEST_CHECK("S4-H43", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_PATH),
               "excessive crossing length has a distinct reason");
    s4_height_baseline();
    s4_height_feed(-10.0f, 0.005f, 1U);
    g_height.rise_m = -0.051f;
    s4_height_feed(0.0f, 0.0f, 1U);
    TEST_CHECK("S4-H44", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_RISE),
               "excessive negative rise has a distinct reason");
    s4_height_baseline();
    s4_height_feed(-10.0f, 0.005f, 1U);
    g_height.peak_m = 1.01f;
    s4_height_feed(-10.0f, 0.005f, 1U);
    TEST_CHECK("S4-H45", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_PEAK),
               "excessive peak height has a distinct reason");
    s4_height_baseline();
    s4_height_feed(-10.0f, 0.005f, 1U);
    g_height_imu.b_tilt_valid = false;
    g_height_imu.b_saturated = true;
    s4_height_feed(-10.0f, 0.005f, 1U);
    TEST_CHECK("S4-H46", (g_height.rejected_count == 1U &&
                          g_height.last_rejection == HEIGHT_REJECT_SATURATION &&
                          imu_height_imu_rejection(&g_height_imu) ==
                          HEIGHT_REJECT_SATURATION),
               "controller and motion fusion use the same primary sensor rejection");
    g_height_imu.b_tilt_valid = true;
    g_height_imu.b_saturated = false;
    s4_height_feed(0.0f, 0.0f, 12U);
    TEST_CHECK("S4-H47", (g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_SATURATION &&
                          imu_height_imu_rejection(&g_height_imu) ==
                          HEIGHT_REJECT_NONE),
               "recovery preserves diagnostic cause but restores healthy sensor status");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 10U);
    {
        float const rise = g_height.rise_m;
        g_height_imu.b_tilt_valid = false;
        imu_height_observe_imu(&g_height, &g_height_imu, g_height_ms + 10U);
        imu_height_apply(&g_height, &g_height_imu);
        TEST_CHECK("S4-G01", (g_height.b_gap && !g_height.b_valid &&
                              g_height.b_active &&
                              g_height_imu.b_height_gap_pending &&
                              !g_height_imu.b_height_tracking_valid &&
                              g_height.rejected_count == 0U),
                   "IMU-side tilt gap preserves crossing but explicitly invalidates live height");
        s4_height_feed(-30.0f, 0.001f, 3U);
        TEST_CHECK("S4-G02", (g_height.rise_m == rise &&
                              !g_height.b_complete && g_height.gap_count == 0U),
                   "pending gap ignores invalid pitch and freezes height without completing");
        g_height_imu.b_tilt_valid = true;
        s4_height_feed(-3.0f, 0.001f, 1U);
        TEST_CHECK("S4-G03", (g_height.b_valid && !g_height.b_gap &&
                              g_height.gap_count == 1U &&
                              g_height.rejected_count == 0U &&
                              rmath_absf(g_height.rise_m - rise -
                              0.004f * rmath_sinf(3.0f * RMATH_DEG_TO_RAD)) <
                              0.000001f &&
                              g_height_imu.height_gap_count == 1U),
                   "80 ms constant-slope gap recovers all buffered distance exactly");
    }
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_tilt_valid = false;
    g_height_ms += 10U;
    imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                      g_height_ms, g_height_ms);
    s4_height_feed(-3.0f, 0.001f, 11U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(-3.0f, 0.001f, 1U);
    TEST_CHECK("S4-G04", (g_height.b_valid && g_height.gap_count == 1U),
               "250 ms bracket is accepted at duration limit");
    g_height_imu.b_tilt_valid = false;
    imu_height_observe_imu(&g_height, &g_height_imu, g_height_ms + 251U);
    TEST_CHECK("S4-G05", (!g_height.b_valid && !g_height.b_gap &&
                          g_height.rejected_count == 1U &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_TIMEOUT),
               "251 ms gap rejects immediately on IMU path even without new wheel event");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(-3.0f, 0.011f, 2U);
    TEST_CHECK("S4-G06", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_DISTANCE),
               "gap with more than 20 mm wheel travel is rejected");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(-9.01f, 0.001f, 1U);
    TEST_CHECK("S4-G07", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_PITCH),
               "endpoint pitch change over 6 degrees cannot be bridged");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 1U);
    for (idx = 0U; idx < 8U; idx++)
    {
        g_height_imu.b_tilt_valid = false;
        s4_height_feed(-3.0f, 0.004f, 1U);
        g_height_imu.b_tilt_valid = true;
        s4_height_feed(-3.0f, 0.004f, 1U);
    }
    TEST_CHECK("S4-G08", (!g_height.b_valid && g_height.gap_count == 7U &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_BUDGET),
               "repeated small gaps cannot exceed 60 mm interpolated-travel budget");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_sensor_valid = false;
    imu_height_observe_imu(&g_height, &g_height_imu, g_height_ms + 10U);
    TEST_CHECK("S4-G09", (!g_height.b_gap && !g_height.b_active &&
                          g_height.last_rejection == HEIGHT_REJECT_SENSOR),
               "sensor failure during gap immediately discards crossing");
    s4_height_baseline();
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(0.0f, 0.0f, 1U);
    g_height_imu.b_shock_detected = true;
    s4_height_feed(0.0f, 0.0f, 1U);
    TEST_CHECK("S4-G10", (!g_height.b_gap &&
                          g_height.last_rejection == HEIGHT_REJECT_SHOCK),
               "shock cannot be hidden by tilt-gap handling");
    s4_height_baseline();
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(0.0f, 0.0f, 1U);
    g_height_imu.b_saturated = true;
    s4_height_feed(0.0f, 0.0f, 1U);
    TEST_CHECK("S4-G11", (!g_height.b_gap &&
                          g_height.last_rejection == HEIGHT_REJECT_SATURATION),
               "clipping cannot be hidden by tilt-gap handling");
    s4_height_baseline();
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(0.0f, 0.0f, 1U);
    g_height_motion.left_speed_m_s = 0.3f;
    imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                      g_height_ms + 20U, g_height_ms + 20U);
    TEST_CHECK("S4-G12", (!g_height.b_gap &&
                          g_height.last_rejection == HEIGHT_REJECT_TURN),
               "wheel turning gate remains enforced during tilt gap");
    imu_height_init(&g_height);
    g_height_imu = (imu_event_t) {
        .b_sensor_valid = true, .b_calibrated = true, .b_tilt_valid = false,
        .acceleration_z_g = 1.0f
    };
    s4_height_feed(0.0f, 0.0f, 10U);
    TEST_CHECK("S4-G13", (!g_height.b_ready && !g_height.b_gap &&
                          g_height.gap_count == 0U),
               "invalid tilt cannot establish initial level baseline");
    s4_height_baseline();
    g_height_ms = 0xFFFFFFC0U;
    g_height.last_ms = g_height_ms;
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(0.0f, 0.001f, 3U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(0.0f, 0.001f, 1U);
    TEST_CHECK("S4-G14", (g_height.b_valid && g_height.gap_count == 1U &&
                          g_height.rise_m == 0.0f),
               "bounded-gap timestamps handle millisecond wrap");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.0f, 1U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(-20.0f, 0.0f, 2U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(-3.0f, 0.0f, 1U);
    TEST_CHECK("S4-G15", (g_height.b_valid && !g_height.b_active &&
                          g_height.rise_m == 0.0f && g_height.count == 0U),
               "stationary tilt gap cannot invent height");
    s4_height_baseline();
    s4_height_feed(-2.94f, 0.00195f, 40U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(-30.0f, 0.00195f, 3U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(-2.94f, 0.00195f, 57U);
    s4_height_feed(2.94f, 0.00195f, 100U);
    s4_height_feed(0.0f, 0.00195f, 30U);
    TEST_CHECK("S4-G16", (g_height.count == 1U &&
                          g_height.gap_count == 1U &&
                          g_height.rejected_count == 0U &&
                          rmath_absf(g_height.last_peak_m - 0.01f) < 0.0002f),
               "ideal 1 cm hump completes with a bounded ascent gap");
    s4_height_baseline();
    s4_height_feed(-2.94f, 0.00195f, 100U);
    s4_height_feed(2.94f, 0.00195f, 40U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(30.0f, 0.00195f, 3U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(2.94f, 0.00195f, 57U);
    s4_height_feed(0.0f, 0.00195f, 30U);
    TEST_CHECK("S4-G17", (g_height.count == 1U &&
                          g_height.gap_count == 1U &&
                          g_height.rejected_count == 0U),
               "ideal 1 cm hump completes with a bounded descent gap");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(0.0f, 0.001f, 2U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(-6.0f, 0.001f, 1U);
    TEST_CHECK("S4-G18", (g_height.b_valid && g_height.b_active &&
                          g_height.gap_count == 1U &&
                          rmath_absf(g_height.rise_m -
                          (0.002f * rmath_sinf(3.0f * RMATH_DEG_TO_RAD) +
                           0.0015f * rmath_sinf(6.0f * RMATH_DEG_TO_RAD))) <
                          0.000001f),
               "changing-slope recovery uses time interpolation rather than held tilt");
    s4_height_baseline();
    s4_calibrate();
    g_processor.b_odometry_height = true;
    for (idx = 0U; idx < 500U; idx++)
    {
        uint32_t step;
        float const angle = (idx < 100U) ? 2.94f
                            : ((idx < 200U) ? -2.94f : 0.0f);
        float const scale = ((idx >= 40U) && (idx < 43U)) ? 1.12f : 1.0f;
        sample = s4_flat();
        sample.acceleration[0] =
            scale * rmath_sinf(angle * RMATH_DEG_TO_RAD);
        sample.acceleration[2] =
            scale * rmath_cosf(angle * RMATH_DEG_TO_RAD);
        for (step = 1U; step <= 2U; step++)
        {
            s4_feed(&sample, 1U);
            g_height_imu = g_processor.output;
            imu_height_observe_imu(&g_height, &g_height_imu,
                                   g_height_ms + step * 10U);
        }
        s4_height_feed(g_processor.output.tilt_pitch_deg, 0.00195f, 1U);
    }
    TEST_CHECK("S4-G19", (g_height.count == 1U &&
                          g_height.gap_count == 2U &&
                          g_height.rejected_count == 0U &&
                          rmath_absf(g_height.last_peak_m - 0.01f) < 0.002f),
               "actual tilt filter and event paths bridge acceleration and transition gaps");
    s4_height_baseline();
    s4_height_feed(-2.94f, 0.00195f, 100U);
    s4_height_feed(2.94f, 0.00195f, 100U);
    s4_height_feed(0.0f, 0.001f, 20U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(0.0f, 0.002f, 3U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(0.0f, 0.002f, 1U);
    TEST_CHECK("S4-G20", (g_height.count == 0U && g_height.b_active &&
                          g_height.level_path_m == 0.0f),
               "interpolated level travel cannot complete hump or count as runout");
    s4_height_feed(0.0f, 0.002f, 16U);
    TEST_CHECK("S4-G21", g_height.count == 1U,
               "valid level runout after a recovered gap completes crossing");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(-3.0f, 0.002f, 1U);
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(-6.0f, 0.001f, 1U);
    TEST_CHECK("S4-G22", (g_height.b_valid &&
                          rmath_absf(g_height.rise_m -
                          (0.0045f * rmath_sinf(3.0f * RMATH_DEG_TO_RAD) +
                           (0.005f / 3.0f) *
                           (rmath_sinf(6.0f * RMATH_DEG_TO_RAD) -
                            rmath_sinf(3.0f * RMATH_DEG_TO_RAD)))) < 0.000001f),
               "buffered variable wheel travel is weighted by each segment time");
    s4_height_baseline();
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(0.0f, 0.0f, 1U);
    imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                      g_height_ms - 101U, g_height_ms + 20U);
    TEST_CHECK("S4-G23", (!g_height.b_gap &&
                          g_height.last_rejection == HEIGHT_REJECT_IMU_AGE),
               "stale IMU data is not accepted as a bridgeable tilt gap");
    s4_height_baseline();
    g_height_imu.b_tilt_valid = false;
    g_height_imu.acceleration_z_g = 1.351f;
    s4_height_feed(0.0f, 0.0f, 1U);
    TEST_CHECK("S4-G24", (!g_height.b_gap &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_ACCELERATION),
               "large acceleration cannot be masked by a short tilt gap");
    s4_height_baseline();
    g_height_imu.b_tilt_valid = false;
    g_height_imu.acceleration_z_g = 0.649f;
    s4_height_feed(0.0f, 0.0f, 1U);
    TEST_CHECK("S4-G25", (!g_height.b_gap &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_ACCELERATION),
               "low acceleration or freefall cannot be bridged");
    s4_height_baseline();
    g_height_imu.b_tilt_valid = false;
    g_height_imu.acceleration_x_g = nonfinite.value;
    s4_height_feed(0.0f, 0.0f, 1U);
    TEST_CHECK("S4-G26", (!g_height.b_gap &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_ACCELERATION),
               "non-finite gap acceleration rejected");
    s4_height_baseline();
    g_height_imu.b_tilt_valid = false;
    for (idx = 0U; idx <= IMU_HEIGHT_GAP_SEGMENTS; idx++)
    {
        g_height_ms++;
        imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                          g_height_ms, g_height_ms);
    }
    TEST_CHECK("S4-G27", (!g_height.b_gap &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_BUFFER),
               "unexpectedly fast motion events cannot overflow gap storage");
    s4_height_baseline();
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(0.0f, 0.011f, 1U);
    s4_height_feed(0.0f, -0.011f, 1U);
    TEST_CHECK("S4-G28", (!g_height.b_gap &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_DISTANCE),
               "gap travel limit counts absolute distance even when reversing");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.002f, 100U);
    {
        float const rise = g_height.rise_m;
        float const peak = g_height.peak_m;
        g_height_imu.tilt_roll_deg = 11.0f;
        s4_height_feed(-3.0f, 0.002f, 1U);
        TEST_CHECK("S4-R01", (g_height.b_interrupted_available &&
                              g_height.interrupted_rise_m == rise &&
                              g_height.interrupted_peak_m == peak &&
                              g_height.interrupted_reason == HEIGHT_REJECT_ROLL &&
                              !g_height.b_active && !g_height.b_valid &&
                              g_height.count == 0U),
                   "rejected active crossing retained separately as invalid diagnostic");
        TEST_CHECK("S4-R02",
                   (g_height_imu.b_height_interrupted_available &&
                    g_height_imu.interrupted_peak_m == peak &&
                    g_height_imu.height_interrupted_reason == HEIGHT_REJECT_ROLL &&
                    !g_height_imu.b_hump_height_valid &&
                    !g_height_imu.b_height_tracking_valid),
                   "event reporting preserves interrupted peak without marking height valid");
        imu_height_invalidate(&g_height, HEIGHT_REJECT_SENSOR);
        TEST_CHECK("S4-R03", (g_height.rejected_count == 1U &&
                              g_height.interrupted_peak_m == peak &&
                              g_height.interrupted_reason == HEIGHT_REJECT_ROLL &&
                              g_height.last_rejection == HEIGHT_REJECT_SENSOR),
                   "later idle failures do not overwrite retained crossing or its cause");
        g_height_imu.tilt_roll_deg = 0.0f;
        s4_height_feed(0.0f, 0.0f, 12U);
        s4_height_feed(-2.94f, 0.00195f, 100U);
        s4_height_feed(2.94f, 0.00195f, 100U);
        s4_height_feed(0.0f, 0.00195f, 30U);
        TEST_CHECK("S4-R04", (g_height.count == 1U && g_height.b_peak_valid &&
                              g_height.interrupted_peak_m == peak &&
                              g_height.interrupted_reason == HEIGHT_REJECT_ROLL),
                   "new completed measurement stays separate from retained rejected crossing");
        s4_height_feed(-3.0f, 0.002f, 20U);
        float const second_peak = g_height.peak_m;
        float const completed_peak = g_height.last_peak_m;
        imu_height_invalidate(&g_height, HEIGHT_REJECT_SHOCK);
        TEST_CHECK("S4-R05",
                   (g_height.interrupted_peak_m == second_peak &&
                    g_height.interrupted_reason == HEIGHT_REJECT_SHOCK &&
                    g_height.last_peak_m == completed_peak &&
                    g_height.b_peak_valid && g_height.count == 1U &&
                    g_height.rejected_count == 2U),
                   "next rejected crossing replaces diagnostic only, never completed result");
    }
    imu_height_init(&g_height);
    imu_height_apply(&g_height, &g_height_imu);
    TEST_CHECK("S4-R06", (!g_height.b_interrupted_available &&
                          !g_height_imu.b_height_interrupted_available &&
                          g_height.interrupted_peak_m == 0.0f),
               "explicit init clears retained interrupted crossing");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 10U);
    {
        float const rise = g_height.rise_m;
        g_height_imu.b_tilt_valid = false;
        g_height_imu.acceleration_z_g = 1.25f;
        s4_height_feed(-30.0f, 0.001f, 8U);
        TEST_CHECK("S4-N01", (g_height.b_active && g_height.b_gap &&
                              !g_height.b_valid && g_height.rise_m == rise &&
                              g_height.rejected_count == 0U),
                   "160 ms 1.25 g disturbance preserves active crossing with height frozen");
        g_height_imu.b_tilt_valid = true;
        g_height_imu.acceleration_z_g = 1.0f;
        s4_height_feed(-3.0f, 0.001f, 1U);
        TEST_CHECK("S4-N02", (g_height.b_valid && g_height.gap_count == 1U &&
                              rmath_absf(g_height.rise_m - rise -
                              0.009f * rmath_sinf(3.0f * RMATH_DEG_TO_RAD)) <
                              0.000001f),
                   "recovery accounts for every buffered wheel segment after moderate acceleration");
    }
    s4_height_baseline();
    g_height_imu.b_tilt_valid = false;
    g_height_imu.acceleration_z_g = 0.65f;
    s4_height_feed(0.0f, 0.001f, 1U);
    TEST_CHECK("S4-N03", (g_height.b_gap && g_height.b_ready),
               "650 mg acceleration boundary accepted only as a pending gap");
    g_height_imu.acceleration_z_g = 1.35f;
    s4_height_feed(0.0f, 0.001f, 1U);
    TEST_CHECK("S4-N04", (g_height.b_gap && g_height.b_ready &&
                          g_height.count == 0U),
               "1350 mg boundary does not complete or silently validate gap");
    s4_height_baseline();
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(0.0f, 0.010f, 1U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(0.0f, 0.010f, 1U);
    TEST_CHECK("S4-N05", (g_height.b_valid && g_height.gap_count == 1U),
               "20 mm bracket accepted at travel boundary");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(3.0f, 0.001f, 1U);
    TEST_CHECK("S4-N06", (g_height.b_valid && g_height.gap_count == 1U &&
                          g_height.count == 0U),
               "6 degree crest transition can recover without claiming completion");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 1U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(-3.0f, 0.0f, 12U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(-3.0f, 0.0f, 1U);
    TEST_CHECK("S4-N07", (!g_height.b_valid && !g_height.b_active &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_TIMEOUT),
               "260 ms gap rejected even if its final tilt is valid");
    s4_height_baseline();
    s4_height_feed(2.0f, 0.001f, 1U);
    {
        float const fall = g_height.rise_m;
        TEST_CHECK("S4-N08", (g_height.b_start_pending && g_height.b_ready &&
                              !g_height.b_active && fall < 0.0f),
                   "tiny initial downhill increment retains original ground datum");
        s4_height_feed(-2.94f, 0.001f, 2U);
        TEST_CHECK("S4-N09", (g_height.b_active && !g_height.b_start_pending &&
                              rmath_absf(g_height.rise_m -
                              (fall + 0.0005f *
                               (rmath_sinf(2.94f * RMATH_DEG_TO_RAD) -
                                rmath_sinf(2.0f * RMATH_DEG_TO_RAD)) +
                               0.001f * rmath_sinf(2.94f * RMATH_DEG_TO_RAD))) <
                              0.000001f),
                   "start recovery includes initial negative travel instead of zeroing the dip");
    }
    s4_height_baseline();
    s4_height_feed(2.0f, 0.001f, 1U);
    s4_height_feed(2.0f, 0.0f, 13U);
    TEST_CHECK("S4-N10", (!g_height.b_ready &&
                          g_height.last_rejection == HEIGHT_REJECT_DOWNHILL_START),
               "unconfirmed start expires after 250 ms even when wheels stop");
    s4_height_baseline();
    s4_height_feed(10.0f, 0.005f, 2U);
    s4_height_feed(-10.0f, 0.005f, 2U);
    TEST_CHECK("S4-N11", (!g_height.b_ready && !g_height.b_active &&
                          g_height.count == 0U),
               "over 0.5 mm initial fall rejects before dip recovery can start a hump");
    s4_height_baseline();
    s4_height_feed(1.01f, 0.004f, 6U);
    TEST_CHECK("S4-N12", (!g_height.b_ready &&
                          g_height.last_rejection == HEIGHT_REJECT_DOWNHILL_START),
               "initial downhill confirmation is bounded by 20 mm absolute travel");
    s4_height_baseline();
    g_height_ms = 0xFFFFFFF0U;
    g_height.last_ms = g_height_ms;
    s4_height_feed(2.0f, 0.001f, 1U);
    s4_height_feed(-2.94f, 0.001f, 2U);
    TEST_CHECK("S4-N13", (g_height.b_active && g_height.b_valid),
               "bounded start confirmation handles timer wrap");
    s4_height_baseline();
    s4_height_feed(2.0f, 0.001f, 1U);
    s4_height_feed(-2.94f, 0.001f, 2U);
    s4_height_feed(-2.94f, 0.00195f, 100U);
    g_height_imu.b_tilt_valid = false;
    g_height_imu.acceleration_z_g = 1.25f;
    s4_height_feed(-30.0f, 0.001f, 8U);
    g_height_imu.b_tilt_valid = true;
    g_height_imu.acceleration_z_g = 1.0f;
    s4_height_feed(2.94f, 0.001f, 1U);
    s4_height_feed(2.94f, 0.00195f, 100U);
    s4_height_feed(0.0f, 0.00195f, 30U);
    TEST_CHECK("S4-N14", (g_height.count == 1U &&
                          g_height.rejected_count == 0U &&
                          g_height.gap_count == 1U &&
                          rmath_absf(g_height.last_peak_m - 0.010f) < 0.001f),
               "ideal 1 cm crossing completes with initial transient and 180 ms accelerated crest gap");
    s4_height_baseline();
    s4_calibrate();
    g_processor.b_odometry_height = true;
    for (idx = 0U; idx < 500U; idx++)
    {
        uint32_t step;
        float const angle = (idx < 100U) ? 2.94f
                            : ((idx < 200U) ? -2.94f : 0.0f);
        float const scale = (idx >= 100U && idx < 106U) ? 1.25f : 1.0f;
        sample = s4_flat();
        sample.acceleration[0] = scale * rmath_sinf(angle * RMATH_DEG_TO_RAD);
        sample.acceleration[2] = scale * rmath_cosf(angle * RMATH_DEG_TO_RAD);
        for (step = 1U; step <= 2U; step++)
        {
            s4_feed(&sample, 1U);
            g_height_imu = g_processor.output;
            imu_height_observe_imu(&g_height, &g_height_imu,
                                   g_height_ms + step * 10U);
        }
        s4_height_feed(g_processor.output.tilt_pitch_deg, 0.00195f, 1U);
    }
    TEST_CHECK("S4-N15", (g_height.count == 1U &&
                          g_height.rejected_count == 0U &&
                          g_height.gap_count > 0U &&
                          rmath_absf(g_height.last_peak_m - 0.010f) < 0.002f),
               "actual tilt filter completes ideal 1 cm hump through 120 ms 1.25 g crest disturbance");
    s4_height_baseline();
    s4_height_feed(-2.94f, 0.0001f, 2000U);
    TEST_CHECK("S4-N16", (g_height.b_active &&
                          g_height_imu.motion_state == MOTION_CLIMBING &&
                          rmath_absf(g_height.peak_m - 0.01026f) < 0.0002f),
               "slow encoder increments below 10 micrometres vertical rise are accumulated");
    s4_height_feed(2.94f, 0.0001f, 2000U);
    TEST_CHECK("S4-N17", (g_height_imu.motion_state == MOTION_DESCENDING),
               "slow downhill wheel increments remain classified as descending");
    s4_height_feed(0.0f, 0.0001f, 400U);
    TEST_CHECK("S4-N18", (g_height.count == 1U &&
                          g_height.rejected_count == 0U),
               "slow ideal shallow crossing completes without a per-sample rise deadband");
    s4_height_baseline();
    s4_height_feed(2.94f, 0.0001f, 2000U);
    s4_height_feed(-2.94f, 0.0001f, 2000U);
    TEST_CHECK("S4-N19", (!g_height.b_ready && !g_height.b_active &&
                          g_height.count == 0U &&
                          g_height.last_rejection == HEIGHT_REJECT_DOWNHILL_START),
               "slow downhill dip cannot lose negative increments and become a false hump");
    s4_height_baseline();
    s4_height_feed(1.3f, 0.002f, 3U);
    s4_height_feed(0.0f, 0.002f, 30U);
    {
        float const debt = g_height.rise_m;
        TEST_CHECK("S4-N20", (g_height.b_ready && g_height.b_start_pending &&
                              !g_height.b_active && debt < -0.0001f &&
                              g_height.path_m > 0.020f),
                   "tiny dip then 600 ms flat travel preserves datum beyond old start deadline");
        s4_height_feed(0.0f, 0.0f, 100U);
        TEST_CHECK("S4-N21", (g_height.b_valid &&
                              g_height.rise_m == debt &&
                              g_height.b_start_pending),
                   "level pause preserves negative height debt without timeout");
        s4_height_feed(-2.94f, 0.001f, 1U);
        TEST_CHECK("S4-N22", (!g_height.b_active &&
                              g_height.rise_m < 0.0f),
                   "first upward increment cannot erase the retained negative debt");
        s4_height_feed(-2.94f, 0.00195f, 100U);
        s4_height_feed(2.94f, 0.00195f, 100U);
        s4_height_feed(0.0f, 0.00195f, 30U);
        TEST_CHECK("S4-N23", (g_height.count == 1U &&
                              g_height.rejected_count == 0U &&
                              rmath_absf(g_height.last_peak_m -
                                  (0.010f + debt)) < 0.001f),
                   "1 cm crossing completes after level pause with initial debt included");
    }
    s4_height_baseline();
    s4_height_feed(1.3f, 0.002f, 3U);
    s4_height_feed(0.0f, 0.001f, 30U);
    s4_height_feed(-1.3f, 0.002f, 3U);
    s4_height_feed(0.0f, 0.001f, 30U);
    TEST_CHECK("S4-N24", (g_height.count == 0U &&
                          g_height.peak_m < 0.0001f),
               "dip-flat-recovery cannot invent a hump by renewing start window");
    s4_height_baseline();
    s4_height_feed(1.3f, 0.002f, 3U);
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(0.0f, 0.0f, 6U);
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(0.0f, 0.0f, 1U);
    s4_height_feed(1.3f, 0.0f, 6U);
    TEST_CHECK("S4-N25", (!g_height.b_ready &&
                          g_height.last_rejection == HEIGHT_REJECT_DOWNHILL_START),
               "interpolated level cannot renew the start confirmation deadline");
    s4_height_baseline();
    s4_height_feed(1.3f, 0.002f, 3U);
    s4_height_feed(0.0f, 0.001f, 30U);
    s4_height_feed(5.0f, 0.004f, 2U);
    TEST_CHECK("S4-N26", (!g_height.b_ready &&
                          g_height.last_rejection == HEIGHT_REJECT_DOWNHILL_START),
               "renewed start window still rejects cumulative fall exceeding 0.5 mm");
    s4_height_baseline();
    s4_height_feed(-4.6f, 0.001f, 10U);
    {
        float const peak = g_height.peak_m;
        g_height_ms += 40U;
        g_height_motion.distance_m += 0.028962f;
        g_height_motion.left_speed_m_s = 0.031f;
        g_height_motion.right_speed_m_s = 0.033f;
        imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                          g_height_ms, g_height_ms);
        TEST_CHECK("S4-T01", (!g_height.b_valid &&
                              g_height.last_rejection ==
                                  HEIGHT_REJECT_DISTANCE_SPEED &&
                              g_height.interrupted_peak_m == peak),
                   "recorded 29 mm travel at 32 mm/s in 40 ms cannot inflate peak");
    }
    s4_height_baseline();
    s4_height_feed(-4.6f, 0.001f, 10U);
    g_height_ms += 900U;
    g_height_motion.distance_m += 0.0288f;
    g_height_motion.left_speed_m_s = 0.032f;
    g_height_motion.right_speed_m_s = 0.032f;
    imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                      g_height_ms, g_height_ms);
    TEST_CHECK("S4-T02", (!g_height.b_valid &&
                          g_height.last_rejection == HEIGHT_REJECT_INTERVAL),
               "real 900 ms unsampled interval is rejected instead of compressed to 40 ms");
    s4_height_baseline();
    s4_height_feed(-4.6f, 0.001f, 10U);
    g_height_ms += 40U;
    g_height_motion.distance_m += 0.00128f;
    g_height_motion.left_speed_m_s = 0.031f;
    g_height_motion.right_speed_m_s = 0.033f;
    imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                      g_height_ms, g_height_ms);
    TEST_CHECK("S4-T03", (g_height.b_valid && g_height.b_active),
               "consistent 1.28 mm travel at 32 mm/s in 40 ms remains supported");
    s4_height_baseline();
    g_height_ms += 20U;
    g_height_motion.left_speed_m_s = 0.3f;
    g_height_motion.right_speed_m_s = 0.3f;
    imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                      g_height_ms, g_height_ms);
    TEST_CHECK("S4-T04", (!g_height.b_valid &&
                          g_height.last_rejection ==
                              HEIGHT_REJECT_DISTANCE_SPEED),
               "speed without matching cumulative displacement is also rejected");
    s4_calibrate();
    g_processor.b_odometry_height = true;
    sample = s4_flat();
    {
        float const gravity = g_processor.gravity[2];
        uint32_t const calibration = g_processor.calibration_count;
        g_ms += 44U;
        imu_processor_update(&g_processor, &sample, true, g_ms);
        TEST_CHECK("S4-T05", (g_processor.output.b_sensor_valid &&
                              g_processor.output.b_timing_gap &&
                              !g_processor.output.b_tilt_valid &&
                              g_processor.output.sample_interval_ms == 44U &&
                              g_processor.output.b_calibrated &&
                              g_processor.calibration_count == calibration &&
                              g_processor.gravity[2] == gravity &&
                              imu_height_imu_rejection(&g_processor.output) ==
                                  HEIGHT_REJECT_TILT),
                   "recorded 44 ms successful read preserves calibration and is a recoverable timing gap");
        s4_feed(&sample, 1U);
        TEST_CHECK("S4-T06", (g_processor.output.b_sensor_valid &&
                              !g_processor.output.b_timing_gap &&
                              g_processor.output.b_tilt_valid &&
                              g_processor.output.sample_interval_ms == 10U),
                   "next normal IMU sample recovers tilt without resetting calibration");
    }
    s4_height_baseline();
    s4_calibrate();
    g_processor.b_odometry_height = true;
    sample = s4_flat();
    sample.acceleration[0] = rmath_sinf(3.0f * RMATH_DEG_TO_RAD);
    sample.acceleration[2] = rmath_cosf(3.0f * RMATH_DEG_TO_RAD);
    s4_feed(&sample, 100U);
    g_height_imu = g_processor.output;
    s4_height_feed(g_height_imu.tilt_pitch_deg, 0.001f, 10U);
    {
        float const rise = g_height.rise_m;
        g_ms += 44U;
        imu_processor_update(&g_processor, &sample, true, g_ms);
        g_height_imu = g_processor.output;
        imu_height_observe_imu(&g_height, &g_height_imu, g_height_ms + 10U);
        s4_height_feed(g_height_imu.tilt_pitch_deg, 0.001f, 2U);
        TEST_CHECK("S4-T07", (g_height.b_gap && g_height.b_active &&
                              g_height.rise_m == rise &&
                              g_height.rejected_count == 0U),
                   "actual processor timing gap freezes active crossing instead of sensor rejection");
        s4_feed(&sample, 1U);
        g_height_imu = g_processor.output;
        s4_height_feed(g_height_imu.tilt_pitch_deg, 0.001f, 1U);
        TEST_CHECK("S4-T08", (g_height.b_valid && !g_height.b_gap &&
                              g_height.b_active && g_height.gap_count == 1U &&
                              g_height.rejected_count == 0U &&
                              rmath_absf(g_height.rise_m - rise -
                                  0.003f * rmath_sinf(3.0f * RMATH_DEG_TO_RAD)) <
                                  0.00001f),
                   "timing recovery replays all wheel distance with valid endpoint tilt");
    }
    g_ms += 157U;
    imu_processor_update(&g_processor, &sample, true, g_ms);
    TEST_CHECK("S4-T09", (g_processor.output.b_sensor_valid &&
                          g_processor.output.b_timing_gap &&
                          g_processor.output.b_calibrated &&
                          imu_height_imu_rejection(&g_processor.output) ==
                              HEIGHT_REJECT_SAMPLE_TIMING),
               "157 ms stall has explicit timing reason not fabricated sensor failure");
    imu_height_observe_imu(&g_height, &g_processor.output, g_height_ms + 10U);
    TEST_CHECK("S4-T10", (!g_height.b_active && !g_height.b_valid &&
                          g_height.interrupted_reason ==
                              HEIGHT_REJECT_SAMPLE_TIMING),
               "unsupported long timing gap retains interrupted peak but cannot certify height");
    s4_calibrate();
    g_ms += 1U;
    imu_processor_update(&g_processor, &sample, true, g_ms);
    TEST_CHECK("S4-T11", (g_processor.output.b_sensor_valid &&
                          g_processor.output.b_timing_gap &&
                          imu_height_imu_rejection(&g_processor.output) ==
                              HEIGHT_REJECT_SAMPLE_TIMING),
               "back-to-back sample does not masquerade as normal integration");
    s4_calibrate();
    g_ms += 44U;
    imu_processor_update(&g_processor, &sample, false, g_ms);
    TEST_CHECK("S4-T12", (!g_processor.output.b_sensor_valid &&
                          imu_height_imu_rejection(&g_processor.output) ==
                              HEIGHT_REJECT_SENSOR),
               "real failed read takes precedence over simultaneous timing gap");
    s4_calibrate();
    sample.acceleration[0] = 4.094f;
    g_ms += 44U;
    imu_processor_update(&g_processor, &sample, true, g_ms);
    TEST_CHECK("S4-T13", (g_processor.output.b_saturated &&
                          imu_height_imu_rejection(&g_processor.output) ==
                              HEIGHT_REJECT_SATURATION),
               "clipping cannot be hidden behind recoverable timing");
    s4_calibrate();
    sample = s4_flat();
    sample.acceleration[0] = 3.2f;
    g_ms += 44U;
    imu_processor_update(&g_processor, &sample, true, g_ms);
    TEST_CHECK("S4-T14", (g_processor.output.b_shock_detected &&
                          imu_height_imu_rejection(&g_processor.output) ==
                              HEIGHT_REJECT_SHOCK),
               "impact remains immediate during delayed sample");
    s4_calibrate();
    sample = s4_flat();
    g_ms += IMU_RECOVERABLE_INTERVAL_MS;
    imu_processor_update(&g_processor, &sample, true, g_ms);
    TEST_CHECK("S4-T15",
               imu_height_imu_rejection(&g_processor.output) == HEIGHT_REJECT_TILT,
               "100 ms delayed sensor endpoint is within timing recovery limit");
    g_ms += IMU_RECOVERABLE_INTERVAL_MS + 1U;
    imu_processor_update(&g_processor, &sample, true, g_ms);
    TEST_CHECK("S4-T16",
               imu_height_imu_rejection(&g_processor.output) ==
                   HEIGHT_REJECT_SAMPLE_TIMING,
               "101 ms delayed sensor endpoint exceeds timing recovery limit");
    s4_height_baseline();
    s4_height_feed(-2.94f, 0.00195f, 40U);
    g_height_imu.b_timing_gap = true;
    g_height_imu.sample_interval_ms = 44U;
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(-2.94f, 0.00195f, 2U);
    g_height_imu.b_timing_gap = false;
    g_height_imu.b_tilt_valid = true;
    s4_height_feed(-2.94f, 0.00195f, 58U);
    s4_height_feed(2.94f, 0.00195f, 100U);
    s4_height_feed(0.0f, 0.00195f, 30U);
    TEST_CHECK("S4-T17", (g_height.count == 1U &&
                          g_height.rejected_count == 0U &&
                          g_height.gap_count == 1U &&
                          rmath_absf(g_height.last_peak_m - 0.01f) < 0.0002f),
               "ideal 1 cm ascent/descent completes after a recoverable timing gap");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 10U);
    g_height_imu.b_timing_gap = true;
    g_height_imu.sample_interval_ms = 44U;
    g_height_imu.b_tilt_valid = false;
    s4_height_feed(-3.0f, 0.0f, 13U);
    TEST_CHECK("S4-T18", (!g_height.b_active && g_height.count == 0U &&
                          g_height.last_rejection == HEIGHT_REJECT_GAP_TIMEOUT),
               "repeated late samples without normal endpoint cannot indefinitely preserve valid tracking");
    s4_calibrate();
    sample = s4_flat();
    sample.acceleration[0] = nonfinite.value;
    g_ms += 44U;
    imu_processor_update(&g_processor, &sample, true, g_ms);
    TEST_CHECK("S4-T19", (!g_processor.output.b_sensor_valid &&
                          imu_height_imu_rejection(&g_processor.output) ==
                              HEIGHT_REJECT_SENSOR),
               "nonfinite acceleration remains a hard failure during a timing gap");
    s4_height_baseline();
    s4_height_feed(-3.0f, 0.001f, 10U);
    {
        float const rise = g_height.rise_m;
        uint32_t const old_imu_ms = g_height_ms;
        g_height_ms += 80U;
        g_height_motion.distance_m += 0.004f;
        g_height_motion.left_speed_m_s = 0.05f;
        g_height_motion.right_speed_m_s = 0.05f;
        imu_height_update(&g_height, &g_height_motion, &g_height_imu,
                          old_imu_ms, g_height_ms);
        TEST_CHECK("S4-T20", (g_height.b_gap && g_height.b_active &&
                              !g_height.b_valid && g_height.rise_m == rise),
                   "80 ms old IMU buffers travel without using stale tilt as a recovery endpoint");
        s4_height_feed(-3.0f, 0.001f, 1U);
        TEST_CHECK("S4-T21", (g_height.b_valid && g_height.gap_count == 1U &&
                              rmath_absf(g_height.rise_m - rise -
                                  0.005f * rmath_sinf(3.0f * RMATH_DEG_TO_RAD)) <
                                  0.000001f),
                   "fresh endpoint recovers travel buffered while waiting for delayed sensor");
    }
    TEST_SUMMARY("subsystem4 LSM303DLHC");
}

#if defined(IMU_HOST_TEST)
int
main (void)
{
    subsystem4_test();
    return ((0 == g_host_failures) ? 0 : 1);
}
#endif
#endif /* APP_TEST || IMU_HOST_TEST */
