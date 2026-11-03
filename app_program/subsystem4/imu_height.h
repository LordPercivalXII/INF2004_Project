/** @file imu_height.h
 * @brief Relative terrain height from signed wheel travel and IMU pitch.
 *
 * X forward, Y left, Z up; pitch is positive nose-down. Encoders measure
 * distance along the surface, so dz = -ds * sin(pitch). Start on level
 * ground. No absolute altitude or wheel-slip detection is provided.
 * Low-pass accelerometer tilt lags transitions and is biased by acceleration;
 * use slow, straight travel, calibrated wheels, and modest cross-slope.
 * A plateau retains live rise; only a descent back to the starting level
 * followed by level travel completes a hump. Manual lifting cannot be measured.
 * Supported gates: pitch within 35 deg, roll within 10 deg; baseline pitch
 * within 1 deg for ten motion updates (so a 1 cm / 19.5 cm slope is not
 * mistaken for level ground). A completed hump needs 3 cm of level
 * runout after descending. Tilt-only gaps may be interpolated after recovery:
 * at most 250 ms between motion anchors, 20 mm travel, and 6 deg change in
 * endpoint pitch. At most 60 mm of interpolated travel is allowed per datum.
 * Gap samples must have acceleration magnitude within 0.65..1.35 g; their
 * invalid pitch is never integrated. Ordinary motion intervals remain at
 * most 100 ms, and sensor/impact/geometry checks are not relaxed.
 * An initial negative excursion is confirmed over at most 250 ms / 20 mm.
 * A fresh measured level sample renews that window without zeroing signed
 * height or total path; interpolated level cannot renew it. Net fall remains
 * limited to 0.5 mm, total path to 10 m. All travel retains the original datum;
 * a sustained descent still rejects instead of becoming a hump on recovery.
 * These bounds limit approximation, not physical accuracy: acceleration-only
 * tilt cannot bound unobserved rotation. Other failures discard the estimate
 * and require a new level baseline (which may be at a different elevation).
 * Pending gaps freeze live height, mark it invalid, and cannot complete a hump
 * or contribute level runout. Baseline acquisition requires valid samples.
 * imu_event_t: terrain_rise_m/terrain_peak_m are live; hump height/count retain
 * the last completed result even when current height tracking is invalid.
 * Endpoint error must be within max(3 mm, 35% of peak); no promise of
 * millimetre hardware accuracy is implied by this consistency gate.
 * last_rejection records the failed gate, including baseline loss before
 * an active crossing. It is retained until another rejection or init.
 * interrupted_* retains the most recently rejected active crossing until
 * init or a later active rejection, independently of live/completed height.
 * It is diagnostic data only, never a valid height measurement.
 */
#ifndef IMU_HEIGHT_H
#define IMU_HEIGHT_H

#include "../common/robot_types.h"

typedef enum
{
    HEIGHT_REJECT_NONE = 0,
    HEIGHT_REJECT_SENSOR,
    HEIGHT_REJECT_CALIBRATION,
    HEIGHT_REJECT_SATURATION,
    HEIGHT_REJECT_SHOCK,
    HEIGHT_REJECT_TILT,
    HEIGHT_REJECT_IMU_AGE,
    HEIGHT_REJECT_PITCH,
    HEIGHT_REJECT_ROLL,
    HEIGHT_REJECT_DISTANCE,
    HEIGHT_REJECT_SPEED,
    HEIGHT_REJECT_TURN,
    HEIGHT_REJECT_INTERVAL,
    HEIGHT_REJECT_DISTANCE_JUMP,
    HEIGHT_REJECT_DOWNHILL_START,
    HEIGHT_REJECT_PATH,
    HEIGHT_REJECT_RISE,
    HEIGHT_REJECT_PEAK,
    HEIGHT_REJECT_ENDPOINT,
    HEIGHT_REJECT_WHEEL_AGE,
    HEIGHT_REJECT_GAP_TIMEOUT,
    HEIGHT_REJECT_GAP_DISTANCE,
    HEIGHT_REJECT_GAP_PITCH,
    HEIGHT_REJECT_GAP_BUDGET,
    HEIGHT_REJECT_GAP_BUFFER,
    HEIGHT_REJECT_GAP_ACCELERATION,
    HEIGHT_REJECT_DISTANCE_SPEED,
    HEIGHT_REJECT_SAMPLE_TIMING
} imu_height_rejection_t;

#define IMU_HEIGHT_GAP_SEGMENTS (16U)

typedef struct
{
    bool b_have_distance;
    bool b_ready;
    bool b_active;
    bool b_start_pending;
    bool b_descended;
    bool b_valid;
    bool b_peak_valid;
    bool b_rejected;
    bool b_complete;
    uint32_t last_ms;
    uint32_t start_ms;
    uint32_t flat_samples;
    uint32_t count;
    uint32_t rejected_count;
    float last_distance_m;
    float last_slope;
    float last_dz;
    float rise_m;
    float peak_m;
    float last_peak_m;
    float path_m;
    float start_path_m;
    float level_path_m;
    imu_height_rejection_t last_rejection;
    bool b_gap;
    uint32_t gap_anchor_ms;
    uint32_t gap_segments;
    uint32_t gap_count;
    float gap_slope;
    float gap_pitch;
    float gap_path_m;
    float bridged_path_m;
    float gap_ds[IMU_HEIGHT_GAP_SEGMENTS];
    uint32_t gap_ms[IMU_HEIGHT_GAP_SEGMENTS];
    bool b_interrupted_available;
    float interrupted_rise_m;
    float interrupted_peak_m;
    imu_height_rejection_t interrupted_reason;
} imu_height_t;

void imu_height_init(imu_height_t * p_height);
imu_height_rejection_t imu_height_imu_rejection(imu_event_t const * p_imu);
char const * imu_height_rejection_name(imu_height_rejection_t reason);
void imu_height_invalidate(imu_height_t * p_height,
                           imu_height_rejection_t reason);
void imu_height_observe_imu(imu_height_t * p_height,
                            imu_event_t const * p_imu, uint32_t imu_ms);
void imu_height_update(imu_height_t * p_height, motion_event_t const * p_motion,
                        imu_event_t const * p_imu, uint32_t imu_ms,
                        uint32_t motion_ms);
void imu_height_apply(imu_height_t const * p_height, imu_event_t * p_imu);

#endif
