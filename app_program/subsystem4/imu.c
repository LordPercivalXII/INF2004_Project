/** @file imu.c
 * @brief 100 Hz IMU acquisition and event reporting to the controller.
 */
#include "subsystem4.h"
#include "drv_lsm303dlhc.h"
#include "../common/drv_board.h"
#include "../common/rmath.h"
#include "../controller/controller.h"

#define IMU_RETRY_MS             (1000U)
#define IMU_CALIBRATION_WARN_MS  (5000U)
#define IMU_SIM_CYCLE_SAMPLES    (800U)

static lsm303_read_result_t
imu_sample (bool b_initialized, imu_sample_t * p_sample, uint32_t sample)
{
#if ROBOT_SIMULATION
    uint32_t const phase = sample % IMU_SIM_CYCLE_SAMPLES;
    (void) b_initialized;
    *p_sample = (imu_sample_t) {
        .acceleration = {0.0f, 0.0f, 1.0f},
        .magnetic_ut = {30.0f, 0.0f, 20.0f},
        .b_magnetic_valid = true,
        .b_magnetic_fresh = true
    };

    /* Two seconds at rest for calibration; a 9.8 cm synthetic hump,
       then acceleration, a turn and one impact per eight-second cycle. */
    if ((phase >= 300U) && (phase < 320U))
    {
        p_sample->acceleration[2] += 0.25f;
    }
    else if ((phase >= 320U) && (phase < 360U))
    {
        p_sample->acceleration[2] -= 0.25f;
    }
    else if ((phase >= 360U) && (phase < 380U))
    {
        p_sample->acceleration[2] += 0.25f;
    }
    else if ((phase >= 450U) && (phase < 490U))
    {
        p_sample->acceleration[0] = 0.25f;
    }
    else if ((phase >= 550U) && (phase < 600U))
    {
        float const angle = (float) (phase - 550U) * 0.3f * RMATH_DEG_TO_RAD;
        p_sample->magnetic_ut[0] = 30.0f * rmath_cosf(angle);
        p_sample->magnetic_ut[1] = -30.0f * rmath_sinf(angle);
    }
    else if (phase >= 600U)
    {
        p_sample->magnetic_ut[0] = 30.0f * rmath_cosf(15.0f * RMATH_DEG_TO_RAD);
        p_sample->magnetic_ut[1] = -30.0f * rmath_sinf(15.0f * RMATH_DEG_TO_RAD);
    }
    if (650U == phase)
    {
        p_sample->acceleration[0] = 3.2f;
    }
    return (LSM303_READ_READY);
#else
    (void) sample;
    *p_sample = (imu_sample_t) {0};
    return (b_initialized ? lsm303_read(p_sample) : LSM303_READ_ERROR);
#endif
}

void
imu_task (INT stacd, void * p_exinf)
{
    /* The bounded hump history must not consume the 2 KB task stack. */
    static imu_processor_t processor;
    imu_sample_t sample;
    system_event_t event = {0};
    uint32_t sequence = 0U;
    uint32_t last_notice_ms = board_millis();
    uint32_t last_status_ms = last_notice_ms;
    uint32_t now_ms;
    uint32_t previous_sample_ms = 0U;
    uint32_t last_ready_ms = board_millis();
    lsm303_read_result_t read_result;
    bool b_initialized;
    bool b_valid;
    bool b_error_reported = false;
    bool b_queue_error = false;
    bool b_was_calibrated = false;

    (void) stacd;
    (void) p_exinf;
    imu_processor_init(&processor);
    processor.b_odometry_height = true;
#if ROBOT_SIMULATION
    b_initialized = true;
#else
    b_initialized = lsm303_init();
#endif
    tm_printf((UB *) "S4: keep car STILL and LEVEL for 2 seconds; "
              "LSM303DLHC X forward, Y left, Z up\n");
    tm_printf((UB *) "S4: magnetic heading is UNCALIBRATED; motors/magnets "
              "affect turn estimates; hump height is APPROXIMATE\n");
    tm_printf((UB *) "S4: height uses IMU pitch + wheel travel; start LEVEL, "
              "roll straight, no lifting/slip; live rise is not altitude\n");
    (void) tk_can_wup(TSK_SELF);

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);
        now_ms = board_millis();
#if !ROBOT_SIMULATION
        if ((!b_initialized) && ((now_ms - last_notice_ms) >= IMU_RETRY_MS))
        {
            last_notice_ms = now_ms;
            tm_printf((UB *) "S4: retrying LSM303DLHC on iica GP4/GP5\n");
            b_initialized = lsm303_init();
            if (b_initialized)
            {
                imu_processor_init(&processor);
                processor.b_odometry_height = true;
                b_was_calibrated = false;
                last_ready_ms = board_millis();
                tm_printf((UB *) "S4: sensor reconnected; recalibrating\n");
            }
            (void) tk_can_wup(TSK_SELF);
        }
#endif
        read_result = imu_sample(b_initialized, &sample, sequence);
        sequence++;
        now_ms = board_millis();
        if ((LSM303_READ_WAIT == read_result) &&
            ((now_ms - last_ready_ms) < 100U))
        {
            (void) tk_can_wup(TSK_SELF);
            continue;
        }
        b_valid = (LSM303_READ_READY == read_result);
        if (b_valid)
        {
            last_ready_ms = now_ms;
        }
        imu_processor_update(&processor, &sample, b_valid, now_ms);
        if (!b_valid)
        {
            b_initialized = false;
        }
        if ((!processor.output.b_sensor_valid) && (!b_error_reported))
        {
            tm_printf((UB *) "S4: invalid IMU sample: read_ok=%u "
                      "interval_ms=%u; terrain window discarded\n",
                      (unsigned int) b_valid,
                      (unsigned int) (now_ms - previous_sample_ms));
            b_error_reported = true;
        }
        else if (processor.output.b_sensor_valid)
        {
            b_error_reported = false;
        }
        if ((now_ms - last_status_ms) >= IMU_CALIBRATION_WARN_MS)
        {
            tm_printf((UB *) "S4: LSM303 init=%u read=%u valid=%u "
                      "cal=%u/200 dt_ms=%u accel_mg=%d,%d,%d\n",
                      (unsigned int) b_initialized, (unsigned int) b_valid,
                      (unsigned int) processor.output.b_sensor_valid,
                      (unsigned int) processor.calibration_count,
                      (unsigned int) (now_ms - previous_sample_ms),
                      (int) (sample.acceleration[0] * 1000.0f),
                      (int) (sample.acceleration[1] * 1000.0f),
                      (int) (sample.acceleration[2] * 1000.0f));
            last_status_ms = now_ms;
        }
        previous_sample_ms = now_ms;
        if ((!b_was_calibrated) && processor.output.b_calibrated)
        {
            tm_printf((UB *) "S4: calibration complete (200 stable samples)\n");
            b_was_calibrated = true;
        }
        if ((!processor.output.b_calibrated) &&
            ((now_ms - last_notice_ms) >= IMU_CALIBRATION_WARN_MS))
        {
            tm_printf((UB *) "S4: calibration waiting: hold car still, "
                      "level, sensor Z up\n");
            last_notice_ms = now_ms;
        }
        if (processor.output.b_hump_rejected)
        {
            tm_printf((UB *) "S4: terrain window rejected; no new height\n");
        }
        event.type = SYSTEM_EVENT_IMU;
        event.timestamp_ms = now_ms;
        event.data.imu = processor.output;
        if (!controller_post_event(&event))
        {
            if (!b_queue_error)
            {
                tm_printf((UB *) "S4: system event buffer full; "
                          "IMU event not delivered\n");
                b_queue_error = true;
            }
        }
        else
        {
            b_queue_error = false;
        }
        /* Never integrate several back-to-back reads as 10 ms samples. */
        (void) tk_can_wup(TSK_SELF);
    }
}

/*** end of file ***/
