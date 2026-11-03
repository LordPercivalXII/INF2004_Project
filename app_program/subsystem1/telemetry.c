/** @file telemetry.c
 *
 * @brief Telemetry task: prints controller snapshots and a heartbeat as
 *        JSON lines on the console (USB CDC, or UART0).
 *
 * tm_printf() has no floating-point conversions, so floats are formatted
 * as fixed-point text by telemetry_format_fixed().  Each JSON line is
 * built in one buffer and written with a single tm_putstring() so lines
 * from tasks on the other core cannot interleave with it.
 */

#include "subsystem1.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../controller/controller.h"

#define TELEMETRY_LINE_MAX      (896U)
#define TELEMETRY_FIXED_LIMIT   (1000000.0f)

/*!
 * @brief Queue a telemetry snapshot without waiting.
 *
 * @return true if queued.
 */
bool
telemetry_publish (telemetry_message_t const * p_message)
{
    bool b_ok = false;

    if ((NULL != p_message) && (gh_telemetry_mbf > 0))
    {
        b_ok = (E_OK == tk_snd_mbf(gh_telemetry_mbf, (void *) p_message,
                                   (INT) sizeof(*p_message), TMO_POL));
    }

    return (b_ok);
}

/*!
 * @brief Forward a user command to the controller without waiting.
 *
 * @return true if queued.
 */
bool
telemetry_submit_command (control_command_t const * p_command)
{
    bool b_ok = false;

    if ((NULL != p_command) && (gh_control_mbf > 0))
    {
        b_ok = (E_OK == tk_snd_mbf(gh_control_mbf, (void *) p_command,
                                   (INT) sizeof(*p_command), TMO_POL));
    }

    return (b_ok);
}

/*!
 * @brief Format a float as fixed-point text, e.g. -0.350.
 *
 * @param[out] p_text    At least S1_FIXED_TEXT_MAX bytes.
 * @param[in]  value     Value; clamped to +-1e6.
 * @param[in]  decimals  1 or 3 (anything else is treated as 3).
 */
void
telemetry_format_fixed (char * p_text, float value, uint32_t decimals)
{
    uint32_t const scale  = (1U == decimals) ? 10U : 1000U;
    bool const     b_neg  = (value < 0.0f);
    float          mag    = b_neg ? -value : value;
    uint32_t       scaled = 0U;

    if (mag > TELEMETRY_FIXED_LIMIT)
    {
        mag = TELEMETRY_FIXED_LIMIT;
    }

    scaled = (uint32_t) ((mag * (float) scale) + 0.5f);

    if (1U == decimals)
    {
        (void) tm_sprintf((UB *) p_text, (UB *) "%s%u.%01u",
                          (b_neg && (0U != scaled)) ? "-" : "",
                          (unsigned int) (scaled / scale),
                          (unsigned int) (scaled % scale));
    }
    else
    {
        (void) tm_sprintf((UB *) p_text, (UB *) "%s%u.%03u",
                          (b_neg && (0U != scaled)) ? "-" : "",
                          (unsigned int) (scaled / scale),
                          (unsigned int) (scaled % scale));
    }
}

/*!
 * @brief Print one telemetry snapshot as a JSON line.
 */
static void
telemetry_print (telemetry_message_t const * p_msg)
{
    char line[TELEMETRY_LINE_MAX];
    char speed[S1_FIXED_TEXT_MAX];
    char error[S1_FIXED_TEXT_MAX];
    char dist[S1_FIXED_TEXT_MAX];
    char hump[S1_FIXED_TEXT_MAX];
    char obst[S1_FIXED_TEXT_MAX];
    char roll[S1_FIXED_TEXT_MAX];
    char pitch[S1_FIXED_TEXT_MAX];
    char turn[S1_FIXED_TEXT_MAX];
    char vertical[S1_FIXED_TEXT_MAX];
    char heading[S1_FIXED_TEXT_MAX];
    char rise[S1_FIXED_TEXT_MAX];
    char peak[S1_FIXED_TEXT_MAX];

    telemetry_format_fixed(speed, p_msg->speed_m_s, 3U);
    telemetry_format_fixed(error, p_msg->line_error, 3U);
    telemetry_format_fixed(dist, p_msg->distance_m, 3U);
    telemetry_format_fixed(hump, p_msg->hump_height_m, 3U);
    telemetry_format_fixed(obst, p_msg->obstacle_distance_cm, 1U);
    telemetry_format_fixed(roll, p_msg->imu.tilt_roll_deg, 1U);
    telemetry_format_fixed(pitch, p_msg->imu.tilt_pitch_deg, 1U);
    telemetry_format_fixed(turn, p_msg->imu.turn_rate_dps, 1U);
    telemetry_format_fixed(vertical, p_msg->imu.vertical_acceleration_g, 3U);
    telemetry_format_fixed(heading, p_msg->imu.magnetic_heading_deg, 1U);
    telemetry_format_fixed(rise, p_msg->imu.terrain_rise_m, 3U);
    telemetry_format_fixed(peak, p_msg->imu.terrain_peak_m, 3U);

    (void) tm_sprintf((UB *) line,
                      (UB *) "{\"t_ms\":%u,\"mission\":%u,\"speed\":%s,"
                      "\"line\":%s,\"distance\":%s,\"hump_m\":%s,"
                      "\"obstacle_cm\":%s,\"barcode\":%u,\"fault\":%u,"
                      "\"imu_valid\":%u,\"imu_calibrated\":%u,"
                      "\"motion\":%u,\"roll_deg\":%s,\"pitch_deg\":%s,"
                      "\"turn_dps\":%s,\"vertical_g\":%s,"
                      "\"hump_active\":%u,\"hump_valid\":%u,"
                      "\"hump_count\":%u,\"imu_saturated\":%u,"
                      "\"tilt_valid\":%u,\"shock_count\":%u,"
                      "\"hump_rejected_count\":%u,\"imu_model\":\"LSM303DLHC\","
                      "\"heading_deg\":%s,\"heading_valid\":%u,"
                      "\"turn_valid\":%u,\"mag_disturbed\":%u,"
                      "\"has_gyro\":%u,\"turn_estimated\":%u,"
                      "\"height_approx\":%u,\"mag_calibrated\":%u,"
                      "\"rise_m\":%s,\"terrain_peak_m\":%s,"
                      "\"height_tracking_valid\":%u,\"height_uses_odometry\":%u,"
                      "\"height_gap_pending\":%u,\"height_gap_count\":%u}\n",
                      (unsigned int) p_msg->timestamp_ms,
                      (unsigned int) p_msg->mission_state,
                      speed, error, dist, hump, obst,
                      (unsigned int) p_msg->last_barcode,
                      (unsigned int) p_msg->fault_code,
                      (unsigned int) p_msg->imu.b_sensor_valid,
                      (unsigned int) p_msg->imu.b_calibrated,
                      (unsigned int) p_msg->imu.motion_state,
                      roll, pitch, turn, vertical,
                      (unsigned int) p_msg->imu.b_hump_active,
                      (unsigned int) p_msg->imu.b_hump_height_valid,
                      (unsigned int) p_msg->imu.hump_count,
                      (unsigned int) p_msg->imu.b_saturated,
                      (unsigned int) p_msg->imu.b_tilt_valid,
                      (unsigned int) p_msg->imu.shock_count,
                      (unsigned int) p_msg->imu.rejected_hump_count,
                      heading,
                      (unsigned int) p_msg->imu.b_heading_valid,
                      (unsigned int) p_msg->imu.b_turn_rate_valid,
                      (unsigned int) p_msg->imu.b_magnetic_disturbed,
                      (unsigned int) p_msg->imu.b_has_gyro,
                      (unsigned int) p_msg->imu.b_turn_rate_estimated,
                      (unsigned int) p_msg->imu.b_height_approximate,
                      (unsigned int) p_msg->imu.b_mag_calibrated,
                      rise, peak,
                      (unsigned int) p_msg->imu.b_height_tracking_valid,
                      (unsigned int) p_msg->imu.b_height_uses_odometry,
                      (unsigned int) p_msg->imu.b_height_gap_pending,
                      (unsigned int) p_msg->imu.height_gap_count);
    (void) tm_putstring((UB *) line);
}

/*!
 * @brief Telemetry task: print snapshots as they arrive, plus a
 *        heartbeat once per ROBOT_TELEMETRY_PERIOD_MS.
 */
void
telemetry_task (INT stacd, void * p_exinf)
{
    telemetry_message_t message;
    uint32_t            last_heartbeat_ms = board_millis();
    uint32_t            now_ms;
    char                line[S1_FIXED_TEXT_MAX * 2U];

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        if (tk_rcv_mbf(gh_telemetry_mbf, &message,
                       (TMO) S1_RECEIVE_TIMEOUT_MS) > 0)
        {
            telemetry_print(&message);
        }

        now_ms = board_millis();

        if ((now_ms - last_heartbeat_ms) >= ROBOT_TELEMETRY_PERIOD_MS)
        {
            (void) tm_sprintf((UB *) line, (UB *) "{\"heartbeat_ms\":%u}\n",
                              (unsigned int) now_ms);
            (void) tm_putstring((UB *) line);
            last_heartbeat_ms = now_ms;
        }
    }
}

/*** end of file ***/
