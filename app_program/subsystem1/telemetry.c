/** @file telemetry.c
 *
 * @brief Buddy 1 telemetry: USB console output and MQTT telemetry bridge.
 *
 * tm_printf() has no floating-point conversions, so floats are formatted
 * as fixed-point text by telemetry_format_fixed().
 *
 * Each USB JSON line is written using a single tm_putstring() call.
 * MQTT telemetry is passed through a non-blocking micro T-Kernel
 * message buffer and retrieved by the Wi-Fi task.
 *
 * Tribunal 1: with USE_BIT_PACKING, MQTT uses 4-byte binary records
 * (status, ramp-height and metadata) with no JSON formatting on the wire.
 * The USB console keeps the readable text.
 */

#include "subsystem1.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../controller/controller.h"
#include "../subsystem4/imu_height.h"
#include "../common/fault_mask.h"
#include "telemetry_codec.h"

#if USE_BIT_PACKING
_Static_assert(S1_MQTT_MESSAGE_SIZE == TELEMETRY_PACKED_SIZE,
               "MQTT message buffer must match the packet size");
#endif

#if TK_SUPPORT_SMP && TM_CONSOLE_USB_CDC
#include "../../kernel/knlinc/smp_lock.h"
extern volatile UW tm_console_max_wait_us[TK_MAX_CORE];
extern volatile UW tm_console_max_hold_us[TK_MAX_CORE];
extern volatile UW tm_usb_dropped;
#endif

#include <stdint.h>

#define TELEMETRY_LINE_MAX (896U)
#define TELEMETRY_FIXED_LIMIT (1000000.0f)

#if USE_BIT_PACKING
#define TELEMETRY_HEIGHT_STATE (6U)

/* A second four-byte record carries live ramp values without changing the
   compact status packet required by the telemetry interface. */
static void telemetry_encode_height(telemetry_message_t const *p_message,
                                    uint8_t packet[4])
{
    float rise_mm = (p_message->imu.b_height_interrupted_available
                         ? p_message->imu.interrupted_rise_m
                         : p_message->imu.terrain_rise_m) * 1000.0f;
    float peak_mm = (p_message->imu.b_height_interrupted_available
                         ? p_message->imu.interrupted_peak_m
                         : p_message->imu.terrain_peak_m) * 1000.0f;
    float hump_mm = p_message->hump_height_m * 1000.0f;
    uint32_t flags = 0U;

    if (p_message->imu.b_tilt_valid) flags |= 0x01U;
    if (p_message->imu.b_calibrated) flags |= 0x02U;
    if (p_message->imu.b_height_tracking_valid) flags |= 0x04U;
    if (p_message->imu.b_hump_active) flags |= 0x08U;
    if (p_message->imu.b_hump_height_valid) flags |= 0x10U;

    if (rise_mm > 127.0f) rise_mm = 127.0f;
    if (rise_mm < -128.0f) rise_mm = -128.0f;
    if (peak_mm < 0.0f) peak_mm = 0.0f;
    if (peak_mm > 255.0f) peak_mm = 255.0f;
    if (hump_mm < 0.0f) hump_mm = 0.0f;
    if (hump_mm > 255.0f) hump_mm = 255.0f;

    packet[0] = (uint8_t)(TELEMETRY_HEIGHT_STATE | (flags << 3));
    packet[1] = (uint8_t)(int8_t)rise_mm;
    packet[2] = (uint8_t)peak_mm;
    packet[3] = (uint8_t)hump_mm;
}

static void telemetry_encode_metadata(telemetry_message_t const *p_message,
                                      uint8_t packet[4])
{
    packet[0] = (uint8_t)(7U | (p_message->b_simulation ? 0x08U : 0U));
    packet[1] = ((p_message->last_barcode >= TELEMETRY_ASCII_FIRST) &&
                 (p_message->last_barcode <= TELEMETRY_ASCII_LAST))
                    ? (uint8_t)p_message->last_barcode : 0U;
    packet[2] = (((uint8_t)p_message->last_junction_char >=
                  TELEMETRY_ASCII_FIRST) &&
                 ((uint8_t)p_message->last_junction_char <=
                  TELEMETRY_ASCII_LAST))
                    ? (uint8_t)p_message->last_junction_char : 0U;
    packet[3] = (uint8_t)(((uint32_t)p_message->last_junction_action & 0x03U) |
                          (((uint32_t)p_message->junction_count & 0x3FU) << 2));
}
#endif

/*!
 * @brief Queue a telemetry snapshot without waiting.
 *
 * @return true if queued.
 */
bool telemetry_publish(telemetry_message_t const *p_message)
{
    bool b_ok = false;

    if ((NULL != p_message) && (gh_telemetry_mbf > 0))
    {
        b_ok = (E_OK == tk_snd_mbf(gh_telemetry_mbf, (void *)p_message,
                                   (INT)sizeof(*p_message), TMO_POL));
    }

    return b_ok;
}

/*!
 * @brief Forward a user command to the controller without waiting.
 *
 * @return true if queued.
 */
bool telemetry_submit_command(control_command_t const *p_command)
{
    bool b_ok = false;

    if ((NULL != p_command) && (gh_control_mbf > 0))
    {
        b_ok = (E_OK == tk_snd_mbf(gh_control_mbf, (void *)p_command,
                                   (INT)sizeof(*p_command), TMO_POL));
    }

    return b_ok;
}

/*
 * Request STOP when MQTT reports a failed or lost connection.
 * This is best effort: a full control message buffer can reject STOP.
 */
void mqtt_utk_disconnected(void)
{
    control_command_t command = {0};

#if (CTRL_AUTO_START_MS > 0U)
    /* Autonomous run: losing the dashboard must not stop the robot. */
    return;
#endif

    fault_mask_set(FAULT_BIT_COMMS);
    command.type = CONTROL_STOP;
    command.value = 0.0f;

    if (!telemetry_submit_command(&command))
    {
        tm_printf((UB *)"S1: WARNING: MQTT disconnect STOP queue failed\n");
    }
}

/*
 * Buddy 1: Accept START, STOP, or SPEED commands from MQTT.
 * Payloads: START, STOP, SPEED:0.35 (ASCII, no trailing whitespace).
 * Parse thousandths as integers to avoid floating-point boundary errors.
 * Valid speed range comes from robot_config.h.
 */
void mqtt_utk_control_rx(char const *p_payload, uint32_t length)
{
    static char const prefix[] = "SPEED:";
    uint32_t pos;
    uint32_t whole = 0U;
    uint32_t fractional = 0U;
    uint32_t fractional_digits = 0U;
    uint32_t speed_milli;
    bool seen_dot = false;
    bool seen_digit = false;
    control_command_t command;

    /* Exact START/STOP payloads are accepted before SPEED parsing. */
    if (NULL == p_payload)
    {
        return;
    }

    if ((5U == length) &&
        ('S' == p_payload[0]) && ('T' == p_payload[1]) &&
        ('A' == p_payload[2]) && ('R' == p_payload[3]) &&
        ('T' == p_payload[4]))
    {
        command.type = CONTROL_START;
        command.value = 0.0f;
        (void)telemetry_submit_command(&command);
        return;
    }

    if ((4U == length) &&
        ('S' == p_payload[0]) && ('T' == p_payload[1]) &&
        ('O' == p_payload[2]) && ('P' == p_payload[3]))
    {
        command.type = CONTROL_STOP;
        command.value = 0.0f;
        (void)telemetry_submit_command(&command);
        return;
    }

    if ((length <= 6U) || (length > 16U))
    {
        return;
    }

    for (pos = 0U; pos < 6U; pos++)
    {
        if (p_payload[pos] != prefix[pos])
        {
            return;
        }
    }

    for (pos = 6U; pos < length; pos++)
    {
        char const c = p_payload[pos];

        if ((c >= '0') && (c <= '9'))
        {
            uint32_t const digit = (uint32_t)(c - '0');
            seen_digit = true;

            if (!seen_dot)
            {
                if (whole > 1U)
                {
                    return;
                }
                whole = (whole * 10U) + digit;
            }
            else
            {
                if (fractional_digits >= 3U)
                {
                    return;
                }
                fractional = (fractional * 10U) + digit;
                fractional_digits++;
            }
        }
        else if ((c == '.') && !seen_dot)
        {
            seen_dot = true;
        }
        else
        {
            return;
        }
    }

    if (!seen_digit || (whole > 1U))
    {
        return;
    }

    while (fractional_digits < 3U)
    {
        fractional *= 10U;
        fractional_digits++;
    }

    speed_milli = (whole * 1000U) + fractional;

    if ((speed_milli < (uint32_t)(ROBOT_MIN_SPEED * 1000.0f + 0.5f)) ||
        (speed_milli > (uint32_t)(ROBOT_MAX_SPEED * 1000.0f + 0.5f)))
    {
        return;
    }

    command.type = CONTROL_SET_SPEED;
    command.value = (float)speed_milli / 1000.0f;
    (void)telemetry_submit_command(&command);
}

/*!
 * @brief Format a float as fixed-point text, e.g. -0.350.
 *
 * @param[out] p_text    At least S1_FIXED_TEXT_MAX bytes.
 * @param[in]  value     Value; clamped to +-1e6.
 * @param[in]  decimals  1 or 3 (anything else is treated as 3).
 */
void telemetry_format_fixed(char *p_text, float value, uint32_t decimals)
{
    uint32_t const scale = (1U == decimals) ? 10U : 1000U;
    bool const b_neg = (value < 0.0f);
    float mag = b_neg ? -value : value;
    uint32_t scaled = 0U;

    if (mag > TELEMETRY_FIXED_LIMIT)
    {
        mag = TELEMETRY_FIXED_LIMIT;
    }

    scaled = (uint32_t)((mag * (float)scale) + 0.5f);

    if (1U == decimals)
    {
        (void)tm_sprintf((UB *)p_text, (UB *)"%s%u.%01u",
                         (b_neg && (0U != scaled)) ? "-" : "",
                         (unsigned int)(scaled / scale),
                         (unsigned int)(scaled % scale));
    }
    else
    {
        (void)tm_sprintf((UB *)p_text, (UB *)"%s%u.%03u",
                         (b_neg && (0U != scaled)) ? "-" : "",
                         (unsigned int)(scaled / scale),
                         (unsigned int)(scaled % scale));
    }
}

/*!
 * @brief Print one telemetry snapshot as a JSON line.
 */
static void
telemetry_trace_fixed(char *p_text, float value)
{
    if ((value >= -TELEMETRY_FIXED_LIMIT) &&
        (value <= TELEMETRY_FIXED_LIMIT))
    {
        telemetry_format_fixed(p_text, value, 3U);
    }
    else
    {
        p_text[0] = 'n';
        p_text[1] = 'u';
        p_text[2] = 'l';
        p_text[3] = 'l';
        p_text[4] = '\0';
    }
}

static void
telemetry_print_height_trace(telemetry_message_t const *p_msg)
{
    char line[640];
    char pitch[S1_FIXED_TEXT_MAX];
    char roll[S1_FIXED_TEXT_MAX];
    char ax[S1_FIXED_TEXT_MAX];
    char ay[S1_FIXED_TEXT_MAX];
    char az[S1_FIXED_TEXT_MAX];
    char ds[S1_FIXED_TEXT_MAX];
    char rise[S1_FIXED_TEXT_MAX];
    char peak[S1_FIXED_TEXT_MAX];
    char distance[S1_FIXED_TEXT_MAX];
    char left[S1_FIXED_TEXT_MAX];
    char right[S1_FIXED_TEXT_MAX];

    telemetry_trace_fixed(pitch, p_msg->imu.tilt_pitch_deg);
    telemetry_trace_fixed(roll, p_msg->imu.tilt_roll_deg);
    telemetry_trace_fixed(ax, p_msg->imu.acceleration_x_g);
    telemetry_trace_fixed(ay, p_msg->imu.acceleration_y_g);
    telemetry_trace_fixed(az, p_msg->imu.acceleration_z_g);
    telemetry_trace_fixed(ds, p_msg->motion_delta_m * 1000.0f);
    telemetry_trace_fixed(rise, p_msg->imu.terrain_rise_m * 1000.0f);
    telemetry_trace_fixed(peak, p_msg->imu.terrain_peak_m * 1000.0f);
    telemetry_trace_fixed(distance, p_msg->motion.distance_m);
    telemetry_trace_fixed(left, p_msg->motion.left_speed_m_s);
    telemetry_trace_fixed(right, p_msg->motion.right_speed_m_s);
    (void)tm_sprintf((UB *)line,
                     (UB *)"{\"height_trace\":1,\"seq\":%u,\"dropped\":%u,"
                           "\"t_ms\":%u,\"imu_ms\":%u,\"dt_ms\":%u,"
                           "\"pitch_deg\":%s,\"roll_deg\":%s,"
                           "\"ax_g\":%s,\"ay_g\":%s,\"az_g\":%s,"
                           "\"left_ticks\":%d,\"right_ticks\":%d,"
                           "\"left_m_s\":%s,\"right_m_s\":%s,"
                           "\"distance_m\":%s,\"ds_mm\":%s,"
                           "\"rise_mm\":%s,\"peak_mm\":%s,"
                           "\"tilt_valid\":%u,\"tracking_valid\":%u,"
                           "\"active\":%u,\"gap\":%u,\"imu_dt_ms\":%u,"
                           "\"timing_gap\":%u,\"acquisition_us\":%u}\n",
                     (unsigned int)p_msg->trace_sequence,
                     (unsigned int)p_msg->trace_dropped,
                     (unsigned int)p_msg->timestamp_ms,
                     (unsigned int)p_msg->imu_ms,
                     (unsigned int)p_msg->motion_interval_ms,
                     pitch, roll, ax, ay, az,
                     (int)p_msg->motion.left_ticks,
                     (int)p_msg->motion.right_ticks,
                     left, right, distance, ds, rise, peak,
                     (unsigned int)p_msg->imu.b_tilt_valid,
                     (unsigned int)p_msg->imu.b_height_tracking_valid,
                     (unsigned int)p_msg->imu.b_hump_active,
                     (unsigned int)p_msg->imu.b_height_gap_pending,
                     (unsigned int)p_msg->imu.sample_interval_ms,
                     (unsigned int)p_msg->imu.b_timing_gap,
                     (unsigned int)p_msg->imu.acquisition_us);
    (void)tm_putstring((UB *)line);
}

static void telemetry_print(telemetry_message_t const *p_msg)
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

#if S1_SERIAL_COMPACT
    {
        static char const * const p_names[] =
            {"IDLE", "FOLLOW", "AVOID", "FAULT"};
        unsigned int const mission = (unsigned int) p_msg->mission_state;
        unsigned int const secs10  = (unsigned int) (p_msg->timestamp_ms / 100U);
        char bc[4] = "-";

        if ((p_msg->last_barcode >= 32U) && (p_msg->last_barcode < 127U))
        {
            bc[0] = (char) p_msg->last_barcode;
            bc[1] = '\0';
        }

        (void) tm_sprintf((UB *) line,
                          (UB *) "[%4u.%u s] %-6s spd=%s line=%s dist=%sm "
                          "pitch=%s roll=%s obst=%scm bc=%s fault=%u "
                          "faults=0x%02x%s%s\n",
                          secs10 / 10U, secs10 % 10U,
                          p_names[(mission < 4U) ? mission : 3U],
                          speed, error, dist, pitch, roll, obst, bc,
                          (unsigned int) p_msg->fault_code,
                          (unsigned int) p_msg->fault_mask,
                          p_msg->imu.b_calibrated ? "" : " IMU-CAL",
                          (p_msg->imu.b_hump_height_valid) ?
                            " H-VALID" : (p_msg->imu.b_hump_active) ? " H-ACTIVE" : "");
        (void) tm_putstring((UB *) line);
    }
#else
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
#endif /* S1_SERIAL_COMPACT */
    if (p_msg->imu.b_height_interrupted_available)
    {
        telemetry_format_fixed(rise, p_msg->imu.interrupted_rise_m, 3U);
        telemetry_format_fixed(peak, p_msg->imu.interrupted_peak_m, 3U);
        (void)tm_sprintf((UB *)line,
                         (UB *)"{\"height_interrupted\":1,\"t_ms\":%u,"
                               "\"interrupted_height_valid\":0,"
                               "\"interrupted_rise_m\":%s,"
                               "\"interrupted_peak_m\":%s,"
                               "\"interrupted_reason\":\"%s\"}\n",
                         (unsigned int)p_msg->timestamp_ms, rise, peak,
                         imu_height_rejection_name(
                             (imu_height_rejection_t)
                                 p_msg->imu.height_interrupted_reason));
        (void)tm_putstring((UB *)line);
    }
}

/*!
 * @brief Retrieve one telemetry snapshot as JSON for MQTT.
 *
 * Called by the Wi-Fi task through the MQTT telemetry interface.
 * Does not block if the message buffer is empty.
 *
 * @param[out] buffer       Destination for JSON.
 * @param[in]  buffer_size  Available destination capacity.
 *
 * @return JSON payload length, or 0 if no data is available.
 */
uint32_t mqtt_utk_telemetry_read(char *buffer, uint32_t buffer_size)
{
#if USE_BIT_PACKING
    uint8_t  packet[TELEMETRY_PACKED_SIZE];
    INT received;

    if ((NULL == buffer) || (buffer_size < TELEMETRY_PACKED_SIZE) ||
        (gh_mqtt_telemetry_mbf <= 0))
    {
        return 0U;
    }

    received = tk_rcv_mbf(gh_mqtt_telemetry_mbf, packet, TMO_POL);
    if (received != (INT)TELEMETRY_PACKED_SIZE)
    {
        return 0U;
    }

    for (INT idx = 0; idx < received; idx++)
    {
        buffer[idx] = (char) packet[idx];
    }

    return (uint32_t)received;
#else
    telemetry_message_t message;
    char speed[S1_FIXED_TEXT_MAX];
    char error[S1_FIXED_TEXT_MAX];
    char dist[S1_FIXED_TEXT_MAX];
    char hump[S1_FIXED_TEXT_MAX];
    char obst[S1_FIXED_TEXT_MAX];
    char roll[S1_FIXED_TEXT_MAX];
    char pitch[S1_FIXED_TEXT_MAX];
    char rise[S1_FIXED_TEXT_MAX];
    char peak[S1_FIXED_TEXT_MAX];
    int len;

    if ((buffer == NULL) ||
        (buffer_size < TELEMETRY_LINE_MAX) ||
        (gh_mqtt_telemetry_mbf <= 0))
    {
        return 0U;
    }

    if (tk_rcv_mbf(gh_mqtt_telemetry_mbf, &message, TMO_POL) <= 0)
    {
        return 0U;
    }

    telemetry_format_fixed(speed, message.speed_m_s, 3U);
    telemetry_format_fixed(error, message.line_error, 3U);
    telemetry_format_fixed(dist, message.distance_m, 3U);
    telemetry_format_fixed(hump, message.hump_height_m, 3U);
    telemetry_format_fixed(obst, message.obstacle_distance_cm, 1U);

    telemetry_format_fixed(roll, message.imu.tilt_roll_deg, 1U);
    telemetry_format_fixed(pitch, message.imu.tilt_pitch_deg, 1U);
    telemetry_format_fixed(rise, message.imu.terrain_rise_m, 3U);
    telemetry_format_fixed(peak, message.imu.terrain_peak_m, 3U);

    len = tm_sprintf(
        (UB *)buffer,
        (UB *)"{\"t_ms\":%u,\"mission\":%u,\"speed\":%s,"
              "\"line\":%s,\"distance\":%s,\"hump_m\":%s,"
              "\"obstacle_cm\":%s,\"barcode\":%u,\"fault\":%u,"
              "\"imu_valid\":%u,\"imu_calibrated\":%u,"
              "\"motion\":%u,\"roll_deg\":%s,\"pitch_deg\":%s,"
              "\"hump_active\":%u,\"hump_valid\":%u,"
              "\"hump_count\":%u,\"tilt_valid\":%u,"
              "\"height_approx\":%u,\"rise_m\":%s,"
              "\"terrain_peak_m\":%s,\"height_tracking_valid\":%u,"
              "\"height_gap_pending\":%u}",
        (unsigned int)message.timestamp_ms,
        (unsigned int)message.mission_state,
        speed,
        error,
        dist,
        hump,
        obst,
        (unsigned int)message.last_barcode,
        (unsigned int)message.fault_code,
        (unsigned int)message.imu.b_sensor_valid,
        (unsigned int)message.imu.b_calibrated,
        (unsigned int)message.imu.motion_state,
        roll, pitch,
        (unsigned int)message.imu.b_hump_active,
        (unsigned int)message.imu.b_hump_height_valid,
        (unsigned int)message.imu.hump_count,
        (unsigned int)message.imu.b_tilt_valid,
        (unsigned int)message.imu.b_height_approximate,
        rise, peak,
        (unsigned int)message.imu.b_height_tracking_valid,
        (unsigned int)message.imu.b_height_gap_pending);

    return (len > 0) ? (uint32_t)len : 0U;
#endif /* USE_BIT_PACKING */
}

/*!
 * @brief Telemetry task: print snapshots as they arrive, forward them
 *        to MQTT, and print a periodic USB heartbeat.
 */
void telemetry_task(INT stacd, void *p_exinf)
{
    telemetry_message_t message;
    uint32_t last_heartbeat_ms = board_millis();
    uint32_t now_ms;
    char line[S1_FIXED_TEXT_MAX * 2U];
#if TM_MQTT
    bool b_mqtt_queue_error = false;
#if USE_BIT_PACKING
    uint8_t packet[TELEMETRY_PACKED_SIZE];
    bool b_obstacle_slot = false;
#endif
#endif

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        if (tk_rcv_mbf(gh_telemetry_mbf, &message,
                       (TMO)S1_RECEIVE_TIMEOUT_MS) > 0)
        {
            if (message.b_height_trace)
            {
                telemetry_print_height_trace(&message);
            }
            else
            {
                telemetry_print(&message);
#if TM_MQTT
                /* Commissioning traces stay on USB; MQTT receives snapshots. */
                if (gh_mqtt_telemetry_mbf > 0)
                {
#if USE_BIT_PACKING
                    /* Byte 3 alternates hump / obstacle each packet. */
                    (void)telemetry_encode(&message, b_obstacle_slot,
                                           packet, sizeof(packet));
                    b_obstacle_slot = !b_obstacle_slot;
                    if (E_OK != tk_snd_mbf(gh_mqtt_telemetry_mbf, packet,
                                           (INT)sizeof(packet), TMO_POL))
#else
                    if (E_OK != tk_snd_mbf(gh_mqtt_telemetry_mbf,
                                           (void *)&message,
                                           (INT)sizeof(message), TMO_POL))
#endif
                    {
                        fault_mask_set(FAULT_BIT_COMMS);
                        if (!b_mqtt_queue_error)
                        {
                            tm_printf((UB *)"S1: MQTT telemetry buffer full; "
                                            "snapshot not delivered\n");
                            b_mqtt_queue_error = true;
                        }
                    }
                    else
                    {
                        b_mqtt_queue_error = false;
                    }
#if USE_BIT_PACKING
                    /* Dashboard height values use a separate four-byte
                       packed record (state 6), while the status packet
                       retains its existing compact wire layout. */
                    telemetry_encode_height(&message, packet);
                    if (E_OK != tk_snd_mbf(gh_mqtt_telemetry_mbf, packet,
                                           (INT)sizeof(packet), TMO_POL) &&
                        !b_mqtt_queue_error)
                    {
                        tm_printf((UB *)"S1: MQTT height record dropped\n");
                        b_mqtt_queue_error = true;
                    }
                    telemetry_encode_metadata(&message, packet);
                    if (E_OK != tk_snd_mbf(gh_mqtt_telemetry_mbf, packet,
                                           (INT)sizeof(packet), TMO_POL) &&
                        !b_mqtt_queue_error)
                    {
                        tm_printf((UB *)"S1: MQTT metadata record dropped\n");
                        b_mqtt_queue_error = true;
                    }
#endif
                }
#endif
            }
        }

        now_ms = board_millis();

#if S1_SERIAL_DIAG
        if ((now_ms - last_heartbeat_ms) >= ROBOT_TELEMETRY_PERIOD_MS)
        {
            (void)tm_sprintf((UB *)line, (UB *)"{\"heartbeat_ms\":%u}\n",
                             (unsigned int)now_ms);
            (void)tm_putstring((UB *)line);
#if TK_SUPPORT_SMP && TM_CONSOLE_USB_CDC
            {
                KNL_LOCK_DIAG locks;

                knl_kernel_lock_audit(&locks);
                tm_printf((UB *)"{\"timing_diag\":1,\"t_ms\":%u,"
                                "\"kernel_hold_us\":%u,\"kernel_wait_us\":%u,"
                                "\"console_hold0_us\":%u,\"console_hold1_us\":%u,"
                                "\"console_wait0_us\":%u,\"console_wait1_us\":%u,"
                                "\"usb_dropped_bytes\":%u}\n",
                          (unsigned int)now_ms,
                          (unsigned int)locks.max_hold_us,
                          (unsigned int)locks.max_wait_us,
                          (unsigned int)tm_console_max_hold_us[0],
                          (unsigned int)tm_console_max_hold_us[1],
                          (unsigned int)tm_console_max_wait_us[0],
                          (unsigned int)tm_console_max_wait_us[1],
                          (unsigned int)tm_usb_dropped);
            }
#endif
            last_heartbeat_ms = now_ms;
        }
#else
        (void) now_ms;
        (void) last_heartbeat_ms;
        (void) line;
#endif /* S1_SERIAL_DIAG */
    }
}

/*** end of file ***/
