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

#define TELEMETRY_LINE_MAX      (256U)
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

    telemetry_format_fixed(speed, p_msg->speed_m_s, 3U);
    telemetry_format_fixed(error, p_msg->line_error, 3U);
    telemetry_format_fixed(dist, p_msg->distance_m, 3U);
    telemetry_format_fixed(hump, p_msg->hump_height_m, 3U);
    telemetry_format_fixed(obst, p_msg->obstacle_distance_cm, 1U);

    (void) tm_sprintf((UB *) line,
                      (UB *) "{\"t_ms\":%u,\"mission\":%u,\"speed\":%s,"
                      "\"line\":%s,\"distance\":%s,\"hump_m\":%s,"
                      "\"obstacle_cm\":%s,\"barcode\":%u,\"fault\":%u}\n",
                      (unsigned int) p_msg->timestamp_ms,
                      (unsigned int) p_msg->mission_state,
                      speed, error, dist, hump, obst,
                      (unsigned int) p_msg->last_barcode,
                      (unsigned int) p_msg->fault_code);
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
