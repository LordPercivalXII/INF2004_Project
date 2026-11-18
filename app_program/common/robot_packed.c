/** @file robot_packed.c
 *
 * @brief Pack/unpack the minimum-bit queue formats of robot_packed.h.
 *
 * Pure C with no kernel calls, so it also builds and runs on a PC
 * (testing/integration/test_packing_host.c).  Every conversion rounds to
 * nearest and saturates to its field; non-finite inputs become zero.
 */

#include <stddef.h>

#include "robot_packed.h"

#define PACK_DIR_NONE           (0U)
#define PACK_DIR_LEFT           (1U)
#define PACK_DIR_RIGHT          (2U)

#define PACK_ANGLE_LEFT         (0U)
#define PACK_ANGLE_AHEAD        (1U)
#define PACK_ANGLE_RIGHT        (2U)
#define PACK_ANGLE_AHEAD_DEG    (7.5f)  /* half a 15 deg beam: "ahead"   */
#define PACK_ANGLE_SIDE_DEG     (30.0f) /* reported for left/right index */

#define PACK_ACTION_MAX         (7U)
#define PACK_COMMAND_MAX        (7U)
#define PACK_ASCII_MASK         (0x7FU)
#define PACK_MOTION_ARG_MAX     (4095U)

static bool
pack_is_finite (float value)
{
    /* NaN fails the self-compare; +-inf fails the range test. */
    return ((value == value) && (value <= 3.0e38f) && (value >= -3.0e38f));
}

static int32_t
pack_round (float value)
{
    return ((value >= 0.0f) ? (int32_t) (value + 0.5f)
                            : (int32_t) (value - 0.5f));
}

/*!
 * @brief Signed value -> offset-binary code, saturating.
 *
 * @param[in] value  Physical value.
 * @param[in] scale  Codes per unit.
 * @param[in] bias   Code of zero.
 * @param[in] max    Largest code (smallest is 0).
 */
uint32_t
pack_offset (float value, float scale, int32_t bias, int32_t max)
{
    int32_t code = bias;
    float   scaled;

    if (pack_is_finite(value))
    {
        scaled = value * scale;

        /* Clamp before converting so the cast cannot overflow. */
        if (scaled > (float) (max - bias))
        {
            code = max;
        }
        else if (scaled < (float) (-bias))
        {
            code = 0;
        }
        else
        {
            code = pack_round(scaled) + bias;
        }
    }

    return ((uint32_t) code);
}

/*!
 * @brief Offset-binary code -> signed value.
 */
float
unpack_offset (uint32_t code, float scale, int32_t bias)
{
    return ((float) ((int32_t) code - bias) / scale);
}

/*!
 * @brief Non-negative value -> unsigned code, saturating at max.
 */
uint32_t
pack_unsigned (float value, float scale, uint32_t max)
{
    uint32_t code = 0U;
    float    scaled;

    if (pack_is_finite(value) && (value > 0.0f))
    {
        scaled = value * scale;
        code   = (scaled >= (float) max) ? max
                                         : (uint32_t) pack_round(scaled);
    }

    return (code);
}

/* ------------------------------------------------------------------ */
/* cmd_q_motion                                                        */
/* ------------------------------------------------------------------ */

/*!
 * @brief Pack a motion command (IF-03).
 *
 * @return false for an unknown type, a TURN whose turn_id is not
 *         1..MOTION_TURN_ID_MAX, or a MOVE longer than 4.095 m.
 */
bool
pack_motion_cmd (motion_command_t const * p_cmd, motion_cmd_packed_t * p_wire)
{
    bool b_ok = true;

    if ((NULL == p_cmd) || (NULL == p_wire))
    {
        return (false);
    }

    p_wire->mode    = (unsigned int) p_cmd->type;
    p_wire->turn_id = 0U;
    p_wire->arg_a   = (unsigned int) PACK_Q12_BIAS;
    p_wire->arg_b   = (unsigned int) PACK_Q12_BIAS;

    switch (p_cmd->type)
    {
        case MOTION_STOP:
        case MOTION_BRAKE:
            break;

        case MOTION_DRIVE:
            p_wire->arg_a = pack_offset(p_cmd->left_speed, PACK_SPEED_SCALE,
                                        PACK_Q12_BIAS, PACK_Q12_MAX);
            p_wire->arg_b = pack_offset(p_cmd->right_speed, PACK_SPEED_SCALE,
                                        PACK_Q12_BIAS, PACK_Q12_MAX);
            break;

        case MOTION_TURN:
            b_ok = (p_cmd->turn_id >= 1U) &&
                   (p_cmd->turn_id <= MOTION_TURN_ID_MAX);
            p_wire->turn_id = (unsigned int) (p_cmd->turn_id &
                                              MOTION_TURN_ID_MAX);
            p_wire->arg_a = pack_offset(p_cmd->turn_angle_deg,
                                        PACK_ANGLE_SCALE,
                                        PACK_Q12_BIAS, PACK_Q12_MAX);
            break;

        case MOTION_MOVE_DISTANCE:
            b_ok = pack_is_finite(p_cmd->target_distance_m) &&
                   ((p_cmd->target_distance_m * PACK_DISTANCE_SCALE) <=
                    (float) PACK_MOTION_ARG_MAX);
            p_wire->arg_a = pack_offset(p_cmd->left_speed, PACK_SPEED_SCALE,
                                        PACK_Q12_BIAS, PACK_Q12_MAX);
            p_wire->arg_b = pack_unsigned(p_cmd->target_distance_m,
                                          PACK_DISTANCE_SCALE,
                                          PACK_MOTION_ARG_MAX);
            break;

        default:
            b_ok = false;
            break;
    }

    return (b_ok);
}

/*!
 * @brief Unpack a motion command (IF-03).
 */
void
unpack_motion_cmd (motion_cmd_packed_t const * p_wire,
                   motion_command_t * p_cmd)
{
    motion_command_t cmd = {0};

    cmd.type = (motion_command_type_t) p_wire->mode;

    switch (cmd.type)
    {
        case MOTION_DRIVE:
            cmd.left_speed  = unpack_offset(p_wire->arg_a, PACK_SPEED_SCALE,
                                            PACK_Q12_BIAS);
            cmd.right_speed = unpack_offset(p_wire->arg_b, PACK_SPEED_SCALE,
                                            PACK_Q12_BIAS);
            break;

        case MOTION_TURN:
            cmd.turn_id        = (uint32_t) p_wire->turn_id;
            cmd.turn_angle_deg = unpack_offset(p_wire->arg_a,
                                               PACK_ANGLE_SCALE,
                                               PACK_Q12_BIAS);
            break;

        case MOTION_MOVE_DISTANCE:
            cmd.left_speed        = unpack_offset(p_wire->arg_a,
                                                  PACK_SPEED_SCALE,
                                                  PACK_Q12_BIAS);
            cmd.right_speed       = cmd.left_speed;
            cmd.target_distance_m = (float) p_wire->arg_b /
                                    PACK_DISTANCE_SCALE;
            break;

        default:
            /* STOP, BRAKE (and anything unknown) carry no arguments. */
            break;
    }

    *p_cmd = cmd;
}

/* ------------------------------------------------------------------ */
/* nav_q_line + cmd_q_barcode                                          */
/* ------------------------------------------------------------------ */

/*!
 * @brief Pack a line event.  barcode_char carries the newly decoded
 *        character, or else the junction's character.
 */
void
pack_line (line_event_t const * p_line, line_packed_t * p_wire)
{
    uint32_t const command = (p_line->junction_action > PACK_COMMAND_MAX)
                             ? (uint32_t) BARCODE_CMD_NONE
                             : (uint32_t) p_line->junction_action;
    bool const     b_char  = ('\0' != p_line->junction_char);

    p_wire->nav.error_q12  = pack_offset(p_line->line_error, PACK_LINE_SCALE,
                                         PACK_Q12_BIAS, PACK_Q12_MAX);
    p_wire->nav.b_junction = p_line->b_junction ? 1U : 0U;
    p_wire->nav.b_fault    = p_line->b_line_detected ? 0U : 1U;
    p_wire->nav.b_scanning = p_line->b_barcode_scanning ? 1U : 0U;
    p_wire->nav.b_barcode  = p_line->b_barcode_valid ? 1U : 0U;

    p_wire->cmd.command    = (unsigned int) command;
    p_wire->cmd.b_char     = b_char ? 1U : 0U;
    p_wire->cmd.b_reversed = p_line->b_barcode_reversed ? 1U : 0U;
    p_wire->cmd.reserved   = 0U;

    if (p_line->b_barcode_valid)
    {
        p_wire->barcode_char = (uint8_t) (p_line->barcode & PACK_ASCII_MASK);
    }
    else if (b_char)
    {
        p_wire->barcode_char = (uint8_t) ((uint8_t) p_line->junction_char &
                                          PACK_ASCII_MASK);
    }
    else
    {
        p_wire->barcode_char = 0U;
    }
}

/*!
 * @brief Unpack a line event.  barcode_text holds the one character.
 */
void
unpack_line (line_packed_t const * p_wire, line_event_t * p_line)
{
    line_event_t line = {0};
    char const   symbol = (char) p_wire->barcode_char;

    line.line_error         = unpack_offset(p_wire->nav.error_q12,
                                            PACK_LINE_SCALE, PACK_Q12_BIAS);
    line.b_junction         = (0U != p_wire->nav.b_junction);
    line.b_line_detected    = (0U == p_wire->nav.b_fault);
    line.b_barcode_scanning = (0U != p_wire->nav.b_scanning);
    line.b_barcode_valid    = (0U != p_wire->nav.b_barcode);
    line.b_barcode_reversed = (0U != p_wire->cmd.b_reversed);
    line.junction_action    = (uint16_t) p_wire->cmd.command;

    if (line.b_barcode_valid)
    {
        line.barcode         = (uint16_t) p_wire->barcode_char;
        line.barcode_text[0] = symbol;
    }

    if (0U != p_wire->cmd.b_char)
    {
        /* Same event as a new barcode: the junction's own character did
           not fit, but the controller only needs to know one was used. */
        line.junction_char = line.b_barcode_valid ? '?' : symbol;
    }

    *p_line = line;
}

/* ------------------------------------------------------------------ */
/* nav_q_obstacle                                                      */
/* ------------------------------------------------------------------ */

/*!
 * @brief Pack an obstacle event.
 *
 * @param[in] event_ms  The event's timestamp: scan_start_ms is sent as
 *                      the 14-bit time back from it.
 */
void
pack_obstacle (obstacle_event_t const * p_obs, uint32_t event_ms,
               obstacle_packed_t * p_wire)
{
    uint32_t const age_ms   = event_ms - p_obs->scan_start_ms;
    float          clear_cm = p_obs->left_clearance_cm;

    p_wire->action      = ((uint32_t) p_obs->action > PACK_ACTION_MAX)
                          ? (unsigned int) OBSTACLE_ACTION_CONTINUE
                          : (unsigned int) p_obs->action;
    p_wire->b_detected  = p_obs->b_detected ? 1U : 0U;
    p_wire->b_scan_done = p_obs->b_scan_complete ? 1U : 0U;
    p_wire->b_side      = p_obs->b_side_reading ? 1U : 0U;

    if (p_obs->bypass_direction < 0)
    {
        p_wire->bypass_dir = PACK_DIR_LEFT;
    }
    else if (p_obs->bypass_direction > 0)
    {
        p_wire->bypass_dir = PACK_DIR_RIGHT;
        clear_cm = p_obs->right_clearance_cm;
    }
    else
    {
        p_wire->bypass_dir = PACK_DIR_NONE;
        if (p_obs->right_clearance_cm < clear_cm)
        {
            clear_cm = p_obs->right_clearance_cm;
        }
    }

    if (p_obs->center_angle_deg < -PACK_ANGLE_AHEAD_DEG)
    {
        p_wire->angle_idx = PACK_ANGLE_LEFT;
    }
    else if (p_obs->center_angle_deg > PACK_ANGLE_AHEAD_DEG)
    {
        p_wire->angle_idx = PACK_ANGLE_RIGHT;
    }
    else
    {
        p_wire->angle_idx = PACK_ANGLE_AHEAD;
    }

    p_wire->distance_cm = pack_unsigned(p_obs->b_side_reading
                                        ? p_obs->side_distance_cm
                                        : p_obs->center_distance_cm,
                                        1.0f, PACK_U9_MAX);
    p_wire->closest_cm  = pack_unsigned(p_obs->closest_distance_cm, 1.0f,
                                        PACK_U9_MAX);
    p_wire->width_cm    = pack_unsigned(p_obs->estimated_width_cm, 1.0f,
                                        PACK_U9_MAX);
    p_wire->clear_cm    = pack_unsigned(clear_cm, 1.0f, PACK_U9_MAX);
    p_wire->offset_q9   = pack_offset(p_obs->bypass_offset_cm, 1.0f,
                                      PACK_Q9_BIAS, PACK_Q9_MAX);
    p_wire->scan_ms     = ((0U == p_obs->scan_start_ms) ||
                           (age_ms > OBSTACLE_SCAN_MS_MAX))
                          ? (unsigned int) ((0U == p_obs->scan_start_ms)
                                            ? 0U : OBSTACLE_SCAN_MS_MAX)
                          : (unsigned int) age_ms;
    p_wire->reserved    = 0U;
}

/*!
 * @brief Unpack an obstacle event (1 cm resolution).
 */
void
unpack_obstacle (obstacle_packed_t const * p_wire, uint32_t event_ms,
                 obstacle_event_t * p_obs)
{
    obstacle_event_t obs = {0};
    float const      clear_cm = (float) p_wire->clear_cm;

    obs.action              = (obstacle_action_t) p_wire->action;
    obs.b_detected          = (0U != p_wire->b_detected);
    obs.b_scan_complete     = (0U != p_wire->b_scan_done);
    obs.b_side_reading      = (0U != p_wire->b_side);
    obs.scan_start_ms       = event_ms - (uint32_t) p_wire->scan_ms;
    obs.closest_distance_cm = (float) p_wire->closest_cm;
    obs.estimated_width_cm  = (float) p_wire->width_cm;
    obs.bypass_offset_cm    = unpack_offset(p_wire->offset_q9, 1.0f,
                                            PACK_Q9_BIAS);

    if (obs.b_side_reading)
    {
        obs.side_distance_cm = (float) p_wire->distance_cm;
    }
    else
    {
        obs.center_distance_cm = (float) p_wire->distance_cm;
    }

    if (PACK_DIR_LEFT == p_wire->bypass_dir)
    {
        obs.bypass_direction  = -1;
        obs.left_clearance_cm = clear_cm;
    }
    else if (PACK_DIR_RIGHT == p_wire->bypass_dir)
    {
        obs.bypass_direction   = 1;
        obs.right_clearance_cm = clear_cm;
    }
    else
    {
        obs.left_clearance_cm  = clear_cm;
        obs.right_clearance_cm = clear_cm;
    }

    if (PACK_ANGLE_LEFT == p_wire->angle_idx)
    {
        obs.center_angle_deg = -PACK_ANGLE_SIDE_DEG;
    }
    else if (PACK_ANGLE_RIGHT == p_wire->angle_idx)
    {
        obs.center_angle_deg = PACK_ANGLE_SIDE_DEG;
    }
    else
    {
        obs.center_angle_deg = 0.0f;
    }

    *p_obs = obs;
}

/* ------------------------------------------------------------------ */
/* evt_q_terrain                                                       */
/* ------------------------------------------------------------------ */

void
pack_terrain (bool b_shock, float height_m, terrain_packed_t * p_wire)
{
    p_wire->b_shock   = b_shock ? 1U : 0U;
    p_wire->height_mm = pack_unsigned(height_m, PACK_DISTANCE_SCALE,
                                      TERRAIN_HEIGHT_MAX_MM);
}

void
unpack_terrain (terrain_packed_t const * p_wire, terrain_event_t * p_terrain)
{
    p_terrain->b_shock  = (0U != p_wire->b_shock);
    p_terrain->height_m = (float) p_wire->height_mm / PACK_DISTANCE_SCALE;
}

/*** end of file ***/
