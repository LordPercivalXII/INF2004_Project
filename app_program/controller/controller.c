/** @file controller.c
 *
 * @brief Vehicle controller task: mission state machine, line following,
 *        obstacle avoidance manoeuvre and telemetry snapshots.
 *
 *   FreeRTOS                                micro T-Kernel
 *   --------------------------------------  ----------------------------
 *   xQueueReceive(event_q, .., 2 ticks)     tk_rcv_mbf(.., 10 ms)
 *   xQueueReceive(cmd_q, .., 0)             tk_rcv_mbf(.., TMO_POL)
 *   xEventGroupSetBits / ClearBits          tk_set_flg / tk_clr_flg
 *   xTaskGetTickCount() deadlines           board_millis() deadlines
 */

#include "controller.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../common/rmath.h"
#include "../common/diag_led.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem4/imu_height.h"
#include "../subsystem4/imu_processing.h"

#define CTRL_STEER_GAIN         (0.05f)
#define CTRL_STEER_LIMIT        (0.08f)
#define CTRL_STEER_SMOOTH       (0.50f) /* low-pass on the error: less jerk */
#define CTRL_STEER_KD           (0.0005f) /* per (error/s): damps zig-zag */
#define CTRL_LINE_DT_S          ((float) ROBOT_LINE_PERIOD_MS / 1000.0f)
/* Ramp handling (S4 pitch: negative = nose up, positive = nose down). */
#define CTRL_RAMP_ENTER_DEG     (6.0f)
#define CTRL_RAMP_EXIT_DEG      (3.0f)
#define CTRL_RAMP_PITCH_SMOOTH  (0.15f)
#define CTRL_RAMP_CLIMB_FACTOR  (1.6f)  /* extra drive to hold speed uphill */
#define CTRL_RAMP_DESCEND_FACTOR (0.6f) /* brake downhill */
#define CTRL_RAMP_SPEED_SLEW    (0.02f) /* factor change per line cycle */
#define CTRL_SHOCK_HOLD_MS      (800U)
#define CTRL_START_DELAY_MS     (3000U)
#define CTRL_BARCODE_SLOW_SIDE  (0.55f)
#define CTRL_SHOCK_FACTOR       (0.7f)

#define FAULT_EXTERNAL          (1U)
#define FAULT_IMU               (2U)
#define FAULT_REACQUIRE         (3U)
#define FAULT_BOXED_IN          (4U)
#define FAULT_MOTION_COMMAND    (6U)

ID gh_event_mbf   = E_ID;
ID gh_control_mbf = E_ID;
ID gh_system_flg  = E_ID;

typedef struct
{
    mission_state_t     state;
    float               base_speed;
    float               pitch_deg;          /* smoothed */
    float               ramp_factor;        /* speed multiplier */
    int8_t              ramp_state;         /* -1 down, 0 flat, 1 up */
    uint32_t            shock_until_ms;
    float               line_error;
    float               distance_m;
    float               heading_deg;    /* from the encoders, + = left */
    float               hump_m;
    float               obstacle_cm;
    uint32_t            barcode;
    uint32_t            fault_code;
    bool                b_imu_healthy;
    imu_event_t         imu;
    imu_height_t        height;
    uint32_t            imu_ms;
    bool                b_have_imu;
    bool                b_turn_in_progress;
    uint16_t            pending_turn;
    uint32_t            next_turn_id;
    uint32_t            active_turn_id;
    bool                b_line_detected;
    float               prev_line_error;    /* for the D term           */
    bool                b_steer_primed;     /* prev_line_error is valid */
    avoid_t             avoid;
    avoid_phase_t       avoid_phase_shown;  /* last phase printed */
    uint32_t            last_telemetry_ms;
    bool                b_start_pending;
    uint32_t            start_requested_ms;
    uint32_t            height_trace_sequence;
    uint32_t            height_trace_dropped;
    uint32_t            height_trace_last_drop_ms;
    uint32_t            height_trace_last_move_ms;
    uint32_t            height_trace_motion_ms;
    bool                b_height_trace_moving;
    bool                b_height_trace_have_motion;
} ctrl_state_t;

/* ------------------------------------------------------------------ */
/* Kernel objects                                                      */
/* ------------------------------------------------------------------ */

/*!
 * @brief Create the controller's message buffers and event flag.
 */
ER
controller_init (void)
{
    T_CMBF  cmbf = {0};
    T_CFLG  cflg = {0};
    ER      er   = E_OK;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.maxmsz = (INT) sizeof(system_event_t);
    cmbf.bufsz  = (SZ) APP_MBF_BUFSZ(ROBOT_EVENT_QUEUE_LENGTH,
                                     sizeof(system_event_t));
    gh_event_mbf = tk_cre_mbf(&cmbf);

    cmbf.maxmsz = (INT) sizeof(control_command_t);
    cmbf.bufsz  = (SZ) APP_MBF_BUFSZ(ROBOT_QUEUE_LENGTH,
                                     sizeof(control_command_t));
    gh_control_mbf = tk_cre_mbf(&cmbf);

    cflg.flgatr  = (TA_TFIFO | TA_WMUL);
    cflg.iflgptn = 0U;
    gh_system_flg = tk_cre_flg(&cflg);

    if (gh_event_mbf < E_OK)
    {
        er = gh_event_mbf;
    }
    else if (gh_control_mbf < E_OK)
    {
        er = gh_control_mbf;
    }
    else if (gh_system_flg < E_OK)
    {
        er = gh_system_flg;
    }
    else
    {
        er = E_OK;
    }

    return (er);
}

/*!
 * @brief Start the controller task.
 */
ER
controller_start (void)
{
    ID const tid = app_start_task((FP) controller_task, CTRL_PRI_MAIN,
                                  APP_CORE_ANY, "controller_task");

    return ((tid < E_OK) ? tid : E_OK);
}

/*!
 * @brief Post a system event to the controller without waiting.
 *        Callable from tasks and (TMO_POL) interrupt handlers.
 *
 * @return true if queued; false if the buffer is full or p_event is NULL.
 */
bool
controller_post_event (system_event_t const * p_event)
{
    bool b_ok = false;

    if ((NULL != p_event) && (gh_event_mbf > 0))
    {
        b_ok = (E_OK == tk_snd_mbf(gh_event_mbf, (void *) p_event,
                                   (INT) sizeof(*p_event), TMO_POL));
    }

    return (b_ok);
}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static void
ctrl_set_bits (uint32_t bits)
{
    (void) tk_set_flg(gh_system_flg, (UINT) bits);
}

static void
ctrl_clear_bits (uint32_t bits)
{
    (void) tk_clr_flg(gh_system_flg, (UINT) ~bits);
}

static void
ctrl_set_motion (float left_speed, float right_speed)
{
    motion_command_t command = {0};

    command.type        = MOTION_DRIVE;
    command.left_speed  = left_speed;
    command.right_speed = right_speed;
    (void) motion_send_command(&command);
}

/*!
 * @brief Heading since power-up from the wheel encoder counts, + = left
 *        (as MOTION_TURN).  Counts, not speeds, so no error builds up
 *        from a dropped motion event.
 */
static float
ctrl_heading_deg (motion_event_t const * p_motion)
{
    float const m_per_tick = (RMATH_PI * MOTOR_WHEEL_DIAMETER_M) /
                             MOTOR_ENCODER_TICKS_PER_REV;

    return (((float) (p_motion->right_ticks - p_motion->left_ticks) *
             m_per_tick / MOTOR_TRACK_WIDTH_M) * RMATH_RAD_TO_DEG);
}

static bool
ctrl_turn_motion (float angle_deg, uint32_t turn_id)
{
    motion_command_t command = {0};

    command.type           = MOTION_TURN;
    command.left_speed     = 0.0f;
    command.right_speed    = 0.0f;
    command.turn_angle_deg = angle_deg;
    command.turn_id        = turn_id;

    return (motion_send_command(&command));
}

static void
ctrl_stop_motion (void)
{
    motion_command_t command = {0};

    command.type        = MOTION_STOP;
    command.left_speed  = 0.0f;
    command.right_speed = 0.0f;
    (void) motion_send_command(&command);
}

static void
ctrl_enter_fault (ctrl_state_t * p_ctrl, uint32_t code)
{
    p_ctrl->fault_code = code;
    p_ctrl->b_start_pending = false;
    p_ctrl->start_requested_ms = 0U;
    p_ctrl->state      = MISSION_FAULT;
    p_ctrl->b_turn_in_progress = false;
    p_ctrl->active_turn_id = 0U;
    p_ctrl->pending_turn = (uint16_t) BARCODE_CMD_NONE;
    ctrl_set_bits(SYSTEM_BIT_FAULT);
    ctrl_clear_bits(SYSTEM_BIT_MISSION_RUN | SYSTEM_BIT_OBSTACLE);
    ctrl_stop_motion();
}

/* ------------------------------------------------------------------ */
/* User commands (IF-02)                                               */
/* ------------------------------------------------------------------ */

static void
ctrl_handle_command (ctrl_state_t * p_ctrl,
                     control_command_t const * p_command)
{
    switch (p_command->type)
    {
        case CONTROL_START:
            if ((MISSION_IDLE == p_ctrl->state) &&
                (!p_ctrl->b_start_pending) && p_ctrl->b_imu_healthy)
            {
                p_ctrl->b_start_pending = true;
                p_ctrl->start_requested_ms = board_millis();
                ctrl_clear_bits(SYSTEM_BIT_MISSION_RUN);
                ctrl_stop_motion();
                tm_printf((UB *) "CTRL: START received, waiting 3 seconds\n");
            }
            else
            {
                tm_printf((UB *) "CTRL: START ignored: not idle, "
                          "already pending, or IMU not ready\n");
            }
            break;

        case CONTROL_STOP:
            p_ctrl->b_start_pending = false;
            p_ctrl->start_requested_ms = 0U;
            p_ctrl->state = MISSION_IDLE;
            p_ctrl->b_turn_in_progress = false;
            p_ctrl->active_turn_id = 0U;
            p_ctrl->pending_turn = (uint16_t) BARCODE_CMD_NONE;
            ctrl_clear_bits(SYSTEM_BIT_MISSION_RUN | SYSTEM_BIT_OBSTACLE);
            ctrl_stop_motion();
            tm_printf((UB *) "CTRL: STOP received, mission idle\n");
            break;

        case CONTROL_SET_SPEED:
            p_ctrl->base_speed = rmath_clampf(p_command->value,
                                              ROBOT_MIN_SPEED,
                                              ROBOT_MAX_SPEED);
            break;

        case CONTROL_CLEAR_FAULT:
            if (p_ctrl->b_imu_healthy)
            {
                p_ctrl->fault_code = 0U;
                p_ctrl->b_start_pending = false;
                p_ctrl->start_requested_ms = 0U;
                p_ctrl->state      = MISSION_IDLE;
                ctrl_clear_bits(SYSTEM_BIT_FAULT | SYSTEM_BIT_MISSION_RUN);
            }
            break;

        default:
            /* Unknown command: ignore. */
            break;
    }
}

/* ------------------------------------------------------------------ */
/* System events (IF-01)                                               */
/* ------------------------------------------------------------------ */

static float
ctrl_drive_speed (ctrl_state_t * p_ctrl)
{
    float target = 1.0f;
    float speed;

    if (p_ctrl->ramp_state > 0)
    {
        target = CTRL_RAMP_CLIMB_FACTOR;
    }
    else if (p_ctrl->ramp_state < 0)
    {
        target = CTRL_RAMP_DESCEND_FACTOR;
    }

    /* Slew so the speed changes smoothly instead of jerking. */
    p_ctrl->ramp_factor += rmath_clampf(target - p_ctrl->ramp_factor,
                                        -CTRL_RAMP_SPEED_SLEW,
                                        CTRL_RAMP_SPEED_SLEW);
    speed = p_ctrl->base_speed * p_ctrl->ramp_factor;
    if ((0 == p_ctrl->ramp_state) &&
        (board_millis() < p_ctrl->shock_until_ms))
    {
        speed = rmath_minf(speed, ROBOT_BASE_SPEED * CTRL_SHOCK_FACTOR);
    }

    return rmath_clampf(speed, ROBOT_MIN_SPEED, ROBOT_MAX_SPEED);
}

static void
ctrl_handle_line (ctrl_state_t * p_ctrl, line_event_t const * p_line)
{
    p_ctrl->line_error      = p_line->line_error;
    p_ctrl->b_line_detected = p_line->b_line_detected;

    if (p_line->b_barcode_valid)
    {
        /* Decoded character (ASCII), for telemetry.  It is only an
           instruction once S3 resolves it at the next junction. */
        p_ctrl->barcode = p_line->barcode;
    }

    if (p_line->b_junction)
    {
        /* S3 looked the stored character up in the MQTT route table;
           BARCODE_CMD_NONE means carry straight on. */
        p_ctrl->pending_turn =
            (p_line->junction_action <= (uint16_t) BARCODE_CMD_UTURN)
            ? p_line->junction_action
            : (uint16_t) BARCODE_CMD_NONE;
    }

    if ((MISSION_FOLLOWING == p_ctrl->state) && p_line->b_junction &&
        (!p_ctrl->b_turn_in_progress) &&
        ((uint16_t) BARCODE_CMD_NONE != p_ctrl->pending_turn))
    {
        float angle_deg = 0.0f;

        if ((uint16_t) BARCODE_CMD_LEFT == p_ctrl->pending_turn)
        {
            angle_deg = 90.0f;
        }
        else if ((uint16_t) BARCODE_CMD_RIGHT == p_ctrl->pending_turn)
        {
            angle_deg = -90.0f;
        }
        else if ((uint16_t) BARCODE_CMD_UTURN == p_ctrl->pending_turn)
        {
            angle_deg = 180.0f;
        }
        else
        {
            /* Unknown commands are discarded below. */
        }

        p_ctrl->next_turn_id++;
        if (0U == p_ctrl->next_turn_id)
        {
            p_ctrl->next_turn_id++;
        }

        if ((0.0f != angle_deg) &&
            ctrl_turn_motion(angle_deg, p_ctrl->next_turn_id))
        {
            p_ctrl->active_turn_id = p_ctrl->next_turn_id;
            p_ctrl->b_turn_in_progress = true;
            p_ctrl->pending_turn = (uint16_t) BARCODE_CMD_NONE;
        }
        else
        {
            p_ctrl->pending_turn = (uint16_t) BARCODE_CMD_NONE;
            ctrl_enter_fault(p_ctrl, FAULT_MOTION_COMMAND);
        }
    }

    if (((MISSION_FOLLOWING == p_ctrl->state) &&
         (!p_ctrl->b_turn_in_progress)) ||
        ((MISSION_AVOIDING == p_ctrl->state) &&
         avoid_is_following(&p_ctrl->avoid)))
    {
        /* PD steering.  With the line sensors on the line edges the
           error is continuous near the centre; the D term damps the
           zig-zag.  No D kick on the first cycle after a turn. */
        static float smooth_error = 0.0f;
        smooth_error = p_ctrl->b_steer_primed
            ? (smooth_error +
               (CTRL_STEER_SMOOTH * (p_ctrl->line_error - smooth_error)))
            : p_ctrl->line_error;
        p_ctrl->line_error = smooth_error;
        float const derivative = p_ctrl->b_steer_primed
            ? ((p_ctrl->line_error - p_ctrl->prev_line_error) /
               CTRL_LINE_DT_S)
            : 0.0f;
        float const correction =
            rmath_clampf((p_ctrl->line_error * CTRL_STEER_GAIN) +
                         (derivative * CTRL_STEER_KD),
                         -CTRL_STEER_LIMIT, CTRL_STEER_LIMIT);

        p_ctrl->prev_line_error = p_ctrl->line_error;
        p_ctrl->b_steer_primed  = true;
        float const speed = ctrl_drive_speed(p_ctrl);

        ctrl_set_motion(speed + correction, speed - correction);
    }
    else
    {
        /* Not steering in this state. */
        p_ctrl->b_steer_primed = false;
    }
}

static void
ctrl_handle_imu (ctrl_state_t * p_ctrl, imu_event_t const * p_imu)
{
    bool const b_new_shock = (p_imu->shock_count != p_ctrl->imu.shock_count);
    bool const b_bad_timing = p_imu->b_timing_gap &&
        ((p_imu->sample_interval_ms < 5U) ||
         (p_imu->sample_interval_ms > IMU_RECOVERABLE_INTERVAL_MS));
    p_ctrl->imu = *p_imu;
    if (p_imu->b_sensor_valid)
    {
        p_ctrl->b_imu_healthy = p_imu->b_calibrated &&
            (!p_imu->b_saturated) && !b_bad_timing;
        p_ctrl->hump_m        = p_imu->estimated_hump_height_m;

        p_ctrl->pitch_deg += CTRL_RAMP_PITCH_SMOOTH *
            (p_imu->tilt_pitch_deg - p_ctrl->pitch_deg);
        int8_t new_ramp = p_ctrl->ramp_state;

        if (p_ctrl->pitch_deg < -CTRL_RAMP_ENTER_DEG)
        {
            new_ramp = 1;
        }
        else if (p_ctrl->pitch_deg > CTRL_RAMP_ENTER_DEG)
        {
            new_ramp = -1;
        }
        else if (rmath_absf(p_ctrl->pitch_deg) < CTRL_RAMP_EXIT_DEG)
        {
            new_ramp = 0;
        }
        else
        {
            /* Between exit and enter thresholds: keep the current state. */
        }

        if (new_ramp != p_ctrl->ramp_state)
        {
            p_ctrl->ramp_state = new_ramp;
            tm_printf((UB *) "[CTRL] RAMP %s pitch=%d.%d deg\n",
                      (new_ramp > 0) ? "UP (climb boost)" :
                      ((new_ramp < 0) ? "DOWN (slowing)" : "FLAT"),
                      (int) p_ctrl->pitch_deg,
                      (int) rmath_absf((p_ctrl->pitch_deg -
                                        (float) (int) p_ctrl->pitch_deg) * 10.0f));
        }

        if (p_imu->b_hump_height_valid || p_imu->estimated_hump_height_m > 0.001f)
        {
            tm_printf((UB *) "[S4] height: hump=%d.%03d m, rise=%d.%03d m, peak=%d.%03d m, valid=%u active=%u\n",
                      (int) p_imu->estimated_hump_height_m,
                      (int) ((p_imu->estimated_hump_height_m * 1000.0f) - (int) p_imu->estimated_hump_height_m * 1000),
                      (int) p_imu->terrain_rise_m,
                      (int) ((p_imu->terrain_rise_m * 1000.0f) - (int) p_imu->terrain_rise_m * 1000),
                      (int) p_imu->terrain_peak_m,
                      (int) ((p_imu->terrain_peak_m * 1000.0f) - (int) p_imu->terrain_peak_m * 1000),
                      (unsigned int) p_imu->b_hump_height_valid,
                      (unsigned int) p_imu->b_hump_active);
        }

        /* A transient IMU timing gap (e.g. motor noise or a blocked I2C
         * read) is not fatal; only an uncalibrated IMU stops the car. */
        if ((!p_imu->b_calibrated) &&
            ((MISSION_FOLLOWING == p_ctrl->state) ||
             (MISSION_AVOIDING == p_ctrl->state)))
        {
            ctrl_enter_fault(p_ctrl, FAULT_IMU);
        }
        else if (p_imu->b_calibrated &&
                 (p_imu->b_shock_detected || b_new_shock))
        {
            p_ctrl->shock_until_ms = board_millis() + CTRL_SHOCK_HOLD_MS;
        }
    }
    else
    {
        p_ctrl->b_imu_healthy = false;
        ctrl_enter_fault(p_ctrl, FAULT_IMU);
    }
}

static void
ctrl_apply_height (ctrl_state_t * p_ctrl)
{
    imu_height_apply(&p_ctrl->height, &p_ctrl->imu);
    p_ctrl->hump_m = p_ctrl->imu.estimated_hump_height_m;
}

static void
ctrl_report_height_rejection (ctrl_state_t const * p_ctrl, bool b_active,
                              uint32_t motion_ms, uint32_t interval_ms)
{
    tm_printf((UB *) "S4: encoder height rejected reason=%s active=%u "
              "imu_ms=%u motion_ms=%u interval_ms=%u "
              "sensor=%u cal=%u tilt=%u saturated=%u shock=%u\n",
              imu_height_rejection_name(p_ctrl->height.last_rejection),
              (unsigned int) b_active,
              (unsigned int) p_ctrl->imu_ms,
              (unsigned int) motion_ms,
              (unsigned int) interval_ms,
              (unsigned int) p_ctrl->imu.b_sensor_valid,
              (unsigned int) p_ctrl->imu.b_calibrated,
              (unsigned int) p_ctrl->imu.b_tilt_valid,
              (unsigned int) p_ctrl->imu.b_saturated,
              (unsigned int) p_ctrl->imu.b_shock_detected);
    if (HEIGHT_REJECT_GAP_ACCELERATION == p_ctrl->height.last_rejection)
    {
        float const ax = p_ctrl->imu.acceleration_x_g;
        float const ay = p_ctrl->imu.acceleration_y_g;
        float const az = p_ctrl->imu.acceleration_z_g;
        if ((ax >= -8.0f && ax <= 8.0f) &&
            (ay >= -8.0f && ay <= 8.0f) &&
            (az >= -8.0f && az <= 8.0f))
        {
            tm_printf((UB *) "S4: gap acceleration ax_mg=%d ay_mg=%d "
                      "az_mg=%d allowed_norm_mg=650..1350\n",
                      (int) (ax * 1000.0f), (int) (ay * 1000.0f),
                      (int) (az * 1000.0f));
        }
        else
        {
            tm_printf((UB *) "S4: gap acceleration nonfinite or out of range\n");
        }
    }
}

static void
ctrl_report_height_wheels (motion_event_t const * p_motion)
{
    char left[S1_FIXED_TEXT_MAX];
    char right[S1_FIXED_TEXT_MAX];
    bool const b_left_finite = (p_motion->left_speed_m_s >= -1000000.0f &&
                                p_motion->left_speed_m_s <= 1000000.0f);
    bool const b_right_finite = (p_motion->right_speed_m_s >= -1000000.0f &&
                                 p_motion->right_speed_m_s <= 1000000.0f);
    if (b_left_finite)
        telemetry_format_fixed(left, p_motion->left_speed_m_s, 3U);
    if (b_right_finite)
        telemetry_format_fixed(right, p_motion->right_speed_m_s, 3U);
    tm_printf((UB *) "S4: rejected wheel data left_m_s=%s right_m_s=%s "
              "left_ticks=%d right_ticks=%d\n",
              b_left_finite ? left : "nonfinite/out_of_range",
              b_right_finite ? right : "nonfinite/out_of_range",
              (int) p_motion->left_ticks, (int) p_motion->right_ticks);
}

static void
ctrl_handle_obstacle (ctrl_state_t * p_ctrl,
                      obstacle_event_t const * p_obs)
{
    if (!p_obs->b_side_reading)
    {
        p_ctrl->obstacle_cm = p_obs->center_distance_cm;
    }

    if (p_obs->b_side_reading)
    {
        /* The sensor is held toward the obstacle's side, not ahead. */
        if (MISSION_AVOIDING == p_ctrl->state)
        {
            avoid_on_side_reading(&p_ctrl->avoid, p_obs->side_distance_cm);
        }
    }
    else if ((MISSION_FOLLOWING == p_ctrl->state) &&
             (OBSTACLE_ACTION_CONTINUE != p_obs->action))
    {
        obstacle_set_look(AVOID_LOOK_SCAN);
        ctrl_set_bits(SYSTEM_BIT_OBSTACLE);
        p_ctrl->state = MISSION_AVOIDING;
        p_ctrl->b_turn_in_progress = false;
        p_ctrl->active_turn_id = 0U;
        p_ctrl->pending_turn = (uint16_t) BARCODE_CMD_NONE;
        avoid_start(&p_ctrl->avoid, board_millis(), p_ctrl->distance_m);
        p_ctrl->avoid_phase_shown = AVOID_PHASE_IDLE;
    }
    else if (MISSION_AVOIDING == p_ctrl->state)
    {
        if ((!p_obs->b_scan_complete) &&
            (OBSTACLE_ACTION_CONTINUE == p_obs->action))
        {
            /* One straight-ahead reading while going round. */
            avoid_on_ahead_reading(&p_ctrl->avoid,
                                   p_obs->center_distance_cm);
        }

        avoid_on_scan(&p_ctrl->avoid, p_obs);
    }
    else
    {
        /* Nothing to do. */
    }
}

#if IMU_HEIGHT_TRACE_ENABLED
static void
ctrl_publish_height_trace (ctrl_state_t * p_ctrl,
                           system_event_t const * p_event,
                           float ds)
{
    telemetry_message_t trace = {0};
    uint32_t const now_ms = p_event->timestamp_ms;
    uint32_t const interval_ms = p_ctrl->b_height_trace_have_motion
        ? now_ms - p_ctrl->height_trace_motion_ms : 0U;
    p_ctrl->height_trace_motion_ms = now_ms;
    p_ctrl->b_height_trace_have_motion = true;
    if ((rmath_absf(ds) > 0.000001f) || p_ctrl->height.b_active ||
        p_ctrl->height.b_start_pending)
    {
        p_ctrl->height_trace_last_move_ms = now_ms;
        p_ctrl->b_height_trace_moving = true;
    }
    if (!p_ctrl->b_height_trace_moving) return;
    if ((now_ms - p_ctrl->height_trace_last_move_ms) > 500U)
    {
        p_ctrl->b_height_trace_moving = false;
        return;
    }
    trace.b_height_trace = true;
    trace.timestamp_ms = now_ms;
    trace.imu_ms = p_ctrl->imu_ms;
    trace.imu = p_ctrl->imu;
    trace.motion = p_event->data.motion;
    trace.motion_interval_ms = interval_ms;
    trace.motion_delta_m = ds;
    trace.trace_sequence = ++p_ctrl->height_trace_sequence;
    trace.trace_dropped = p_ctrl->height_trace_dropped;
    if (!telemetry_publish(&trace))
    {
        p_ctrl->height_trace_dropped++;
        if ((1U == p_ctrl->height_trace_dropped) ||
            ((now_ms - p_ctrl->height_trace_last_drop_ms) >= 1000U))
        {
            tm_printf((UB *) "S4: height trace queue full dropped_total=%u\n",
                      (unsigned int) p_ctrl->height_trace_dropped);
            p_ctrl->height_trace_last_drop_ms = now_ms;
        }
    }
}
#endif

static void
ctrl_handle_event (ctrl_state_t * p_ctrl, system_event_t const * p_event)
{
    switch (p_event->type)
    {
        case SYSTEM_EVENT_LINE:
            ctrl_handle_line(p_ctrl, &p_event->data.line);
            break;

        case SYSTEM_EVENT_MOTION:
        {
            bool const b_was_valid = p_ctrl->height.b_valid;
            bool const b_was_active = p_ctrl->height.b_active;
            bool const b_was_gap = p_ctrl->height.b_gap;
            uint32_t const gap_count = p_ctrl->height.gap_count;
            uint32_t const interval_ms =
                p_event->timestamp_ms - p_ctrl->height.last_ms;
            float const ds = p_event->data.motion.distance_m - p_ctrl->distance_m;
            p_ctrl->distance_m = p_event->data.motion.distance_m;
            p_ctrl->heading_deg = ctrl_heading_deg(&p_event->data.motion);
            if (p_ctrl->b_have_imu)
            {
                imu_height_update(&p_ctrl->height, &p_event->data.motion,
                                  &p_ctrl->imu, p_ctrl->imu_ms,
                                  p_event->timestamp_ms);
                if (((b_was_valid || b_was_gap) && !p_ctrl->height.b_valid &&
                     !p_ctrl->height.b_gap) ||
                    p_ctrl->height.b_rejected)
                {
                    ctrl_report_height_rejection(p_ctrl, b_was_active,
                                                  p_event->timestamp_ms,
                                                  interval_ms);
                    ctrl_report_height_wheels(&p_event->data.motion);
                    tm_printf((UB *) "S4: [diagnostic] rise=%d.%03d m peak=%d.%03d m\n",
                              (int) p_ctrl->height.rise_m,
                              (int) ((p_ctrl->height.rise_m * 1000.0f) - 
                                     (int) p_ctrl->height.rise_m * 1000),
                              (int) p_ctrl->height.peak_m,
                              (int) ((p_ctrl->height.peak_m * 1000.0f) - 
                                     (int) p_ctrl->height.peak_m * 1000));
                }
                if (p_ctrl->height.gap_count != gap_count)
                {
                    tm_printf((UB *) "S4: bounded tilt gap bridged count=%u\n",
                              (unsigned int) p_ctrl->height.gap_count);
                }
                if (p_ctrl->height.b_complete)
                {
                    tm_printf((UB *) "S4: encoder hump complete peak_mm=%d\n",
                              (int) (p_ctrl->height.last_peak_m * 1000.0f));
                }
                ctrl_apply_height(p_ctrl);
#if IMU_HEIGHT_TRACE_ENABLED
                ctrl_publish_height_trace(p_ctrl, p_event, ds);
#else
                (void) ds;
#endif
            }
            if (p_ctrl->b_turn_in_progress &&
                p_event->data.motion.turn_complete &&
                (p_event->data.motion.turn_id == p_ctrl->active_turn_id))
            {
                p_ctrl->b_turn_in_progress = false;
                p_ctrl->active_turn_id = 0U;
            }
            break;
        }

        case SYSTEM_EVENT_IMU:
        {
            imu_height_rejection_t reason;
            bool const b_was_valid = p_ctrl->height.b_valid;
            bool const b_was_active = p_ctrl->height.b_active;
            bool const b_was_gap = p_ctrl->height.b_gap;
            ctrl_handle_imu(p_ctrl, &p_event->data.imu);
            p_ctrl->imu_ms = p_event->timestamp_ms;
            p_ctrl->b_have_imu = true;
            imu_height_observe_imu(&p_ctrl->height, &p_ctrl->imu,
                                   p_ctrl->imu_ms);
            reason = HEIGHT_REJECT_NONE;
            if (p_ctrl->height.b_have_distance &&
                 ((p_ctrl->imu_ms - p_ctrl->height.last_ms) > 100U) &&
                 ((p_ctrl->height.last_ms - p_ctrl->imu_ms) > 60U))
            {
                reason = HEIGHT_REJECT_WHEEL_AGE;
            }
            if (HEIGHT_REJECT_NONE != reason)
            {
                imu_height_invalidate(&p_ctrl->height, reason);
            }
            if ((b_was_valid || b_was_active || b_was_gap) &&
                !p_ctrl->height.b_gap && !p_ctrl->height.b_valid)
            {
                ctrl_report_height_rejection(p_ctrl, b_was_active,
                                              p_ctrl->height.last_ms,
                                              p_ctrl->imu_ms -
                                              p_ctrl->height.last_ms);
            }
            ctrl_apply_height(p_ctrl);
            break;
        }

        case SYSTEM_EVENT_OBSTACLE:
            ctrl_handle_obstacle(p_ctrl, &p_event->data.obstacle);
            break;

        case SYSTEM_EVENT_FAULT:
            ctrl_enter_fault(p_ctrl, FAULT_EXTERNAL);
            break;

        default:
            /* Unknown event: ignore. */
            break;
    }
}

/* ------------------------------------------------------------------ */
/* Avoidance manoeuvre (subsystem5/avoidance.c) and telemetry          */
/* ------------------------------------------------------------------ */

/*!
 * @brief Print each new step of the manoeuvre, the obstacle's measured
 *        thickness and how it ended, so a test log shows where it went.
 */
static void
ctrl_show_avoidance (ctrl_state_t * p_ctrl, avoid_status_t status)
{
    static char const * const results[] =
    {
        "running", "done", "boxed in", "line not found", "timed out",
        "blocked ahead"
    };
    avoid_t const * const p_avoid = &p_ctrl->avoid;

    if (p_avoid->phase != p_ctrl->avoid_phase_shown)
    {
        if ((AVOID_PHASE_TURN_IN == p_avoid->phase) &&
            (AVOID_PHASE_LEG_PAST == p_ctrl->avoid_phase_shown))
        {
            if (p_avoid->b_rear_found)
            {
                tm_printf((UB *) "S5: obstacle thickness %d cm\n",
                          (int) (p_avoid->thickness_cm + 0.5f));
            }
            else
            {
                tm_printf((UB *) "S5: obstacle thickness not measured "
                          "(side %s)\n",
                          p_avoid->b_side_seen ? "never cleared"
                                               : "not seen");
            }
        }

        tm_printf((UB *) "S5: avoid %s heading %d along %d out %d cm\n",
                  avoid_phase_name(p_avoid->phase),
                  (int) (p_ctrl->heading_deg - p_avoid->heading0_deg),
                  (int) p_avoid->along_cm, (int) p_avoid->lateral_cm);
        p_ctrl->avoid_phase_shown = p_avoid->phase;
    }

    if ((AVOID_RUNNING != status) &&
        ((uint32_t) status < (sizeof(results) / sizeof(results[0]))))
    {
        tm_printf((UB *) "S5: avoid %s\n", results[status]);
    }
}

static void
ctrl_step_avoidance (ctrl_state_t * p_ctrl, uint32_t now_ms)
{
    avoid_input_t  input;
    avoid_output_t output;
    avoid_status_t status;

    if (MISSION_AVOIDING == p_ctrl->state)
    {
        input.now_ms          = now_ms;
        input.distance_m      = p_ctrl->distance_m;
        input.heading_deg     = p_ctrl->heading_deg;
        input.b_line_detected = p_ctrl->b_line_detected;
        input.b_turn_done     = !p_ctrl->b_turn_in_progress;

        status = avoid_step(&p_ctrl->avoid, &input, &output);
        obstacle_set_look(avoid_look(&p_ctrl->avoid));
        ctrl_show_avoidance(p_ctrl, status);

        if ((AVOID_OUT_STOP == output.type) ||
            (AVOID_OUT_DRIVE == output.type))
        {
            /* Replaces any turn still running: it will never complete. */
            p_ctrl->b_turn_in_progress = false;
            p_ctrl->active_turn_id     = 0U;
        }

        if (AVOID_OUT_STOP == output.type)
        {
            ctrl_stop_motion();
        }
        else if (AVOID_OUT_DRIVE == output.type)
        {
            ctrl_set_motion(output.left_speed, output.right_speed);
        }
        else if (AVOID_OUT_TURN == output.type)
        {
            p_ctrl->next_turn_id++;
            if (0U == p_ctrl->next_turn_id)
            {
                p_ctrl->next_turn_id++;
            }

            if (ctrl_turn_motion(output.turn_deg, p_ctrl->next_turn_id))
            {
                p_ctrl->active_turn_id = p_ctrl->next_turn_id;
                p_ctrl->b_turn_in_progress = true;
            }
            else
            {
                ctrl_enter_fault(p_ctrl, FAULT_MOTION_COMMAND);
            }
        }
        else
        {
            /* Keep the last command. */
        }

        if (MISSION_AVOIDING != p_ctrl->state)
        {
            /* The turn command just failed. */
        }
        else if (AVOID_DONE == status)
        {
            p_ctrl->state = MISSION_FOLLOWING;
            obstacle_set_look(AVOID_LOOK_SCAN);
            ctrl_clear_bits(SYSTEM_BIT_OBSTACLE);
        }
        else if ((AVOID_FAILED_BOXED_IN == status) ||
                 (AVOID_FAILED_BLOCKED == status))
        {
            ctrl_enter_fault(p_ctrl, FAULT_BOXED_IN);
        }
        else if (AVOID_RUNNING != status)
        {
            ctrl_enter_fault(p_ctrl, FAULT_REACQUIRE);
        }
        else
        {
            /* Manoeuvre still running. */
        }
    }
}

static void
ctrl_publish_telemetry (ctrl_state_t * p_ctrl, uint32_t now_ms)
{
    telemetry_message_t telemetry = {0};

    if ((now_ms - p_ctrl->last_telemetry_ms) >= ROBOT_TELEMETRY_PERIOD_MS)
    {
        telemetry.timestamp_ms         = now_ms;
        telemetry.mission_state        = p_ctrl->state;
        telemetry.speed_m_s            = p_ctrl->base_speed * p_ctrl->ramp_factor;
        telemetry.line_error           = p_ctrl->line_error;
        telemetry.distance_m           = p_ctrl->distance_m;
        telemetry.hump_height_m        = p_ctrl->hump_m;
        telemetry.obstacle_distance_cm = p_ctrl->obstacle_cm;
        telemetry.last_barcode         = p_ctrl->barcode;
        telemetry.fault_code           = p_ctrl->fault_code;
        telemetry.imu                  = p_ctrl->imu;
        if (!telemetry_publish(&telemetry))
        {
            tm_printf((UB *) "S1: telemetry snapshot queue full t_ms=%u\n",
                      (unsigned int) now_ms);
        }

        if (p_ctrl->height.b_active || p_ctrl->height.b_valid || 
            p_ctrl->height.b_complete)
        {
            tm_printf((UB *) "[S4] height: rise=%d.%03d m peak=%d.%03d m "
                      "valid=%u active=%u complete=%u gap=%u\n",
                      (int) p_ctrl->height.rise_m,
                      (int) ((p_ctrl->height.rise_m * 1000.0f) - 
                             (int) p_ctrl->height.rise_m * 1000),
                      (int) p_ctrl->height.peak_m,
                      (int) ((p_ctrl->height.peak_m * 1000.0f) - 
                             (int) p_ctrl->height.peak_m * 1000),
                      (unsigned int) p_ctrl->height.b_valid,
                      (unsigned int) p_ctrl->height.b_active,
                      (unsigned int) p_ctrl->height.b_complete,
                      (unsigned int) p_ctrl->height.gap_count);
        }

        p_ctrl->last_telemetry_ms = now_ms;
    }
}

/* ------------------------------------------------------------------ */
/* Task                                                                */
/* ------------------------------------------------------------------ */

/*!
 * @brief Controller task: commands, events, manoeuvre, telemetry.
 */
void
controller_task (INT stacd, void * p_exinf)
{
    static ctrl_state_t ctrl;
    control_command_t   command;
    system_event_t      event;
    INT                 msg_size;

    (void) stacd;
    (void) p_exinf;

    ctrl.state             = MISSION_IDLE;
    ctrl.b_start_pending   = false;
    ctrl.start_requested_ms = 0U;
    ctrl.base_speed        = ROBOT_BASE_SPEED;
    ctrl.ramp_factor       = 1.0f;
    ctrl.b_turn_in_progress = false;
    ctrl.pending_turn       = (uint16_t) BARCODE_CMD_NONE;
    ctrl.next_turn_id       = 0U;
    ctrl.active_turn_id     = 0U;
    ctrl.last_telemetry_ms = board_millis();
    imu_height_init(&ctrl.height);

    /* Diagnostic LED: show boot status */
    diag_led_init();
    diag_led_set_state(DIAG_LED_BOOT);

    /* Bench-test autostart (AUTO_START_MS build option); 0 = dashboard only. */
    ctrl_set_bits(SYSTEM_BIT_READY);
#if (CTRL_AUTO_START_MS > 0U)
    bool b_auto_started = false;
#endif

    for (;;)
    {
#if (CTRL_AUTO_START_MS > 0U)
        if (!b_auto_started && (MISSION_IDLE == ctrl.state) &&
            ctrl.b_imu_healthy && (board_millis() >= CTRL_AUTO_START_MS))
        {
            control_command_t const auto_cmd = { .type = CONTROL_START };

            b_auto_started = true;
            ctrl_handle_command(&ctrl, &auto_cmd);
        }
#endif
        if (tk_rcv_mbf(gh_control_mbf, &command, TMO_POL) > 0)
        {
            ctrl_handle_command(&ctrl, &command);
        }

        /* Update LED diagnostic based on IMU health */
        if (ctrl.b_imu_healthy) {
            diag_led_set_state(DIAG_LED_IMU_HEALTHY);
        } else if (!ctrl.imu.b_sensor_valid) {
            /* I2C/sensor failure: blink error pattern */
            diag_led_set_state(DIAG_LED_I2C_FAIL);
        } else {
            /* Still booting/calibrating */
            diag_led_set_state(DIAG_LED_BOOT);
        }
        diag_led_poll(board_millis());

        /* Nonblocking 3-second START countdown. STOP clears the pending flag. */
        if (ctrl.b_start_pending &&
            ((uint32_t) (board_millis() - ctrl.start_requested_ms) >=
             CTRL_START_DELAY_MS))
        {
            ctrl.b_start_pending = false;
            ctrl.start_requested_ms = 0U;
            if ((MISSION_IDLE == ctrl.state) && ctrl.b_imu_healthy)
            {
                ctrl.state = MISSION_FOLLOWING;
                ctrl_set_bits(SYSTEM_BIT_MISSION_RUN);
                tm_printf((UB *) "CTRL: Countdown complete, mission STARTED\n");
            }
            else
            {
                ctrl_stop_motion();
                tm_printf((UB *) "CTRL: START cancelled, robot not ready\n");
            }
        }

        /* Wait for one event, then drain whatever else is queued. */
        msg_size = tk_rcv_mbf(gh_event_mbf, &event,
                              (TMO) ROBOT_CONTROLLER_WAIT_MS);

        while (msg_size > 0)
        {
            ctrl_handle_event(&ctrl, &event);
            msg_size = tk_rcv_mbf(gh_event_mbf, &event, TMO_POL);
        }

        ctrl_step_avoidance(&ctrl, board_millis());
        ctrl_publish_telemetry(&ctrl, board_millis());
    }
}

/*** end of file ***/
