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
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/avoidance.h"
#include "../subsystem4/imu_height.h"

#define CTRL_STEER_GAIN         (0.18f)
#define CTRL_STEER_LIMIT        (0.25f)
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
    float               line_error;
    float               distance_m;
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
    avoid_t             avoid;
    uint32_t            last_telemetry_ms;
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
            if ((MISSION_FAULT != p_ctrl->state) && p_ctrl->b_imu_healthy)
            {
                p_ctrl->state = MISSION_FOLLOWING;
                ctrl_set_bits(SYSTEM_BIT_MISSION_RUN);
            }
            else
            {
                tm_printf((UB *) "CTRL: START refused: fault or IMU "
                          "not calibrated/healthy\n");
            }
            break;

        case CONTROL_STOP:
            p_ctrl->state = MISSION_IDLE;
            p_ctrl->b_turn_in_progress = false;
            p_ctrl->active_turn_id = 0U;
            p_ctrl->pending_turn = (uint16_t) BARCODE_CMD_NONE;
            ctrl_clear_bits(SYSTEM_BIT_MISSION_RUN | SYSTEM_BIT_OBSTACLE);
            ctrl_stop_motion();
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
        float const correction =
            rmath_clampf(p_ctrl->line_error * CTRL_STEER_GAIN,
                         -CTRL_STEER_LIMIT, CTRL_STEER_LIMIT);

        ctrl_set_motion(p_ctrl->base_speed + correction,
                        p_ctrl->base_speed - correction);
    }
    else
    {
        /* Not steering in this state. */
    }
}

static void
ctrl_handle_imu (ctrl_state_t * p_ctrl, imu_event_t const * p_imu)
{
    bool const b_new_shock = (p_imu->shock_count != p_ctrl->imu.shock_count);
    p_ctrl->imu = *p_imu;
    if (p_imu->b_sensor_valid)
    {
        p_ctrl->b_imu_healthy = p_imu->b_calibrated && (!p_imu->b_saturated);
        p_ctrl->hump_m        = p_imu->estimated_hump_height_m;

        if ((!p_imu->b_calibrated) &&
            ((MISSION_FOLLOWING == p_ctrl->state) ||
             (MISSION_AVOIDING == p_ctrl->state)))
        {
            ctrl_enter_fault(p_ctrl, FAULT_IMU);
        }
        else if (p_imu->b_calibrated &&
                 (p_imu->b_shock_detected || b_new_shock))
        {
            p_ctrl->base_speed = rmath_minf(p_ctrl->base_speed,
                                            ROBOT_BASE_SPEED *
                                            CTRL_SHOCK_FACTOR);
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
    p_ctrl->obstacle_cm = p_obs->center_distance_cm;

    if ((MISSION_FOLLOWING == p_ctrl->state) &&
        (OBSTACLE_ACTION_CONTINUE != p_obs->action))
    {
        ctrl_set_bits(SYSTEM_BIT_OBSTACLE);
        p_ctrl->state = MISSION_AVOIDING;
        p_ctrl->b_turn_in_progress = false;
        p_ctrl->active_turn_id = 0U;
        p_ctrl->pending_turn = (uint16_t) BARCODE_CMD_NONE;
        avoid_start(&p_ctrl->avoid, board_millis(), p_ctrl->distance_m);
    }
    else if (MISSION_AVOIDING == p_ctrl->state)
    {
        avoid_on_scan(&p_ctrl->avoid, p_obs);
    }
    else
    {
        /* Nothing to do. */
    }
}

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
            p_ctrl->distance_m = p_event->data.motion.distance_m;
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
        input.b_line_detected = p_ctrl->b_line_detected;
        input.b_turn_done     = !p_ctrl->b_turn_in_progress;

        status = avoid_step(&p_ctrl->avoid, &input, &output);

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
            ctrl_clear_bits(SYSTEM_BIT_OBSTACLE);
        }
        else if (AVOID_FAILED_BOXED_IN == status)
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
    telemetry_message_t telemetry;

    if ((now_ms - p_ctrl->last_telemetry_ms) >= ROBOT_TELEMETRY_PERIOD_MS)
    {
        telemetry.timestamp_ms         = now_ms;
        telemetry.mission_state        = p_ctrl->state;
        telemetry.speed_m_s            = p_ctrl->base_speed;
        telemetry.line_error           = p_ctrl->line_error;
        telemetry.distance_m           = p_ctrl->distance_m;
        telemetry.hump_height_m        = p_ctrl->hump_m;
        telemetry.obstacle_distance_cm = p_ctrl->obstacle_cm;
        telemetry.last_barcode         = p_ctrl->barcode;
        telemetry.fault_code           = p_ctrl->fault_code;
        telemetry.imu                  = p_ctrl->imu;
        (void) telemetry_publish(&telemetry);
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

#if (!ROBOT_SIMULATION) && (CTRL_AUTO_START_MS > 0)
    bool                b_auto_started = false;
#elif ROBOT_SIMULATION
    bool                b_sim_started = false;
#endif

    ctrl.state             = MISSION_IDLE;
    ctrl.base_speed        = ROBOT_BASE_SPEED;
    ctrl.b_turn_in_progress = false;
    ctrl.pending_turn       = (uint16_t) BARCODE_CMD_NONE;
    ctrl.next_turn_id       = 0U;
    ctrl.active_turn_id     = 0U;
    ctrl.last_telemetry_ms = board_millis();
    imu_height_init(&ctrl.height);

#if (!ROBOT_SIMULATION) && (CTRL_AUTO_START_MS > 0)
#warning "CTRL_AUTO_START_MS is on: the car drives off by itself after power-up"
    tm_printf((UB *) "CTRL: TEST auto-start: following the line in %u ms\n",
              (unsigned int) CTRL_AUTO_START_MS);
#endif
    ctrl_set_bits(SYSTEM_BIT_READY);

    for (;;)
    {
        if (tk_rcv_mbf(gh_control_mbf, &command, TMO_POL) > 0)
        {
            ctrl_handle_command(&ctrl, &command);
        }

#if (!ROBOT_SIMULATION) && (CTRL_AUTO_START_MS > 0)
        /* Test only: one START, as if a user had sent it. */
        if ((!b_auto_started) && ctrl.b_imu_healthy &&
            (board_millis() >= CTRL_AUTO_START_MS))
        {
            b_auto_started = true;
            command.type   = CONTROL_START;
            command.value  = 0.0f;
            ctrl_handle_command(&ctrl, &command);
            tm_printf((UB *) "CTRL: TEST auto-start: START sent\n");
        }
#endif

        /* Wait for one event, then drain whatever else is queued. */
        msg_size = tk_rcv_mbf(gh_event_mbf, &event,
                              (TMO) ROBOT_CONTROLLER_WAIT_MS);

        while (msg_size > 0)
        {
            ctrl_handle_event(&ctrl, &event);
            msg_size = tk_rcv_mbf(gh_event_mbf, &event, TMO_POL);
        }

#if ROBOT_SIMULATION
        if ((!b_sim_started) && ctrl.b_imu_healthy)
        {
            b_sim_started = true;
            command.type = CONTROL_START;
            ctrl_handle_command(&ctrl, &command);
        }
#endif
        ctrl_step_avoidance(&ctrl, board_millis());
        ctrl_publish_telemetry(&ctrl, board_millis());
    }
}

/*** end of file ***/
