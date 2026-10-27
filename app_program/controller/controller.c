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

#define CTRL_TURN_OUT_MS        (350U)
#define CTRL_PASS_MS            (900U)
#define CTRL_TURN_BACK_MS       (350U)
#define CTRL_REACQUIRE_MS       (2500U)
#define CTRL_STEER_GAIN         (0.18f)
#define CTRL_STEER_LIMIT        (0.25f)
#define CTRL_REACQUIRE_ERROR    (0.5f)
#define CTRL_BARCODE_SLOW_SIDE  (0.55f)
#define CTRL_SPIN_FACTOR        (0.45f)
#define CTRL_PASS_FACTOR        (0.55f)
#define CTRL_CREEP_FACTOR       (0.35f)
#define CTRL_SHOCK_FACTOR       (0.7f)

#define FAULT_EXTERNAL          (1U)
#define FAULT_IMU               (2U)
#define FAULT_REACQUIRE         (3U)
#define FAULT_BOXED_IN          (4U)

ID gh_event_mbf   = E_ID;
ID gh_control_mbf = E_ID;
ID gh_system_flg  = E_ID;

typedef enum
{
    AVOIDANCE_NONE = 0,
    AVOIDANCE_TURN_OUT,
    AVOIDANCE_PASS_OBSTACLE,
    AVOIDANCE_TURN_BACK,
    AVOIDANCE_REACQUIRE
} avoidance_phase_t;

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
    avoidance_phase_t   phase;
    int8_t              direction;
    uint32_t            deadline_ms;
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
    motion_command_t command;

    command.type        = MOTION_DRIVE;
    command.left_speed  = left_speed;
    command.right_speed = right_speed;
    (void) motion_send_command(&command);
}

static void
ctrl_stop_motion (void)
{
    motion_command_t command;

    command.type        = MOTION_STOP;
    command.left_speed  = 0.0f;
    command.right_speed = 0.0f;
    (void) motion_send_command(&command);
}

static void
ctrl_spin (ctrl_state_t const * p_ctrl, int8_t direction)
{
    float const speed = p_ctrl->base_speed * CTRL_SPIN_FACTOR;

    ctrl_set_motion(-(float) direction * speed, (float) direction * speed);
}

static void
ctrl_enter_fault (ctrl_state_t * p_ctrl, uint32_t code)
{
    p_ctrl->fault_code = code;
    p_ctrl->state      = MISSION_FAULT;
    ctrl_set_bits(SYSTEM_BIT_FAULT);
    ctrl_clear_bits(SYSTEM_BIT_MISSION_RUN);
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
            if (MISSION_FAULT != p_ctrl->state)
            {
                p_ctrl->state = MISSION_FOLLOWING;
                ctrl_set_bits(SYSTEM_BIT_MISSION_RUN);
            }
            break;

        case CONTROL_STOP:
            p_ctrl->state = MISSION_IDLE;
            ctrl_clear_bits(SYSTEM_BIT_MISSION_RUN);
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
    p_ctrl->line_error = p_line->line_error;

    if (p_line->b_barcode_valid)
    {
        p_ctrl->barcode = p_line->barcode;

        /* Waypoint codes 1 and 2 request left/right. */
        if ((MISSION_FOLLOWING == p_ctrl->state) &&
            ((uint32_t) BARCODE_CMD_LEFT == p_ctrl->barcode))
        {
            ctrl_set_motion(p_ctrl->base_speed * CTRL_BARCODE_SLOW_SIDE,
                            p_ctrl->base_speed);
        }
        else if ((MISSION_FOLLOWING == p_ctrl->state) &&
                 ((uint32_t) BARCODE_CMD_RIGHT == p_ctrl->barcode))
        {
            ctrl_set_motion(p_ctrl->base_speed,
                            p_ctrl->base_speed * CTRL_BARCODE_SLOW_SIDE);
        }
        else
        {
            /* Other codes: continue straight. */
        }
    }

    if (MISSION_FOLLOWING == p_ctrl->state)
    {
        float const correction =
            rmath_clampf(p_ctrl->line_error * CTRL_STEER_GAIN,
                         -CTRL_STEER_LIMIT, CTRL_STEER_LIMIT);

        ctrl_set_motion(p_ctrl->base_speed + correction,
                        p_ctrl->base_speed - correction);
    }
    else if ((MISSION_AVOIDING == p_ctrl->state) &&
             (AVOIDANCE_REACQUIRE == p_ctrl->phase) &&
             p_line->b_line_detected &&
             (rmath_absf(p_ctrl->line_error) <= CTRL_REACQUIRE_ERROR))
    {
        p_ctrl->state = MISSION_FOLLOWING;
        p_ctrl->phase = AVOIDANCE_NONE;
        ctrl_clear_bits(SYSTEM_BIT_OBSTACLE);
    }
    else
    {
        /* Not steering in this state. */
    }
}

static void
ctrl_handle_imu (ctrl_state_t * p_ctrl, imu_event_t const * p_imu)
{
    if (p_imu->b_sensor_valid)
    {
        p_ctrl->b_imu_healthy = true;
        p_ctrl->hump_m        = p_imu->estimated_hump_height_m;

        if (p_imu->b_shock_detected)
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
ctrl_handle_obstacle (ctrl_state_t * p_ctrl,
                      obstacle_event_t const * p_obs)
{
    p_ctrl->obstacle_cm = p_obs->center_distance_cm;

    if (p_obs->b_detected && p_obs->b_scan_complete &&
        (MISSION_FOLLOWING == p_ctrl->state))
    {
        ctrl_set_bits(SYSTEM_BIT_OBSTACLE);

        if (0 == p_obs->bypass_direction)
        {
            ctrl_enter_fault(p_ctrl, FAULT_BOXED_IN);
        }
        else
        {
            p_ctrl->state       = MISSION_AVOIDING;
            p_ctrl->phase       = AVOIDANCE_TURN_OUT;
            p_ctrl->direction   = (p_obs->bypass_direction < 0) ? -1 : 1;
            p_ctrl->deadline_ms = board_millis() + CTRL_TURN_OUT_MS;
            ctrl_spin(p_ctrl, p_ctrl->direction);
        }
    }
    else if ((!p_obs->b_detected) && (MISSION_AVOIDING == p_ctrl->state) &&
             (AVOIDANCE_REACQUIRE == p_ctrl->phase))
    {
        ctrl_set_motion(p_ctrl->base_speed * CTRL_CREEP_FACTOR,
                        p_ctrl->base_speed * CTRL_CREEP_FACTOR);
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
            p_ctrl->distance_m = p_event->data.motion.distance_m;
            break;

        case SYSTEM_EVENT_IMU:
            ctrl_handle_imu(p_ctrl, &p_event->data.imu);
            break;

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
/* Timed avoidance manoeuvre and telemetry                             */
/* ------------------------------------------------------------------ */

static void
ctrl_step_avoidance (ctrl_state_t * p_ctrl, uint32_t now_ms)
{
    float const pass = p_ctrl->base_speed * CTRL_PASS_FACTOR;
    float const creep = p_ctrl->base_speed * CTRL_CREEP_FACTOR;

    if ((MISSION_AVOIDING == p_ctrl->state) &&
        ((int32_t) (now_ms - p_ctrl->deadline_ms) >= 0))
    {
        switch (p_ctrl->phase)
        {
            case AVOIDANCE_TURN_OUT:
                p_ctrl->phase       = AVOIDANCE_PASS_OBSTACLE;
                p_ctrl->deadline_ms = now_ms + CTRL_PASS_MS;
                ctrl_set_motion(pass, pass);
                break;

            case AVOIDANCE_PASS_OBSTACLE:
                p_ctrl->phase       = AVOIDANCE_TURN_BACK;
                p_ctrl->deadline_ms = now_ms + CTRL_TURN_BACK_MS;
                ctrl_spin(p_ctrl, (int8_t) -p_ctrl->direction);
                break;

            case AVOIDANCE_TURN_BACK:
                p_ctrl->phase       = AVOIDANCE_REACQUIRE;
                p_ctrl->deadline_ms = now_ms + CTRL_REACQUIRE_MS;
                ctrl_set_motion(creep, creep);
                break;

            case AVOIDANCE_REACQUIRE:
                p_ctrl->phase = AVOIDANCE_NONE;
                ctrl_clear_bits(SYSTEM_BIT_OBSTACLE);
                ctrl_enter_fault(p_ctrl, FAULT_REACQUIRE);
                break;

            default:
                /* AVOIDANCE_NONE: nothing timed. */
                break;
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

    ctrl.state             = MISSION_IDLE;
    ctrl.base_speed        = ROBOT_BASE_SPEED;
    ctrl.phase             = AVOIDANCE_NONE;
    ctrl.direction         = 1;
    ctrl.last_telemetry_ms = board_millis();

#if ROBOT_SIMULATION
    ctrl.state = MISSION_FOLLOWING;
    ctrl_set_bits(SYSTEM_BIT_MISSION_RUN);
#endif
    ctrl_set_bits(SYSTEM_BIT_READY);

    for (;;)
    {
        if (tk_rcv_mbf(gh_control_mbf, &command, TMO_POL) > 0)
        {
            ctrl_handle_command(&ctrl, &command);
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
