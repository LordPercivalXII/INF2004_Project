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
#include "../subsystem5/subsystem5.h"
#include "../subsystem4/imu_height.h"
#include "../subsystem4/imu_processing.h"

/* Steering is a fraction of the drive speed, so a given line error gives
 * the same path curvature at any speed: 1.0 = inner wheel stopped (pivot
 * on it), above 1.0 = inner wheel backwards. */
#define CTRL_STEER_GAIN         (0.40f) /* steer per unit line error    */
#define CTRL_STEER_LIMIT        (1.20f)
#define CTRL_STEER_SMOOTH       (0.35f) /* weight of newest sample      */
#define CTRL_STEER_KD           (0.0f)   /* steer per (error/s)          */
#define CTRL_CURVE_SLOW         (0.50f)  /* speed cut at full steer      */
#define CTRL_STEER_DEADBAND     (0.10f)
#define CTRL_LOST_PIVOT_SPEED   (0.20f) /* m/s outer wheel while searching */
#define CTRL_LOST_FIRST_MS      (1300U)  /* arc towards last side this long */
#define CTRL_LOST_SWEEP_MS      (1000U) /* alternate sides after initial arc */
#define CTRL_WHEEL_MIN          (0.04f) /* m/s: slowest wheel that turns   */
#define CTRL_WHEEL_ZERO         (0.03f) /* below this the wheel is stopped */
#define CTRL_SLEW_PER_CYCLE     (0.02f) /* max wheel target change / 10 ms */
#define CTRL_LOST_SIDE_LEVEL    (0.20f) /* |error| that counts as a side  */
#define CTRL_CENTRED_LEVEL      (0.12f) /* |error| below this = centred   */
#define CTRL_CENTRED_CYCLES     (10U)   /* centred this long: forget side */
#define CTRL_CENTRE_LEVEL       (0.60f) /* last |error| below this: line is between the sensors */
#define CTRL_BETWEEN_STEER      (0.25f) /* steer while line is between sensors */
#define CTRL_CENTRE_HOLD_CYCLES (40U)   /* x 10 ms to drive straight on both-white */
#define CTRL_LOST_CYCLES        (8U)    /* x 10 ms lost before pivoting   */
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

#define CTRL_IMU_BAD_MS         (1000U) /* IMU must be bad this long to fault */
#define CTRL_FAULT_RETRY_MS     (1500U) /* auto-resume after a fault       */

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
    bool                b_turn_in_progress;  /* also set while
                                                approaching a junction */
    bool                b_junction_approach; /* driving to the pivot   */
    float               junction_start_m;    /* odometer at detection  */
    float               junction_turn_deg;   /* turn to make there     */
    uint32_t            junction_start_ms;
    uint16_t            pending_turn;
    uint32_t            next_turn_id;
    uint32_t            active_turn_id;
    bool                b_line_detected;
    float               prev_line_error;    /* for the D term           */
    bool                b_steer_primed;     /* prev_line_error is valid */
    uint32_t            lost_cycles;
    float               last_seen_error;
    uint32_t            imu_bad_since_ms;
    uint32_t            fault_since_ms;
    uint32_t            centred_cycles;
    uint32_t            lost_start_ms;
    float               last_side_any;      /* last side seen, never cleared */
    float               last_side;          /* +1/-1: side line was last off-centre */
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

static float s_cmd_left  = 0.0f;
static float s_cmd_right = 0.0f;

/* A wheel commanded this slowly barely turns, so the encoder watchdog in
   motion.c trips and brakes the car for good.  Never command a wheel
   inside the band: push it out to the minimum, or stop it. */
static float
ctrl_wheel_floor (float speed, bool b_allow_stop)
{
    if (b_allow_stop && (rmath_absf(speed) < CTRL_WHEEL_ZERO))
    {
        return 0.0f;
    }
    if (speed <= -CTRL_WHEEL_MIN)
    {
        return speed;       /* deliberate reverse: in-place search turn */
    }
    return (speed < CTRL_WHEEL_MIN) ? CTRL_WHEEL_MIN : speed;
}

/* Line-following drive: rate-limited so the wheels ramp instead of jerk. */
static void
ctrl_set_motion_slewed (float left_speed, float right_speed,
                        bool b_allow_stop)
{
    s_cmd_left  += rmath_clampf(left_speed - s_cmd_left,
                                -CTRL_SLEW_PER_CYCLE, CTRL_SLEW_PER_CYCLE);
    s_cmd_right += rmath_clampf(right_speed - s_cmd_right,
                                -CTRL_SLEW_PER_CYCLE, CTRL_SLEW_PER_CYCLE);
    ctrl_set_motion(ctrl_wheel_floor(s_cmd_left, b_allow_stop),
                    ctrl_wheel_floor(s_cmd_right, b_allow_stop));
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
    p_ctrl->fault_since_ms = board_millis();
    tm_printf((UB *) "CTRL: FAULT code %u\n", (unsigned int) code);
    p_ctrl->b_start_pending = false;
    p_ctrl->start_requested_ms = 0U;
    p_ctrl->state      = MISSION_FAULT;
    p_ctrl->b_turn_in_progress = false;
    p_ctrl->b_junction_approach = false;
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
                (!p_ctrl->b_start_pending))
            {
                p_ctrl->b_start_pending = true;
                p_ctrl->barcode = 0U;   /* telemetry copy only */
                p_ctrl->start_requested_ms = board_millis();
                ctrl_clear_bits(SYSTEM_BIT_MISSION_RUN);
                ctrl_stop_motion();
                tm_printf((UB *) "CTRL: START received, waiting 3 seconds%s\n",
                          p_ctrl->b_imu_healthy ? "" : " (IMU not ready yet)");
            }
            else
            {
                tm_printf((UB *) "CTRL: START ignored: state=%d pending=%d\n",
                          (int) p_ctrl->state,
                          (int) p_ctrl->b_start_pending);
            }
            break;

        case CONTROL_STOP:
            p_ctrl->b_start_pending = false;
            p_ctrl->start_requested_ms = 0U;
            p_ctrl->state = MISSION_IDLE;
            p_ctrl->b_turn_in_progress = false;
            p_ctrl->b_junction_approach = false;
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

/* ------------------------------------------------------------------ */
/* Junction turns                                                      */
/* ------------------------------------------------------------------ */

/*!
 * @brief Turn angle for a junction action (+ = left, as MOTION_TURN).
 *
 * @return 0 for straight on (BARCODE_CMD_NONE) or anything unknown.
 */
static float
ctrl_junction_angle (uint16_t action)
{
    float angle_deg = 0.0f;

    switch (action)
    {
        case BARCODE_CMD_LEFT:
            angle_deg = 90.0f;
            break;

        case BARCODE_CMD_RIGHT:
            angle_deg = -90.0f;
            break;

        case BARCODE_CMD_UTURN:
            angle_deg = 180.0f;
            break;

        default:
            angle_deg = 0.0f;   /* BARCODE_CMD_NONE: straight on */
            break;
    }

    return (angle_deg);
}

/*!
 * @brief Start a junction turn: drive straight to the junction centre
 *        (JUNCTION_APPROACH_M on the odometer), then turn (see
 *        ctrl_junction_step).  Line steering is suspended meanwhile.
 */
static void
ctrl_junction_begin (ctrl_state_t * p_ctrl, float angle_deg)
{
    float const speed = rmath_minf(JUNCTION_APPROACH_SPEED,
                                   ctrl_drive_speed(p_ctrl));

    p_ctrl->b_junction_approach = true;
    p_ctrl->b_turn_in_progress  = true;     /* suspends line steering */
    p_ctrl->active_turn_id      = 0U;
    p_ctrl->junction_turn_deg   = angle_deg;
    p_ctrl->junction_start_m    = p_ctrl->distance_m;
    p_ctrl->junction_start_ms   = board_millis();
    ctrl_set_motion(speed, speed);
    tm_printf((UB *) "CTRL: junction -> turn %d deg after %d mm\n",
              (int) angle_deg, (int) (JUNCTION_APPROACH_M * 1000.0f));
}

/*!
 * @brief While approaching: once the odometer has advanced
 *        JUNCTION_APPROACH_M (or the approach takes too long), make the
 *        closed-loop turn.  Called on every motion event.
 */
static void
ctrl_junction_step (ctrl_state_t * p_ctrl)
{
    bool const b_arrived = ((p_ctrl->distance_m - p_ctrl->junction_start_m)
                            >= JUNCTION_APPROACH_M);
    bool const b_timeout = ((board_millis() - p_ctrl->junction_start_ms)
                            >= JUNCTION_APPROACH_TIMEOUT_MS);

    if (p_ctrl->b_junction_approach && (b_arrived || b_timeout))
    {
        p_ctrl->b_junction_approach = false;
        p_ctrl->next_turn_id++;

        if (0U == p_ctrl->next_turn_id)
        {
            p_ctrl->next_turn_id++;
        }

        if (b_timeout && !b_arrived)
        {
            tm_printf((UB *) "CTRL: junction approach timed out, "
                      "turning here\n");
        }

        if (ctrl_turn_motion(p_ctrl->junction_turn_deg, p_ctrl->next_turn_id))
        {
            p_ctrl->active_turn_id = p_ctrl->next_turn_id;
        }
        else
        {
            ctrl_enter_fault(p_ctrl, FAULT_MOTION_COMMAND);
        }
    }
}

static void
ctrl_handle_line (ctrl_state_t * p_ctrl, line_event_t const * p_line)
{
    p_ctrl->line_error      = p_line->line_error;
    p_ctrl->b_line_detected = p_line->b_line_detected;

    if (p_line->b_line_detected)
    {
        p_ctrl->lost_cycles = 0U;
        p_ctrl->last_seen_error = p_line->line_error;
        float const abs_error = rmath_absf(p_line->line_error);

        /* The last sample before a sharp corner often reads near centre,
           so the side is remembered until the line has stayed centred. */
        if (abs_error >= CTRL_LOST_SIDE_LEVEL)
        {
            p_ctrl->last_side = (p_line->line_error > 0.0f) ? 1.0f : -1.0f;
            p_ctrl->last_side_any = p_ctrl->last_side;
            p_ctrl->centred_cycles = 0U;
        }
        else if (abs_error < CTRL_CENTRED_LEVEL)
        {
            if (++p_ctrl->centred_cycles >= CTRL_CENTRED_CYCLES)
            {
                p_ctrl->last_side = 0.0f;
            }
        }
        else
        {
            /* In between: keep the remembered side. */
        }
    }
    else
    {
        if (0U == p_ctrl->lost_cycles)
        {
            p_ctrl->lost_start_ms = board_millis();
        }
        p_ctrl->lost_cycles++;
    }

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
        (!p_ctrl->b_turn_in_progress))
    {
        /* The stored barcode decides (S3 resolved it; none stored or
           "straight" = BARCODE_CMD_NONE): straight carries on following
           the line, a turn first drives to the junction centre. */
        float const angle_deg = ctrl_junction_angle(p_ctrl->pending_turn);

        p_ctrl->pending_turn = (uint16_t) BARCODE_CMD_NONE;

        if (0.0f == angle_deg)
        {
            tm_printf((UB *) "CTRL: junction -> STRAIGHT (%s)\n",
                      ('\0' != p_line->junction_char) ? "barcode"
                                                       : "no barcode stored");
        }
        else
        {
            ctrl_junction_begin(p_ctrl, angle_deg);
        }
    }

    if (((MISSION_FOLLOWING == p_ctrl->state) &&
         (!p_ctrl->b_turn_in_progress)) ||
        ((MISSION_AVOIDING == p_ctrl->state) &&
         avoid_is_following(&p_ctrl->avoid)))
    {
        uint32_t const lost_ms = board_millis() - p_ctrl->lost_start_ms;
        /* The sensors sit just outside the line edges, so a centred car
           sees white on both.  If the line was last seen near centre,
           that is "between the sensors": drive straight, not "lost". */
        bool const b_between = (!p_line->b_line_detected) &&
            (p_ctrl->lost_cycles < CTRL_CENTRE_HOLD_CYCLES);

        if ((!p_line->b_line_detected) && (0.0f != p_ctrl->last_side_any) &&
            (p_ctrl->lost_cycles >= CTRL_LOST_CYCLES) &&
            (!b_between) &&
            (MISSION_FOLLOWING == p_ctrl->state))
        {
            /* Line lost at a sharp corner: arc towards the side it was
               last seen.  If that does not find it, sweep the other way
               instead of spinning round, then fall back to steering. */
            /* Search toward the last seen side first, then alternate on a
               fixed interval until either sensor reacquires the line. */
            bool const b_first_arc = (lost_ms < CTRL_LOST_FIRST_MS);
            uint32_t const sweep_phase = b_first_arc ? 0U :
                ((lost_ms - CTRL_LOST_FIRST_MS) / CTRL_LOST_SWEEP_MS);
            float const side = (0U == (sweep_phase & 1U))
                ? p_ctrl->last_side_any : -p_ctrl->last_side_any;
            /* Arc gently towards the last side. A forward outer wheel with
               the inner wheel stopped is more reliable than commanding both
               wheels slowly in opposite directions during line reacquisition. */
            float const outer = CTRL_LOST_PIVOT_SPEED;
            float const inner = 0.0f;

            ctrl_set_motion_slewed((side > 0.0f) ? outer : inner,
                                   (side > 0.0f) ? inner : outer, true);
            p_ctrl->b_steer_primed = false;
            return;
        }

        /* PD steering.  With the line sensors on the line edges the
           error is continuous near the centre; the D term damps the
           zig-zag.  Deadband reduces jitter near the line centre. */
        static float smooth_error = 0.0f;
        /* A short sensor dropout reads as full-scale error; hold the
           smoothed value instead of letting it kick the steering. */
        float const raw_error =
            ((!p_line->b_line_detected) &&
             (b_between || (p_ctrl->lost_cycles < CTRL_LOST_CYCLES)))
            ? (b_between ? (0.5f * p_ctrl->last_seen_error) : smooth_error)
            : p_ctrl->line_error;
        smooth_error = p_ctrl->b_steer_primed
            ? (smooth_error + (CTRL_STEER_SMOOTH * (raw_error - smooth_error)))
            : raw_error;
        p_ctrl->line_error = smooth_error;
        float const derivative = p_ctrl->b_steer_primed
            ? ((p_ctrl->line_error - p_ctrl->prev_line_error) /
               CTRL_LINE_DT_S)
            : 0.0f;
        /* Shifted deadband: no step in steering at the band edge. */
        float const abs_err = rmath_absf(p_ctrl->line_error);
        float const error_deadband = (abs_err < CTRL_STEER_DEADBAND) ? 0.0f
            : ((p_ctrl->line_error > 0.0f) ? (abs_err - CTRL_STEER_DEADBAND)
                                           : -(abs_err - CTRL_STEER_DEADBAND));
        float const pd_steer =
            rmath_clampf((error_deadband * CTRL_STEER_GAIN) +
                         (derivative * CTRL_STEER_KD),
                         -CTRL_STEER_LIMIT, CTRL_STEER_LIMIT);

        /* Line dropped out between the sensors (long curve, S-bend): keep
           turning decisively towards the side it was last seen, so the
           car follows the bend instead of drifting straight on. */
        float const steer = (b_between &&
                             (rmath_absf(p_ctrl->last_seen_error) >=
                              CTRL_LOST_SIDE_LEVEL))
            ? ((p_ctrl->last_seen_error > 0.0f ? 1.0f : -1.0f) *
               rmath_minf(CTRL_BETWEEN_STEER *
                          (1.0f + ((float)p_ctrl->lost_cycles / 15.0f)),
                          CTRL_STEER_LIMIT))
            : pd_steer;

        p_ctrl->prev_line_error = p_ctrl->line_error;
        p_ctrl->b_steer_primed  = true;

        /* Slow down while steering hard, so the sensors keep up with
           tight curves at high speed; full speed again on straights. */
        float const speed = rmath_maxf(
            ctrl_drive_speed(p_ctrl) *
                (1.0f - (CTRL_CURVE_SLOW *
                         rmath_minf(rmath_absf(steer), 1.0f))),
            ROBOT_MIN_SPEED);

        float const left_cmd  = speed * (1.0f + steer);
        float const right_cmd = speed * (1.0f - steer);
        ctrl_set_motion_slewed(left_cmd, right_cmd, false);
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
            if (0U == p_ctrl->imu_bad_since_ms)
            {
                p_ctrl->imu_bad_since_ms = board_millis() | 1U;
            }
            if ((board_millis() - p_ctrl->imu_bad_since_ms) >= CTRL_IMU_BAD_MS)
            {
                ctrl_enter_fault(p_ctrl, FAULT_IMU);
            }
        }
        else if (p_imu->b_calibrated &&
                 (p_imu->b_shock_detected || b_new_shock))
        {
            p_ctrl->shock_until_ms = board_millis() + CTRL_SHOCK_HOLD_MS;
        }
        else
        {
            p_ctrl->imu_bad_since_ms = 0U;
        }
        if (p_imu->b_calibrated)
        {
            p_ctrl->imu_bad_since_ms = 0U;
        }
    }
    else
    {
        /* Brief I2C glitches (motor noise) are ridden out; the car keeps
           its last drive command and only faults if it persists. */
        p_ctrl->b_imu_healthy = false;
        if (0U == p_ctrl->imu_bad_since_ms)
        {
            p_ctrl->imu_bad_since_ms = board_millis() | 1U;
        }
        if ((board_millis() - p_ctrl->imu_bad_since_ms) >= CTRL_IMU_BAD_MS)
        {
            ctrl_enter_fault(p_ctrl, FAULT_IMU);
        }
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
        p_ctrl->b_junction_approach = false;
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
            /* S5 looks where the car is going: from how tightly the
               wheels say it is turning (curves, junction spins, line
               searches alike).  Straight ahead when not following. */
            obstacle_set_lead((MISSION_FOLLOWING == p_ctrl->state)
                ? obstacle_lead_from_wheels(
                      p_event->data.motion.left_speed_m_s,
                      p_event->data.motion.right_speed_m_s)
                : 0.0f);
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
            ctrl_junction_step(p_ctrl);

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
            if ((MISSION_FOLLOWING == p_ctrl->state) &&
                (!p_ctrl->b_turn_in_progress))
            {
                /* Encoder watchdog braked the car (a wheel stalled).
                   Restart from standstill on the next line cycle instead
                   of staying stopped. */
                s_cmd_left  = 0.0f;
                s_cmd_right = 0.0f;
                p_ctrl->b_steer_primed = false;
                tm_printf((UB *) "CTRL: wheel stall, restarting\n");
            }
            else
            {
                ctrl_enter_fault(p_ctrl, FAULT_EXTERNAL);
            }
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
            if (p_avoid->b_rear_bounded)
            {
                tm_printf((UB *) "S5: obstacle thickness at most %d cm "
                          "(too shallow to see from the side)\n",
                          (int) (p_avoid->thickness_cm + 0.5f));
            }
            else if (p_avoid->b_rear_found)
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
            /* Bench autostart: skip the 3 s dashboard countdown. */
            if (ctrl.b_start_pending)
            {
                ctrl.start_requested_ms = board_millis() - CTRL_START_DELAY_MS;
            }
        }
#endif
        if (tk_rcv_mbf(gh_control_mbf, &command, TMO_POL) > 0)
        {
            ctrl_handle_command(&ctrl, &command);
        }

        /* Auto-resume after a recoverable fault once the IMU is healthy. */
        if ((MISSION_FAULT == ctrl.state) && ctrl.b_imu_healthy &&
            (FAULT_MOTION_COMMAND != ctrl.fault_code) &&
            ((board_millis() - ctrl.fault_since_ms) >= CTRL_FAULT_RETRY_MS))
        {
            ctrl.fault_code        = 0U;
            ctrl.imu_bad_since_ms  = 0U;
            ctrl.b_steer_primed    = false;
            s_cmd_left             = 0.0f;
            s_cmd_right            = 0.0f;
            ctrl.state             = MISSION_FOLLOWING;
            ctrl_clear_bits(SYSTEM_BIT_FAULT);
            ctrl_set_bits(SYSTEM_BIT_MISSION_RUN);
            tm_printf((UB *) "CTRL: fault cleared, resuming\n");
        }

        /* Nonblocking 3-second START countdown. STOP clears the pending flag.
         * A START that arrives before the IMU has calibrated stays pending
         * and runs as soon as the IMU is healthy. */
        if (ctrl.b_start_pending &&
            ((uint32_t) (board_millis() - ctrl.start_requested_ms) >=
             CTRL_START_DELAY_MS))
        {
            if (MISSION_IDLE != ctrl.state)
            {
                ctrl.b_start_pending = false;
                ctrl.start_requested_ms = 0U;
                ctrl_stop_motion();
                tm_printf((UB *) "CTRL: START cancelled, robot not ready\n");
            }
            else if (ctrl.b_imu_healthy)
            {
                ctrl.b_start_pending = false;
                ctrl.start_requested_ms = 0U;
                ctrl.state = MISSION_FOLLOWING;
                ctrl_set_bits(SYSTEM_BIT_MISSION_RUN);
                tm_printf((UB *) "CTRL: Countdown complete, mission STARTED\n");
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
