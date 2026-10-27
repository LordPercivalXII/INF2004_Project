/** @file motion.c
 *
 * @brief Motion task: applies the latest command with a per-wheel speed
 *        PID and reports odometry to the controller every 20 ms.
 *
 *   FreeRTOS                          micro T-Kernel
 *   --------------------------------  ---------------------------------
 *   vTaskDelayUntil(20 ms)            cyclic handler + tk_slp_tsk()
 *   xQueueReceive(cmd_q, .., 0) loop  tk_rcv_mbf(.., TMO_POL) loop
 *   Robot_PostEvent()                 controller_post_event()
 */

#include <float.h>
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../common/rmath.h"
#include "../controller/controller.h"

#define MOTION_INTEGRAL_LIMIT   (1.0f)
#define MOTION_PI               (3.14159265f)

static bool
motion_float_is_finite (float value)
{
    return ((value == value) && (rmath_absf(value) <= FLT_MAX));
}

static bool
motion_command_is_valid (motion_command_t const * p_command)
{
    bool b_valid = false;

    if (NULL != p_command)
    {
        switch (p_command->type)
        {
            case MOTION_STOP:
            case MOTION_BRAKE:
                b_valid = true;
                break;

            case MOTION_DRIVE:
                b_valid = motion_float_is_finite(p_command->left_speed) &&
                          motion_float_is_finite(p_command->right_speed);
                break;

            case MOTION_TURN:
                b_valid = motion_float_is_finite(p_command->turn_angle_deg) &&
                          (0.0f != p_command->turn_angle_deg) &&
                          (rmath_absf(p_command->turn_angle_deg) <= 360.0f) &&
                          (0U != p_command->turn_id);
                break;

            default:
                break;
        }
    }

    return (b_valid);
}

/*!
 * @brief Queue a motion command without waiting.
 *
 * @return true if queued.
 */
bool
motion_send_command (motion_command_t const * p_command)
{
    bool b_ok = false;

    if (motion_command_is_valid(p_command) && (gh_motion_cmd_mbf > 0))
    {
        b_ok = (E_OK == tk_snd_mbf(gh_motion_cmd_mbf, (void *) p_command,
                                   (INT) sizeof(*p_command), TMO_POL));
    }

    return (b_ok);
}

void
motion_pid_reset (motion_pid_t * p_pid)
{
    p_pid->integral       = 0.0f;
    p_pid->previous_error = 0.0f;
}

/*!
 * @brief One PID step: feed-forward target plus correction.
 *
 * @return Motor output clamped to +-1.
 */
float
motion_pid_step (motion_pid_t * p_pid, float target, float measured,
                 float interval_s)
{
    float const error = target - measured;
    float const derivative = (error - p_pid->previous_error) / interval_s;
    float const old_integral = p_pid->integral;
    float const candidate_integral =
        rmath_clampf(old_integral + (error * interval_s),
                     -MOTION_INTEGRAL_LIMIT, MOTION_INTEGRAL_LIMIT);
    float output = target + (MOTOR_PID_KP * error) +
                   (MOTOR_PID_KI * candidate_integral) +
                   (MOTOR_PID_KD * derivative);

    if ((rmath_absf(output) > 1.0f) && (error * output > 0.0f))
    {
        output = target + (MOTOR_PID_KP * error) +
                 (MOTOR_PID_KI * old_integral) +
                 (MOTOR_PID_KD * derivative);
    }
    else
    {
        p_pid->integral = candidate_integral;
    }
    p_pid->previous_error = error;

    return (rmath_clampf(output, -1.0f, 1.0f));
}

/*!
 * @brief Keep only the newest queued command.
 */
static bool
motion_take_commands (motion_command_t * p_requested)
{
    motion_command_t incoming;
    INT              msg_size;
    bool             b_received = false;

    msg_size = tk_rcv_mbf(gh_motion_cmd_mbf, &incoming, TMO_POL);

    while (msg_size > 0)
    {
        *p_requested = incoming;
        b_received = true;
        msg_size = tk_rcv_mbf(gh_motion_cmd_mbf, &incoming, TMO_POL);
    }

    return (b_received);
}

static void
motion_post (int32_t left_now, int32_t right_now, float left_speed,
             float right_speed, float distance_m, uint32_t completed_turn_id)
{
    system_event_t event = {0};

    event.type                        = SYSTEM_EVENT_MOTION;
    event.timestamp_ms                = board_millis();
    event.data.motion.left_ticks      = left_now;
    event.data.motion.right_ticks     = right_now;
    event.data.motion.left_speed_m_s  = left_speed;
    event.data.motion.right_speed_m_s = right_speed;
    event.data.motion.distance_m      = distance_m;
    event.data.motion.turn_complete   = (0U != completed_turn_id);
    event.data.motion.turn_id         = completed_turn_id;
    (void) controller_post_event(&event);
}

static float
motion_target_speed (motion_command_t const * p_command, bool b_left_wheel)
{
    float target = 0.0f;

    if (MOTION_DRIVE == p_command->type)
    {
        target = b_left_wheel ? p_command->left_speed : p_command->right_speed;
        target = rmath_clampf(target, -ROBOT_MAX_SPEED, ROBOT_MAX_SPEED);
    }
    else if (MOTION_TURN == p_command->type)
    {
        float const direction = (p_command->turn_angle_deg > 0.0f) ? 1.0f : -1.0f;
        target = b_left_wheel ? -direction * MOTOR_TURN_SPEED_M_S
                              :  direction * MOTOR_TURN_SPEED_M_S;
    }
    else
    {
        /* Stop and brake commands have no wheel-speed target. */
    }

    return (target);
}

static float
motion_turn_speed (float remaining_deg)
{
    float const ramp = rmath_clampf(remaining_deg / MOTOR_TURN_SLOWDOWN_DEG,
                                    0.0f, 1.0f);

    return (MOTOR_TURN_MIN_SPEED_M_S +
            (MOTOR_TURN_SPEED_M_S - MOTOR_TURN_MIN_SPEED_M_S) * ramp);
}

/*!
 * @brief Motion task (20 ms, woken by its cyclic handler).
 */
void
motion_task (INT stacd, void * p_exinf)
{
    float const interval_s = (float) ROBOT_MOTION_PERIOD_MS / 1000.0f;
    float const m_per_tick = (RMATH_PI * MOTOR_WHEEL_DIAMETER_M) /
                             MOTOR_ENCODER_TICKS_PER_REV;
    motion_command_t requested = {MOTION_STOP, 0.0f, 0.0f, 0.0f, 0U};
    motion_pid_t     left_pid;
    motion_pid_t     right_pid;
    float            left_fractional_ticks = 0.0f;
    float            right_fractional_ticks = 0.0f;
    int32_t          prev_left  = 0;
    int32_t          prev_right = 0;
    int32_t          left_now;
    int32_t          right_now;
    float            total_distance = 0.0f;
    float            left_speed;
    float            right_speed;
    float            left_out;
    float            right_out;
    float            left_target;
    float            right_target;
    float            turn_progress_deg = 0.0f;
    uint32_t         active_turn_id = 0U;
    uint32_t         completed_turn_id;
    uint32_t         left_last_tick_ms;
    uint32_t         right_last_tick_ms;
    uint32_t         now_ms;
    bool             encoder_fault_latched = false;
    bool             encoder_fault_pending = false;

    (void) stacd;
    (void) p_exinf;

    motion_pid_reset(&left_pid);
    motion_pid_reset(&right_pid);
    left_last_tick_ms = board_millis();
    right_last_tick_ms = left_last_tick_ms;

#if !ROBOT_SIMULATION
    motor_init();

    if (!encoder_init())
    {
        tm_printf((UB *) "S2: encoder interrupt registration failed\n");
    }
#endif

    for (;;)
    {
        motion_command_t incoming;

        (void) tk_slp_tsk(TMO_FEVR);    /* woken every 20 ms */

        if (motion_take_commands(&incoming))
        {
            float const old_left_target = motion_target_speed(&requested, true);
            float const old_right_target = motion_target_speed(&requested, false);
            bool const b_entering_turn =
                (MOTION_TURN == incoming.type) &&
                ((MOTION_TURN != requested.type) ||
                 (incoming.turn_id != requested.turn_id));

            if ((MOTION_TURN == requested.type) &&
                ((MOTION_TURN != incoming.type) ||
                 (incoming.turn_id != requested.turn_id)))
            {
                active_turn_id = 0U;
            }

            requested = incoming;

            if (b_entering_turn)
            {
                turn_progress_deg = 0.0f;
                active_turn_id = incoming.turn_id;
                motion_pid_reset(&left_pid);
                motion_pid_reset(&right_pid);
            }

            if ((rmath_absf(old_left_target) <= 0.001f) &&
                (rmath_absf(motion_target_speed(&requested, true)) > 0.001f))
            {
                left_last_tick_ms = board_millis();
                encoder_fault_latched = false;
            }
            if ((rmath_absf(old_right_target) <= 0.001f) &&
                (rmath_absf(motion_target_speed(&requested, false)) > 0.001f))
            {
                right_last_tick_ms = board_millis();
                encoder_fault_latched = false;
            }

            if ((MOTION_DRIVE != requested.type) &&
                (MOTION_TURN != requested.type))
            {
                motion_pid_reset(&left_pid);
                motion_pid_reset(&right_pid);
            }
        }

        left_target = motion_target_speed(&requested, true);
        right_target = motion_target_speed(&requested, false);
        if (MOTION_TURN == requested.type)
        {
            float const turn_direction =
                (requested.turn_angle_deg > 0.0f) ? 1.0f : -1.0f;
            float const remaining_deg =
                rmath_absf(requested.turn_angle_deg) -
                (turn_direction * turn_progress_deg);
            float const turn_speed = motion_turn_speed(remaining_deg);

            left_target = -turn_direction * turn_speed;
            right_target = turn_direction * turn_speed;
        }

#if ROBOT_SIMULATION
        if ((MOTION_DRIVE == requested.type) || (MOTION_TURN == requested.type))
        {
            float const left_total = left_fractional_ticks +
                (left_target * interval_s / m_per_tick);
            float const right_total = right_fractional_ticks +
                (right_target * interval_s / m_per_tick);
            int32_t const left_whole = (int32_t) left_total;
            int32_t const right_whole = (int32_t) right_total;

            left_fractional_ticks = left_total - (float) left_whole;
            right_fractional_ticks = right_total - (float) right_whole;
            encoder_sim_add(left_whole, right_whole);
        }
#endif

        encoder_read(&left_now, &right_now);
        now_ms = board_millis();
        {
            int32_t const left_delta = left_now - prev_left;
            int32_t const right_delta = right_now - prev_right;

            left_speed  = ((float) left_delta * m_per_tick) / interval_s;
            right_speed = ((float) right_delta * m_per_tick) / interval_s;
            total_distance += ((float) (left_delta + right_delta) * 0.5f) *
                              m_per_tick;
            if (0 != left_delta)
            {
                left_last_tick_ms = now_ms;
            }
            if (0 != right_delta)
            {
                right_last_tick_ms = now_ms;
            }
            prev_left  = left_now;
            prev_right = right_now;
        }

        completed_turn_id = 0U;
        if (MOTION_TURN == requested.type)
        {
            float const wheel_delta_m =
                (right_speed - left_speed) * interval_s;
            float const turn_direction =
                (requested.turn_angle_deg > 0.0f) ? 1.0f : -1.0f;
            float remaining_deg;

            turn_progress_deg += (wheel_delta_m / MOTOR_TRACK_WIDTH_M) *
                                 (180.0f / MOTION_PI);
            remaining_deg = rmath_absf(requested.turn_angle_deg) -
                            (turn_direction * turn_progress_deg);
            if (remaining_deg <= 0.0f)
            {
                completed_turn_id = active_turn_id;
                requested.type = MOTION_STOP;
                active_turn_id = 0U;
                left_target = 0.0f;
                right_target = 0.0f;
                motion_pid_reset(&left_pid);
                motion_pid_reset(&right_pid);
            }
            else
            {
                float const turn_speed = motion_turn_speed(remaining_deg);

                left_target = -turn_direction * turn_speed;
                right_target = turn_direction * turn_speed;
            }
        }

        if (!encoder_fault_latched &&
            (((rmath_absf(left_target) > 0.001f) &&
              ((uint32_t) (now_ms - left_last_tick_ms) >=
               MOTOR_ENCODER_TIMEOUT_MS)) ||
             ((rmath_absf(right_target) > 0.001f) &&
              ((uint32_t) (now_ms - right_last_tick_ms) >=
               MOTOR_ENCODER_TIMEOUT_MS))))
        {
            requested.type = MOTION_BRAKE;
            active_turn_id = 0U;
            completed_turn_id = 0U;
            encoder_fault_latched = true;
            encoder_fault_pending = true;
            motion_pid_reset(&left_pid);
            motion_pid_reset(&right_pid);
        }

        if (encoder_fault_pending)
        {
            system_event_t fault = {0};

            fault.type         = SYSTEM_EVENT_FAULT;
            fault.timestamp_ms = now_ms;
            if (controller_post_event(&fault))
            {
                encoder_fault_pending = false;
            }
        }

        left_out = 0.0f;
        right_out = 0.0f;
        if ((MOTION_DRIVE == requested.type) || (MOTION_TURN == requested.type))
        {
            left_out = motion_pid_step(&left_pid, left_target, left_speed,
                                       interval_s);
            right_out = motion_pid_step(&right_pid, right_target, right_speed,
                                        interval_s);
        }

#if !ROBOT_SIMULATION
        motor_set(PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_DIR, left_out);
        motor_set(PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_DIR, right_out);
#else
        (void) left_out;
        (void) right_out;
#endif

        motion_post(left_now, right_now, left_speed, right_speed,
                    total_distance, completed_turn_id);
    }
}

/*** end of file ***/
