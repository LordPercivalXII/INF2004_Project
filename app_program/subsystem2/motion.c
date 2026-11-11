/** @file motion.c
 *
 * @brief Motion task: applies the latest command with a per-wheel speed
 *        PID and reports odometry to the controller every 20 ms.
 *        Speed, PID and simulated travel use measured microsecond elapsed
 *        time; queued wake-ups cannot produce extra zero-time samples.
 *
 *   FreeRTOS                          micro T-Kernel
 *   --------------------------------  ---------------------------------
 *   vTaskDelayUntil(20 ms)            cyclic handler + tk_slp_tsk()
 *   xQueueReceive(cmd_q, .., 0) loop  tk_rcv_mbf(.., TMO_POL) loop
 *   Robot_PostEvent()                 controller_post_event()
 *   xQueueReceive(cmd_q, .., 20 ms)   STOP/BRAKE sender: tk_wup_tsk();
 *     (wake on HALT, Tribunal 1)        early wake zeroes the motors
 *
 * Tribunal 1: the motion task used to see a STOP only at its next
 * 20 ms cycle, using the whole NFR10 margin.  motion_queue_command() now
 * wakes the task for STOP/BRAKE; the early wake-up drains the queue and
 * zeroes both motor outputs at once, and the normal cycle then applies
 * the STOP to the PID state as before.  A blocking tk_rcv_mbf() with a
 * 20 ms timeout (the tribunal's FreeRTOS wording) is not used: with the
 * 10 ms tick its timeout would jitter the PID period by up to a tick,
 * whereas the cyclic handler keeps the period exact.
 */

#include <float.h>
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../common/rmath.h"
#include "../controller/controller.h"
#include "../common/robot_packed.h"
#include "../common/fault_mask.h"

#define MOTION_INTEGRAL_LIMIT   (1.0f)
#define MOTION_PI               (3.14159265f)
#define MOTION_STRAIGHT_GAIN    (0.1f)  /* speed error feedback for straight-line */

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

            case MOTION_MOVE_DISTANCE:
                b_valid = motion_float_is_finite(p_command->left_speed) &&
                          motion_float_is_finite(p_command->right_speed) &&
                          motion_float_is_finite(p_command->target_distance_m) &&
                          (p_command->target_distance_m > 0.0f);
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
#if S2_BENCH_DRIVE
volatile bool g_motion_bench_owner = false;   /* bench test owns the motors */
#endif

bool
motion_queue_command (motion_command_t const * p_command)
{
    motion_cmd_wire_t wire;
    bool              b_ok = false;

    if (motion_command_is_valid(p_command) && (gh_motion_cmd_mbf > 0))
    {
#if USE_BIT_PACKING
        b_ok = pack_motion_cmd(p_command, &wire);
#else
        wire = *p_command;
        b_ok = true;
#endif
        if (b_ok)
        {
            b_ok = (E_OK == tk_snd_mbf(gh_motion_cmd_mbf, &wire,
                                       (INT) sizeof(wire), TMO_POL));
        }

        /* HALT or junction TURN: do not wait for the next 20 ms cycle. */
        if (b_ok && (gh_motion_tsk > 0) &&
            ((MOTION_STOP == p_command->type) ||
             (MOTION_BRAKE == p_command->type) ||
             (MOTION_TURN == p_command->type)))
        {
            (void) tk_wup_tsk(gh_motion_tsk);
        }
    }

    return (b_ok);
}

bool
motion_send_command (motion_command_t const * p_command)
{
#if S2_BENCH_DRIVE
    if (g_motion_bench_owner)
    {
        return (false);     /* controller commands are ignored on the bench */
    }
#endif

    return (motion_queue_command(p_command));
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

/* Commands drained since the last 20 ms cycle (early wake-ups drain
   too).  A STOP/BRAKE wins over later drive commands until the cycle. */
typedef struct
{
    motion_command_t    command;
    bool                b_have;
    bool                b_stop;
} motion_inbox_t;

/*!
 * @brief Drain the command buffer into the inbox, keeping the newest
 *        command and giving STOP/BRAKE priority.
 */
static void
motion_collect (motion_inbox_t * p_inbox)
{
    motion_cmd_wire_t wire;
    motion_command_t  incoming;

    while (tk_rcv_mbf(gh_motion_cmd_mbf, &wire, TMO_POL) > 0)
    {
#if USE_BIT_PACKING
        unpack_motion_cmd(&wire, &incoming);
#else
        incoming = wire;
#endif
        if ((MOTION_STOP == incoming.type) ||
            (MOTION_BRAKE == incoming.type))
        {
            p_inbox->command = incoming;
            p_inbox->b_stop  = true;
            p_inbox->b_have  = true;
        }
        else if (!p_inbox->b_stop)
        {
            p_inbox->command = incoming;
            p_inbox->b_have  = true;
        }
        else
        {
            /* A STOP is pending: it wins until the next cycle. */
        }
    }
}

/*!
 * @brief Take the inbox for this cycle.
 *
 * @return true if a command was received since the last cycle.
 */
static bool
motion_take_commands (motion_inbox_t * p_inbox, motion_command_t * p_cmd)
{
    bool b_have;

    motion_collect(p_inbox);
    b_have = p_inbox->b_have;

    if (b_have)
    {
        *p_cmd = p_inbox->command;
    }

    p_inbox->b_have = false;
    p_inbox->b_stop = false;

    return (b_have);
}

static bool
motion_mission_running (void)
{
    T_RFLG status = {0};

    return ((gh_system_flg > 0) &&
            (E_OK == tk_ref_flg(gh_system_flg, &status)) &&
            (0U != (status.flgptn & SYSTEM_BIT_MISSION_RUN)));
}

/*!
 * @brief Early wake-up (before the 20 ms period): drain commands and,
 *        for a STOP/BRAKE, a new TURN or a cleared mission bit, zero the
 *        motors now.  For a TURN this stops the forward roll at once, so
 *        the car does not run past the junction while the pivot waits
 *        (at most 20 ms) for the next PID cycle.
 */
static void
motion_early_halt (motion_inbox_t * p_inbox)
{
    motion_collect(p_inbox);

    if (p_inbox->b_stop || !motion_mission_running() ||
        (p_inbox->b_have && (MOTION_TURN == p_inbox->command.type)))
    {
#if (!ROBOT_SIMULATION) && (!S2_ENCODER_CALIBRATION)
        motor_set(PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_DIR, 0.0f);
        motor_set(PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_DIR, 0.0f);
#endif
#if S2_HALT_TRACE
        if (p_inbox->b_stop)
        {
            tm_printf((UB *) "[NFR10] S2: motors off t_us=%u\n",
                      (unsigned int) board_micros());
        }
#endif
    }
}

static void
motion_post (int32_t left_now, int32_t right_now, float left_speed,
             float right_speed, float distance_m, uint32_t completed_turn_id,
             uint32_t sample_ms)
{
    system_event_t event = {0};
    static uint32_t dropped;
    static uint32_t last_report_ms;

    event.type                        = SYSTEM_EVENT_MOTION;
    event.timestamp_ms                = sample_ms;
    event.data.motion.left_ticks      = left_now;
    event.data.motion.right_ticks     = right_now;
    event.data.motion.left_speed_m_s  = left_speed;
    event.data.motion.right_speed_m_s = right_speed;
    event.data.motion.distance_m      = distance_m;
    event.data.motion.turn_complete   = (0U != completed_turn_id);
    event.data.motion.turn_id         = completed_turn_id;
    if (!controller_post_event(&event))
    {
        dropped++;
        if ((1U == dropped) || ((sample_ms - last_report_ms) >= 1000U))
        {
            tm_printf((UB *) "S2: motion event queue full dropped_total=%u\n",
                      (unsigned int) dropped);
            last_report_ms = sample_ms;
        }
    }
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
    else if (MOTION_MOVE_DISTANCE == p_command->type)
    {
        target = b_left_wheel ? p_command->left_speed : p_command->right_speed;
        target = rmath_clampf(target, -ROBOT_MAX_SPEED, ROBOT_MAX_SPEED);
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
 * @brief On-the-spot turns only: at least MOTOR_TURN_MIN_DUTY in the
 *        wheel's turn direction.  A pivot drags the wheels sideways, so
 *        the speed PID's output (~0.19 at 0.12 m/s) leaves a wheel
 *        stalled until the encoder watchdog stops the car.
 *
 * @param[in] duty    PID output for the wheel.
 * @param[in] target  The wheel's turn speed target (sign = direction).
 */
static float
motion_turn_duty (float duty, float target)
{
    float result = duty;

    if (target > 0.001f)
    {
        result = rmath_maxf(duty, MOTOR_TURN_MIN_DUTY);
    }
    else if (target < -0.001f)
    {
        result = rmath_minf(duty, -MOTOR_TURN_MIN_DUTY);
    }
    else
    {
        /* No target: leave it. */
    }

    return (result);
}

/*!
 * @brief Motion task (20 ms, woken by its cyclic handler).
 */
void
motion_task (INT stacd, void * p_exinf)
{
    float            interval_s;
    float const m_per_tick = (RMATH_PI * MOTOR_WHEEL_DIAMETER_M) /
                             MOTOR_ENCODER_TICKS_PER_REV;
    motion_command_t requested = {0};   /* MOTION_STOP */
    motion_pid_t     left_pid;
    motion_pid_t     right_pid;
#if ROBOT_SIMULATION
    float            left_fractional_ticks = 0.0f;
    float            right_fractional_ticks = 0.0f;
#endif
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
    uint32_t         previous_sample_us;
    uint32_t         last_encoder_report_ms = 0U;
    uint32_t         reported_left_rejected = 0U;
    uint32_t         reported_left_reversed = 0U;
    uint32_t         reported_right_reversed = 0U;
    uint32_t         reported_right_rejected = 0U;
    bool             encoder_fault_latched = false;
    bool             encoder_fault_pending = false;
    float            distance_target_m = 0.0f;  /* target distance for MOTION_MOVE_DISTANCE */
    float            distance_traveled_m = 0.0f;  /* distance accumulated for current move */
    static motion_inbox_t inbox;    /* zero-initialised: nothing pending */

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
    encoder_read(&prev_left, &prev_right);
    previous_sample_us = board_micros();

    for (;;)
    {
        motion_command_t incoming;

        /* Woken every 20 ms by the cyclic handler, and at once by a
           STOP/BRAKE (motion_queue_command). */
        (void) tk_slp_tsk(TMO_FEVR);

        {
            uint32_t const sample_us = board_micros();
            uint32_t const elapsed_us = sample_us - previous_sample_us;
            /* Early wake-up (a HALT, or queued cyclic wake-ups): stop the
               motors now if asked, but never divide new ticks by a
               fictitious 20 ms or run the PID at zero elapsed time. */
            if (elapsed_us < ROBOT_MOTION_PERIOD_MS * 1000U)
            {
                motion_early_halt(&inbox);
                continue;
            }
        }

        /* Controller-owned mission flag is the motor movement interlock.
         * Fail closed if the flag object is unavailable or cannot be read.
         */
        bool const mission_running = motion_mission_running();

        /* Drain stale commands even while stopped. They must not be replayed
         * when the next START countdown completes.
         */
        bool const have_command = motion_take_commands(&inbox, &incoming);
        if (!mission_running)
        {
            requested.type = MOTION_STOP;
            active_turn_id = 0U;
            turn_progress_deg = 0.0f;
            distance_target_m = 0.0f;
            distance_traveled_m = 0.0f;
            motion_pid_reset(&left_pid);
            motion_pid_reset(&right_pid);
        }
        else if (have_command)
        {
            float const old_left_target = motion_target_speed(&requested, true);
            float const old_right_target = motion_target_speed(&requested, false);
            bool const b_entering_turn =
                (MOTION_TURN == incoming.type) &&
                ((MOTION_TURN != requested.type) ||
                 (incoming.turn_id != requested.turn_id));
            bool const b_entering_distance_move =
                (MOTION_MOVE_DISTANCE == incoming.type) &&
                (MOTION_MOVE_DISTANCE != requested.type);

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

            if (b_entering_distance_move)
            {
                distance_target_m = incoming.target_distance_m;
                distance_traveled_m = 0.0f;
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
                (MOTION_TURN != requested.type) &&
                (MOTION_MOVE_DISTANCE != requested.type))
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

        {
            uint32_t const sample_us = board_micros();
            interval_s = (float) (sample_us - previous_sample_us) / 1000000.0f;
            previous_sample_us = sample_us;
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
        if ((now_ms - last_encoder_report_ms) >= 1000U)
        {
            uint32_t left_rejected;
            uint32_t right_rejected;
            uint32_t left_reversed;
            uint32_t right_reversed;
            encoder_read_rejected(&left_rejected, &right_rejected);
            encoder_read_reversed(&left_reversed, &right_reversed);
            if ((left_rejected != reported_left_rejected) ||
                (right_rejected != reported_right_rejected) ||
                (left_reversed != reported_left_reversed) ||
                (right_reversed != reported_right_reversed))
            {
                tm_printf((UB *) "S2: encoder short edges rejected "
                          "left=%u right=%u, reversed glitches "
                          "left=%u right=%u (totals)\n",
                          (unsigned int) left_rejected,
                          (unsigned int) right_rejected,
                          (unsigned int) left_reversed,
                          (unsigned int) right_reversed);
                reported_left_rejected = left_rejected;
                reported_right_rejected = right_rejected;
                reported_left_reversed = left_reversed;
                reported_right_reversed = right_reversed;
            }
            last_encoder_report_ms = now_ms;
        }
        {
            int32_t const left_delta = left_now - prev_left;
            int32_t const right_delta = right_now - prev_right;

            left_speed  = ((float) left_delta * m_per_tick) / interval_s;
            right_speed = ((float) right_delta * m_per_tick) / interval_s;
            total_distance += ((float) (left_delta + right_delta) * 0.5f) *
                              m_per_tick;
            
            /* Track distance for MOTION_MOVE_DISTANCE commands */
            if (MOTION_MOVE_DISTANCE == requested.type)
            {
                distance_traveled_m += ((float) (left_delta + right_delta) * 0.5f) *
                                       m_per_tick;
            }
            
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
        
        /* Check if distance-based movement is complete */
        if ((MOTION_MOVE_DISTANCE == requested.type) &&
            (distance_traveled_m >= distance_target_m))
        {
            requested.type = MOTION_STOP;
            distance_target_m = 0.0f;
            distance_traveled_m = 0.0f;
            left_target = 0.0f;
            right_target = 0.0f;
            motion_pid_reset(&left_pid);
            motion_pid_reset(&right_pid);
        }
        
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
            bool const left_stalled =
                (rmath_absf(left_target) > 0.001f) &&
                ((uint32_t) (now_ms - left_last_tick_ms) >=
                 MOTOR_ENCODER_TIMEOUT_MS);
            bool const right_stalled =
                (rmath_absf(right_target) > 0.001f) &&
                ((uint32_t) (now_ms - right_last_tick_ms) >=
                 MOTOR_ENCODER_TIMEOUT_MS);
            requested.type = MOTION_BRAKE;
            active_turn_id = 0U;
            completed_turn_id = 0U;
            encoder_fault_latched = true;
            encoder_fault_pending = true;
            fault_mask_set(FAULT_BIT_ENCODER);
            motion_pid_reset(&left_pid);
            motion_pid_reset(&right_pid);
            tm_printf((UB *) "S2: encoder stall L=%u R=%u target=%d/%d "
                      "speed=%d/%d ages=%u/%u ms\n",
                      (unsigned int) left_stalled,
                      (unsigned int) right_stalled,
                      (int) (left_target * 1000.0f),
                      (int) (right_target * 1000.0f),
                      (int) (left_speed * 1000.0f),
                      (int) (right_speed * 1000.0f),
                      (unsigned int) (now_ms - left_last_tick_ms),
                      (unsigned int) (now_ms - right_last_tick_ms));
        }

        if (encoder_fault_pending)
        {
            system_event_t fault = {0};

            fault.type         = SYSTEM_EVENT_FAULT;
            fault.timestamp_ms = now_ms;
            fault.fault_code   = SYSTEM_FAULT_ENCODER_STALL;
            if (controller_post_event(&fault))
            {
                encoder_fault_pending = false;
            }
        }

        left_out = 0.0f;
        right_out = 0.0f;
        if ((MOTION_DRIVE == requested.type) || (MOTION_TURN == requested.type) ||
            (MOTION_MOVE_DISTANCE == requested.type))
        {
            /* Straight-line correction: compare wheel speeds and adjust
             * targets to keep the car pointing straight. If one wheel is
             * going faster, reduce its target slightly.  Not in a turn:
             * there the wheels run opposite on purpose, and the
             * correction would slow both. */
            float speed_error = (MOTION_TURN == requested.type)
                                ? 0.0f : (left_speed - right_speed);
            float correction = speed_error * MOTION_STRAIGHT_GAIN;
            float left_target_corrected = rmath_clampf(left_target - correction,
                                                       -1.0f, 1.0f);
            float right_target_corrected = rmath_clampf(right_target + correction,
                                                        -1.0f, 1.0f);
            
            left_out = motion_pid_step(&left_pid, left_target_corrected, left_speed,
                                       interval_s);
            right_out = motion_pid_step(&right_pid, right_target_corrected, right_speed,
                                        interval_s);

            if (MOTION_TURN == requested.type)
            {
                left_out  = motion_turn_duty(left_out, left_target);
                right_out = motion_turn_duty(right_out, right_target);
            }
        }

        /* Recheck immediately before writing motor outputs. A STOP may
         * clear the mission bit while this iteration is running.
         */
        if (!motion_mission_running())
        {
            left_out = 0.0f;
            right_out = 0.0f;
        }

#if (!ROBOT_SIMULATION) && (!S2_ENCODER_CALIBRATION)
        motor_set(PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_DIR, left_out);
        motor_set(PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_DIR, right_out);
#else
        (void) left_out;
        (void) right_out;
#endif

        motion_post(left_now, right_now, left_speed, right_speed,
                    total_distance, completed_turn_id, now_ms);
    }
}

/*!
 * @brief Move forward a specified distance at base speed.
 *
 * @param distance_m Distance to travel in meters (must be positive).
 * @return true if command was queued.
 */
bool
motion_move_forward (float distance_m)
{
    motion_command_t cmd = {0};

    cmd.type = MOTION_MOVE_DISTANCE;
    cmd.left_speed = ROBOT_BASE_SPEED;
    cmd.right_speed = ROBOT_BASE_SPEED;
    cmd.target_distance_m = distance_m;

    return motion_send_command(&cmd);
}

/*!
 * @brief Move backward a specified distance at base speed.
 *
 * @param distance_m Distance to travel in meters (must be positive).
 * @return true if command was queued.
 */
bool
motion_move_backward (float distance_m)
{
    motion_command_t cmd = {0};

    cmd.type = MOTION_MOVE_DISTANCE;
    cmd.left_speed = -ROBOT_BASE_SPEED;
    cmd.right_speed = -ROBOT_BASE_SPEED;
    cmd.target_distance_m = distance_m;

    return motion_send_command(&cmd);
}

/*** end of file ***/
