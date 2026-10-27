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

#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../common/rmath.h"
#include "../controller/controller.h"

#define MOTION_SIM_TICKS        (2)
#define MOTION_INTEGRAL_LIMIT   (1.0f)

/*!
 * @brief Queue a motion command without waiting.
 *
 * @return true if queued.
 */
bool
motion_send_command (motion_command_t const * p_command)
{
    bool b_ok = false;

    if ((NULL != p_command) && (gh_motion_cmd_mbf > 0))
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
 * @return Motor output (not yet clamped to +-1).
 */
float
motion_pid_step (motion_pid_t * p_pid, float target, float measured,
                 float interval_s)
{
    float const error = target - measured;
    float       derivative;

    p_pid->integral = rmath_clampf(p_pid->integral + (error * interval_s),
                                   -MOTION_INTEGRAL_LIMIT,
                                   MOTION_INTEGRAL_LIMIT);
    derivative = (error - p_pid->previous_error) / interval_s;
    p_pid->previous_error = error;

    return (target + (MOTOR_PID_KP * error) +
            (MOTOR_PID_KI * p_pid->integral) +
            (MOTOR_PID_KD * derivative));
}

/*!
 * @brief Keep only the newest queued command.
 */
static void
motion_take_commands (motion_command_t * p_requested)
{
    motion_command_t incoming;
    INT              msg_size;

    msg_size = tk_rcv_mbf(gh_motion_cmd_mbf, &incoming, TMO_POL);

    while (msg_size > 0)
    {
        *p_requested = incoming;
        msg_size = tk_rcv_mbf(gh_motion_cmd_mbf, &incoming, TMO_POL);
    }
}

static void
motion_post (int32_t left_now, int32_t right_now, float left_speed,
             float right_speed, float distance_m)
{
    system_event_t event = {0};

    event.type                        = SYSTEM_EVENT_MOTION;
    event.timestamp_ms                = board_millis();
    event.data.motion.left_ticks      = left_now;
    event.data.motion.right_ticks     = right_now;
    event.data.motion.left_speed_m_s  = left_speed;
    event.data.motion.right_speed_m_s = right_speed;
    event.data.motion.distance_m      = distance_m;
    (void) controller_post_event(&event);
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
    motion_command_t requested = {MOTION_STOP, 0.0f, 0.0f};
    motion_pid_t     left_pid;
    motion_pid_t     right_pid;
    int32_t          prev_left  = 0;
    int32_t          prev_right = 0;
    int32_t          left_now;
    int32_t          right_now;
    float            total_distance = 0.0f;
    float            left_speed;
    float            right_speed;
    float            left_out;
    float            right_out;

    (void) stacd;
    (void) p_exinf;

    motion_pid_reset(&left_pid);
    motion_pid_reset(&right_pid);

#if !ROBOT_SIMULATION
    motor_init();

    if (!encoder_init())
    {
        tm_printf((UB *) "S2: encoder interrupt registration failed\n");
    }
#endif

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);    /* woken every 20 ms */

        motion_take_commands(&requested);

#if ROBOT_SIMULATION
        if (MOTION_DRIVE == requested.type)
        {
            encoder_sim_add(
                (requested.left_speed >= 0.0f) ? MOTION_SIM_TICKS
                                               : -MOTION_SIM_TICKS,
                (requested.right_speed >= 0.0f) ? MOTION_SIM_TICKS
                                                : -MOTION_SIM_TICKS);
        }
#endif

        encoder_read(&left_now, &right_now);
        left_speed  = ((float) (left_now - prev_left) * m_per_tick) /
                      interval_s;
        right_speed = ((float) (right_now - prev_right) * m_per_tick) /
                      interval_s;
        total_distance += ((float) ((left_now - prev_left) +
                                    (right_now - prev_right)) * 0.5f) *
                          m_per_tick;
        prev_left  = left_now;
        prev_right = right_now;

        left_out  = 0.0f;
        right_out = 0.0f;

        if (MOTION_DRIVE == requested.type)
        {
            left_out  = motion_pid_step(&left_pid, requested.left_speed,
                                        left_speed, interval_s);
            right_out = motion_pid_step(&right_pid, requested.right_speed,
                                        right_speed, interval_s);
        }
        else
        {
            motion_pid_reset(&left_pid);
            motion_pid_reset(&right_pid);
        }

#if !ROBOT_SIMULATION
        motor_set(PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_DIR, left_out);
        motor_set(PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_DIR, right_out);
#else
        (void) left_out;
        (void) right_out;
#endif

        motion_post(left_now, right_now, left_speed, right_speed,
                    total_distance);
    }
}

/*** end of file ***/
