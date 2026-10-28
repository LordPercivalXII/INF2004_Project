/** @file subsystem2_test.c
 *
 * @brief On-target tests for subsystem 2 (motion).  make TEST=1.
 */

#include "subsystem2.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "../common/robot_config.h"
#include "../test_report.h"

#if APP_TEST

/* Set to 1, build with SIM=0, then turn a wheel by hand to count ticks/rev. */
#ifndef S2_ENCODER_CALIBRATION
#define S2_ENCODER_CALIBRATION (1)
#endif

#if S2_BENCH_DRIVE
extern volatile bool g_motion_bench_owner;

/* Bench run through the real motion task (speed PID + encoders), no IMU. */
static void
s2_bench_send (motion_command_type_t type, float speed, float angle_deg,
               uint32_t turn_id, int32_t duration_ms)
{
    motion_command_t command = {0};
    int32_t          elapsed;

    command.type           = type;
    command.left_speed     = speed;
    command.right_speed    = speed;
    command.turn_angle_deg = angle_deg;
    command.turn_id        = turn_id;

    if (MOTION_TURN == type)
    {
        /* A turn is sent once; repeating it would start a new turn. */
        (void) tk_snd_mbf(gh_motion_cmd_mbf, &command,
                          (INT) sizeof(command), TMO_POL);
        (void) tk_dly_tsk(duration_ms);
        return;
    }

    for (elapsed = 0; elapsed < duration_ms; elapsed += 100)
    {
        (void) tk_snd_mbf(gh_motion_cmd_mbf, &command,
                          (INT) sizeof(command), TMO_POL);
        (void) tk_dly_tsk(100);
    }
}

static void
s2_bench_report (char const * p_name, int32_t left_before,
                 int32_t right_before)
{
    int32_t left_now;
    int32_t right_now;
    int32_t const mm_per_rev = (int32_t) (3.14159f * MOTOR_WHEEL_DIAMETER_M *
                                          1000.0f);

    (void) tk_dly_tsk(500);
    encoder_read(&left_now, &right_now);
    tm_printf((UB *) "[BENCH] %s: left_ticks=%d right_ticks=%d "
              "left_mm=%d right_mm=%d\n", p_name,
              (int) (left_now - left_before), (int) (right_now - right_before),
              (int) (((left_now - left_before) * mm_per_rev) /
                     (int32_t) MOTOR_ENCODER_TICKS_PER_REV),
              (int) (((right_now - right_before) * mm_per_rev) /
                     (int32_t) MOTOR_ENCODER_TICKS_PER_REV));
}

static void
s2_bench_drive (void)
{
    int32_t left_before;
    int32_t right_before;

    g_motion_bench_owner = true;
    tm_printf((UB *) "[BENCH] start in 5 s: robot on the floor, clear 1.5 m\n");
    (void) tk_dly_tsk(5000);

    encoder_read(&left_before, &right_before);
    tm_printf((UB *) "[BENCH] straight 0.20 m/s for 5 s (about 1 m)\n");
    s2_bench_send(MOTION_DRIVE, 0.20f, 0.0f, 0U, 5000);
    s2_bench_send(MOTION_STOP, 0.0f, 0.0f, 0U, 500);
    s2_bench_report("straight", left_before, right_before);
    (void) tk_dly_tsk(2000);

    encoder_read(&left_before, &right_before);
    tm_printf((UB *) "[BENCH] turn +90 deg\n");
    s2_bench_send(MOTION_TURN, 0.0f, 90.0f, 1U, 3000);
    s2_bench_send(MOTION_STOP, 0.0f, 0.0f, 0U, 500);
    s2_bench_report("turn +90", left_before, right_before);
    (void) tk_dly_tsk(2000);

    encoder_read(&left_before, &right_before);
    tm_printf((UB *) "[BENCH] turn 180 deg\n");
    s2_bench_send(MOTION_TURN, 0.0f, 180.0f, 2U, 4000);
    s2_bench_send(MOTION_STOP, 0.0f, 0.0f, 0U, 500);
    s2_bench_report("turn 180", left_before, right_before);

    tm_printf((UB *) "[BENCH] done\n");
}
#endif /* S2_BENCH_DRIVE */

void
subsystem2_test (void)
{
    motion_pid_t pid;
    float        output;
    int32_t      left_before;
    int32_t      right_before;
    int32_t      left_after;
    int32_t      right_after;

    TEST_INIT();

    motion_pid_reset(&pid);
    output = motion_pid_step(&pid, 0.3f, 0.3f, 0.02f);
    TEST_CHECK("S2-U01", ((output > 0.299f) && (output < 0.301f)),
               "PID output = target when there is no error");

    motion_pid_reset(&pid);
    output = motion_pid_step(&pid, 0.3f, 0.0f, 0.02f);
    TEST_CHECK("S2-U02", (output > 0.3f),
               "PID pushes harder when the wheel is too slow");

    motion_pid_reset(&pid);
    output = motion_pid_step(&pid, 2.0f, 0.0f, 0.02f);
    TEST_CHECK("S2-U05", ((output == 1.0f) && (pid.integral == 0.0f)),
               "PID clamps output and prevents integral windup");

    /* The controller drives in simulation, so the encoders must move. */
    encoder_read(&left_before, &right_before);
    (void) tk_dly_tsk(200);
    encoder_read(&left_after, &right_after);
    TEST_CHECK("S2-U03", ((left_after != left_before) ||
                          (right_after != right_before) ||
                          (0 == ROBOT_SIMULATION)),
               "encoder counts advance while driving (simulation)");

    TEST_SUMMARY("subsystem2");

#if S2_ENCODER_CALIBRATION && S2_MOTOR_DIR_TEST
    /* Wheels must be off the ground.  Each wheel runs alone at 30% power. */
    (void) tk_dly_tsk(3000);
    for (int32_t wheel = 0; wheel < 2; wheel++)
    {
        uint32_t const pwm = (0 == wheel) ? PIN_MOTOR_LEFT_PWM : PIN_MOTOR_RIGHT_PWM;
        uint32_t const dir = (0 == wheel) ? PIN_MOTOR_LEFT_DIR : PIN_MOTOR_RIGHT_DIR;

        encoder_read(&left_before, &right_before);
        tm_printf((UB *) "[DIR] %s wheel: FORWARD command for 1.5 s\n",
                  (0 == wheel) ? "LEFT" : "RIGHT");
        motor_set(pwm, dir, 0.3f);
        (void) tk_dly_tsk(1500);
        motor_set(pwm, dir, 0.0f);
        (void) tk_dly_tsk(300);
        encoder_read(&left_after, &right_after);
        tm_printf((UB *) "[DIR] encoder change: left=%d right=%d\n",
                  (int) (left_after - left_before),
                  (int) (right_after - right_before));
        (void) tk_dly_tsk(2000);
    }
    encoder_read(&left_before, &right_before);
    tm_printf((UB *) "[DIR] BOTH wheels: FORWARD together for 4 s\n");
    motor_set(PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_DIR, 0.3f);
    motor_set(PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_DIR, 0.3f);
    (void) tk_dly_tsk(4000);
    motor_set(PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_DIR, 0.0f);
    motor_set(PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_DIR, 0.0f);
    (void) tk_dly_tsk(300);
    encoder_read(&left_after, &right_after);
    tm_printf((UB *) "[DIR] both encoder change: left=%d right=%d\n",
              (int) (left_after - left_before),
              (int) (right_after - right_before));
    tm_printf((UB *) "[DIR] done\n");
#endif

#if S2_BENCH_DRIVE
    s2_bench_drive();
#endif

#if S2_ENCODER_CALIBRATION
    tm_printf((UB *) "[CAL] turn a wheel by hand; ticks print every 500 ms\n");
    for (;;)
    {
        encoder_read(&left_after, &right_after);
        tm_printf((UB *) "[CAL] left=%d right=%d\n",
                  (int) left_after, (int) right_after);
        (void) tk_dly_tsk(500);
    }
#endif
}

#endif /* APP_TEST */

/*** end of file ***/
