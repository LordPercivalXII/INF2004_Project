/** @file subsystem2_test.c
 *
 * @brief On-target tests for subsystem 2 (motion).  make TEST=1.
 */

#include "subsystem2.h"
#include "drv_encoder.h"
#include "../test_report.h"

#if APP_TEST

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

    /* The controller drives in simulation, so the encoders must move. */
    encoder_read(&left_before, &right_before);
    (void) tk_dly_tsk(200);
    encoder_read(&left_after, &right_after);
    TEST_CHECK("S2-U03", ((left_after != left_before) ||
                          (right_after != right_before) ||
                          (0 == ROBOT_SIMULATION)),
               "encoder counts advance while driving (simulation)");

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */

/*** end of file ***/
