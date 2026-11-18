/** @file subsystem2.h
 *
 * @brief Subsystem 2 -- Buddy 2: motion (motors, wheel encoders, speed
 *        PID, encoder odometry and closed-loop turns.  Student ID:
 *        <student ID>.  Port of src/buddy2_motion/.
 *
 * Public interface (TEAM.md section 5):
 *   IF-03  controller -> S2: motion commands, message buffer
 *          gh_motion_cmd_mbf (motion_command_t), sent with
 *          motion_send_command().
 *   IF-01  S2 -> controller: SYSTEM_EVENT_MOTION every 20 ms.
 *
 * IF-03 travels packed (motion_cmd_packed_t, 4 bytes) when
 * USE_BIT_PACKING is set.  A STOP or BRAKE also wakes the motion task
 * at once (tk_wup_tsk), so the motors stop in well under 1 ms instead
 * of at the next 20 ms cycle (Tribunal 1, NFR10).
 *
 * Owns: PWM slices 4 and 5 (GP8, GP10), GP9, GP11, GP12-14, GP19.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "../common/robot_types.h"

#define S2_PRI_MOTION       (10)    /* band 10-11: highest (TEAM.md) */

extern ID gh_motion_cmd_mbf;
extern ID gh_motion_tsk;        /* private: woken early by STOP/BRAKE */

/* Print when an early wake-up has zeroed the motor outputs (NFR10). */
#ifndef S2_HALT_TRACE
#define S2_HALT_TRACE       (1)
#endif

bool motion_send_command(motion_command_t const * p_command);
/* MOTION_TURN commands require a nonzero turn_id; completion is reported in
 * the matching SYSTEM_EVENT_MOTION event. */

/* Distance-based motion APIs */
bool motion_move_forward(float distance_m);
bool motion_move_backward(float distance_m);

/* Private to subsystem 2 (also used by subsystem2_test.c): queue a
   command, bypassing the bench-test ownership check. */
bool motion_queue_command(motion_command_t const * p_command);

/* Private to subsystem 2. */
typedef struct
{
    float integral;
    float previous_error;
} motion_pid_t;

void  motion_task(INT stacd, void * p_exinf);
void  motion_pid_reset(motion_pid_t * p_pid);
float motion_pid_step(motion_pid_t * p_pid, float target, float measured,
                      float interval_s);

#endif /* SUBSYSTEM2_H */

/*** end of file ***/
