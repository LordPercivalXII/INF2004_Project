/** @file subsystem2.h
 *
 * @brief Subsystem 2 -- Buddy 2: motion (motors, wheel encoders, speed
 *        PID).  Student ID: <student ID>.  Port of src/buddy2_motion/.
 *
 * Public interface (TEAM.md section 5):
 *   IF-03  controller -> S2: motion commands, message buffer
 *          gh_motion_cmd_mbf (motion_command_t), sent with
 *          motion_send_command().
 *   IF-01  S2 -> controller: SYSTEM_EVENT_MOTION every 20 ms.
 *
 * Owns: PWM slices 4 and 5 (GP8, GP10), GP9, GP11, GP12-14, GP19.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "../common/robot_types.h"

#define S2_PRI_MOTION       (10)    /* band 10-11: highest (TEAM.md) */

extern ID gh_motion_cmd_mbf;

bool motion_send_command(motion_command_t const * p_command);

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
