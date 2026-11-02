/** @file subsystem5.h
 *
 * @brief Subsystem 5 -- Buddy 5: ultrasonic obstacle scanner on a servo.
 *        Student ID: <student ID>.  Port of src/buddy5_obstacle/.
 *
 * While driving, the sensor faces straight ahead with the servo still; 3
 * readings in a row within 35 cm post OBSTACLE_ACTION_STOP at once, because
 * a sweep takes too long at driving speed.  Once stopped: coarse scan at
 * 60/75/90/105/120 deg, then a 5 deg fine scan around any hit; builds an
 * obstacle profile (location, width, closest point, clearances), decides
 * what to do (stage 4) and posts SYSTEM_EVENT_OBSTACLE after every scan
 * (IF-01).  avoidance.c holds the manoeuvre and line recovery (stage 5); the
 * controller runs it, because the controller owns the motors.
 *
 * Owns: GP15 (PWM slice 7), GP16, GP17.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "../common/robot_types.h"
#include "obstacle_profile.h"

#define S5_PRI_OBSTACLE     (18)    /* band 18-19 (TEAM.md) */

/* Private to subsystem 5: echo widths from the interrupt to the task. */
extern ID gh_echo_mbf;

void   obstacle_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM5_H */

/*** end of file ***/
