/** @file subsystem5.h
 *
 * @brief Subsystem 5 -- Buddy 5: adaptive servo ultrasonic scanning and
 *        obstacle profiling.  Student ID: <student ID>.
 *
 * Driving: the sensor looks straight ahead; 3 close readings in a row
 * (20 cm) post OBSTACLE_ACTION_STOP.  Stopped: coarse scan (stage 1),
 * fine scan over the obstacle (stage 2), obstacle profile (stage 3) and
 * decision (stage 4), posted as SYSTEM_EVENT_OBSTACLE (IF-01).  The
 * controller, which owns the motors, then runs avoidance.c: the bypass
 * and the line recovery (stage 5), and hands back to line following.
 *
 * Junction routing after avoidance: when no barcode is present at a
 * junction, Buddy 3's route_take_junction_action() defaults to STRAIGHT.
 * To reacquire the original line (Tribunal 1 requirement), the controller
 * should check avoid_junction_hint(p_avoid) after avoidance completes
 * (AVOID_DONE). This function returns a barcode_command_t value that
 * indicates which way to turn to continue the original path:
 *   - BARCODE_CMD_LEFT/RIGHT: branch that continues away from obstacle
 *   - BARCODE_CMD_NONE: no specific hint (no avoidance was done)
 * Alternatively, query avoid_last_direction(p_avoid) for the raw direction
 * the car took to bypass (-1 = LEFT, 0 = none, +1 = RIGHT).
 *
 * Files:
 *   obstacle.c           the task: driving watch, scans, events
 *   obstacle_geometry.c  stages 1-4 as pure functions
 *   avoidance.c          the bypass and line recovery
 *   drv_servo.c, drv_ultrasonic.c  the hardware
 *
 * Owns: GP15 (servo PWM, slice 7), GP16 (echo), GP17 (trigger).
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "../common/robot_types.h"
#include "obstacle_profile.h"
#include "avoidance.h"

#define S5_PRI_OBSTACLE     (18)    /* band 18-19 (TEAM.md) */

/* Controller -> S5: where to point the scanner while avoiding (read only
 * while SYSTEM_BIT_OBSTACLE is set): scan while stopped, else ahead. */
void   obstacle_set_look(avoid_look_t look);

/* Controller -> S5: which way the car is turning.  Kept so controller.c
 * need not change; this version always looks straight ahead. */
void   obstacle_set_lead(float lead);

/* Private to subsystem 5: echo widths from the interrupt to the task. */
extern ID gh_echo_mbf;

void   obstacle_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM5_H */

/*** end of file ***/
