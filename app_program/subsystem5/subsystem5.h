/** @file subsystem5.h
 *
 * @brief Subsystem 5 -- Buddy 5: ultrasonic obstacle scanner on a servo.
 *        Student ID: <student ID>.  Port of src/buddy5_obstacle/.
 *
 * Coarse scan at 45/90/135 deg, then a 5 deg fine scan around any hit;
 * estimates obstacle width and the clearer side, and posts
 * SYSTEM_EVENT_OBSTACLE after every scan (IF-01).
 *
 * Owns: GP15 (PWM slice 7), GP16, GP17.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "../common/robot_types.h"

#define S5_PRI_OBSTACLE     (18)    /* band 18-19 (TEAM.md) */

/* Private to subsystem 5: echo widths from the interrupt to the task. */
extern ID gh_echo_mbf;

/* Accumulated result of one scan (obstacle_geometry.c). */
typedef struct
{
    float left_clearance;
    float right_clearance;
    float minimum_hit;
    float minimum_lateral;
    float maximum_lateral;
    bool  b_detected;
    bool  b_has_hit;
} obstacle_scan_t;

void   obstacle_task(INT stacd, void * p_exinf);
void   obstacle_scan_reset(obstacle_scan_t * p_scan);
void   obstacle_scan_include(obstacle_scan_t * p_scan, uint32_t angle_deg,
                             float distance_cm);
int8_t obstacle_choose_bypass(float left_clearance, float right_clearance);
float  obstacle_estimate_width(obstacle_scan_t const * p_scan);

#endif /* SUBSYSTEM5_H */

/*** end of file ***/
