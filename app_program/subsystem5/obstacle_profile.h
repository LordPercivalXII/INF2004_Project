/** @file obstacle_profile.h
 *
 * @brief Scan readings, the obstacle profile and the avoidance decision
 *        (obstacle_geometry.c).  Pure data and functions: no kernel calls.
 */

#ifndef OBSTACLE_PROFILE_H
#define OBSTACLE_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_types.h"

/* Every reading of one scan, in angle order.
 * 13 fine angles fit with room to spare. */
#define OBSTACLE_MAX_POINTS (32U)

typedef struct
{
    uint32_t angle_deg;         /* servo angle, 90 = straight ahead */
    float    distance_cm;
} obstacle_point_t;

typedef struct
{
    obstacle_point_t points[OBSTACLE_MAX_POINTS];
    uint32_t         count;
} obstacle_scan_t;

/* Obstacle profile.  Angles relative to straight ahead: - left, + right.
 * Clearances are the free space beside the obstacle (0 = blocked); with
 * no obstacle they are the nearest reading on each side of the robot. */
typedef struct
{
    bool  b_detected;
    float closest_cm;
    float closest_angle_deg;
    float left_edge_deg;
    float right_edge_deg;
    float center_angle_deg;
    float lateral_offset_cm;    /* centre's sideways offset, - left */
    float width_cm;
    float left_clearance_cm;
    float right_clearance_cm;
} obstacle_profile_t;

/* What to do about a profile (stage 4). */
typedef struct
{
    obstacle_action_t action;
    float             bypass_offset_cm; /* sideways move for a turn action */
} obstacle_plan_t;

void   obstacle_scan_reset(obstacle_scan_t * p_scan);
void   obstacle_scan_add(obstacle_scan_t * p_scan, uint32_t angle_deg,
                         float distance_cm);
void   obstacle_profile_build(obstacle_scan_t const * p_scan,
                              obstacle_profile_t * p_profile);
int8_t obstacle_choose_bypass(float left_clearance, float right_clearance);
bool   obstacle_watch_update(uint32_t * p_close_count, float distance_cm);
void   obstacle_plan_build(obstacle_profile_t const * p_profile,
                           obstacle_plan_t * p_plan);

#endif /* OBSTACLE_PROFILE_H */

/*** end of file ***/
