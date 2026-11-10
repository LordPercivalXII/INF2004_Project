/** @file obstacle_profile.h
 *
 * @brief Buddy 5 stages 1-4 as pure functions (obstacle_geometry.c): the
 *        scan readings, where to fine-scan, the obstacle profile, the
 *        avoidance decision, and the stop check used while driving.
 *        No kernel calls, so all of it is unit-tested on the PC.
 */

#ifndef OBSTACLE_PROFILE_H
#define OBSTACLE_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_types.h"

/* Every reading of one scan, in angle order (13 fine angles fit). */
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

/* Stage 3: the obstacle profile.  Angles are from straight ahead, - left,
 * + right; lateral distances are sideways from the sensor, - left.
 * A clearance is the nearest reading beside the obstacle, 0 if the
 * obstacle reaches the edge of the scan on that side. */
typedef struct
{
    bool  b_detected;
    float closest_cm;           /* closest point: distance ...          */
    float closest_angle_deg;    /* ... and angle                        */
    float left_edge_deg;
    float right_edge_deg;
    float center_angle_deg;     /* location: direction of the centre    */
    float lateral_offset_cm;    /* location: centre's sideways offset   */
    float width_cm;
    float left_clearance_cm;
    float right_clearance_cm;
} obstacle_profile_t;

/* Stage 4: what to do about it. */
typedef struct
{
    obstacle_action_t action;
    float             bypass_offset_cm; /* sideways move for a turn */
} obstacle_plan_t;

void  obstacle_scan_reset(obstacle_scan_t * p_scan);
void  obstacle_scan_add(obstacle_scan_t * p_scan, uint32_t angle_deg,
                        float distance_cm);
bool  obstacle_fine_range(obstacle_scan_t const * p_coarse,
                          uint32_t * p_from_deg, uint32_t * p_to_deg);
void  obstacle_profile_build(obstacle_scan_t const * p_scan,
                             obstacle_profile_t * p_profile);
void  obstacle_plan_build(obstacle_profile_t const * p_profile,
                          obstacle_plan_t * p_plan);
bool  obstacle_watch_update(uint32_t * p_close_count, float distance_cm);
float obstacle_lead_from_wheels(float left_m_s, float right_m_s);

#endif /* OBSTACLE_PROFILE_H */

/*** end of file ***/
