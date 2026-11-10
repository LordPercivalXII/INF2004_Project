/** @file obstacle_geometry.c
 *
 * @brief Scan bookkeeping and the obstacle profile: location, width,
 *        closest point, clearances, bypass side.
 *        Pure functions (unit-tested in subsystem5_test.c).
 *
 * Angles in the profile are relative to straight ahead: negative = left,
 * positive = right.  Servo angles below 90 deg point left.
 */

#include "obstacle_profile.h"
#include "../common/robot_config.h"
#include "../common/rmath.h"

#define OBSTACLE_HALF_STEP_DEG  ((float) ULTRASONIC_FINE_STEP_DEG * 0.5f)
#define OBSTACLE_HALF_BEAM_DEG  ((float) ULTRASONIC_BEAM_WIDTH_DEG * 0.5f)
#define OBSTACLE_PATH_HALF_CM   (9.5f)  /* half the 13 cm car + 3 margin */
#define OBSTACLE_TOO_CLOSE_CM   (12.0f) /* no room to spin */
#define OBSTACLE_STOP_CM        (20.0f) /* car still rolls after the check */
#define OBSTACLE_STOP_READINGS  (3U)    /* close readings in a row */
#define OBSTACLE_ROOMY_CM       (50.0f)
#define OBSTACLE_OFFSET_MIN_CM  (10.0f)
#define OBSTACLE_OFFSET_MAX_CM  (30.0f) /* stays on the tile mat */
#define OBSTACLE_LEAD_STEP_DEG  (5U)    /* look-into-turn aiming steps */
#define OBSTACLE_LOOK_AHEAD_CM  (25.0f) /* aim at the car's path this far
                                           ahead: the stop distance plus
                                           the 5 cm the nose is ahead of
                                           the sensor */
#define OBSTACLE_SPEED_MIN_M_S  (0.03f) /* slower: treated as a spin      */
#define OBSTACLE_HARD_TURN_LEAD (0.99f) /* lead this high: turning hard   */
#define OBSTACLE_HARD_TURN_CM   (12.0f) /* one reading this close stops a
                                           hard turn                      */

static bool
obstacle_is_hit (obstacle_point_t const * p_point)
{
    return (p_point->distance_cm <= ULTRASONIC_THRESHOLD_CM);
}

static float
obstacle_relative_deg (uint32_t angle_deg)
{
    return ((float) angle_deg - (float) ULTRASONIC_COARSE_CENTER_DEG);
}

/*!
 * @brief Sideways offset of a point: distance * sin(relative angle).
 */
static float
obstacle_lateral_cm (float relative_deg, float distance_cm)
{
    return (distance_cm * rmath_sinf(relative_deg * RMATH_DEG_TO_RAD));
}

/*!
 * @brief Nearest reading among points[first..last].
 */
static float
obstacle_nearest_cm (obstacle_scan_t const * p_scan, uint32_t first,
                     uint32_t last)
{
    float    nearest = ULTRASONIC_MAX_DISTANCE_CM;
    uint32_t idx;

    for (idx = first; idx <= last; idx++)
    {
        nearest = rmath_minf(nearest, p_scan->points[idx].distance_cm);
    }

    return (nearest);
}

void
obstacle_scan_reset (obstacle_scan_t * p_scan)
{
    p_scan->count = 0U;
}

/*!
 * @brief Add one reading, keeping the points in angle order.  A second
 *        reading at the same angle replaces the first.
 */
void
obstacle_scan_add (obstacle_scan_t * p_scan, uint32_t angle_deg,
                   float distance_cm)
{
    uint32_t idx = 0U;
    uint32_t move;

    while ((idx < p_scan->count) &&
           (p_scan->points[idx].angle_deg < angle_deg))
    {
        idx++;
    }

    if ((idx < p_scan->count) && (p_scan->points[idx].angle_deg == angle_deg))
    {
        p_scan->points[idx].distance_cm = distance_cm;
    }
    else if (p_scan->count < OBSTACLE_MAX_POINTS)
    {
        for (move = p_scan->count; move > idx; move--)
        {
            p_scan->points[move] = p_scan->points[move - 1U];
        }

        p_scan->points[idx].angle_deg   = angle_deg;
        p_scan->points[idx].distance_cm = distance_cm;
        p_scan->count++;
    }
    else
    {
        /* Full: drop the reading. */
    }
}

/*!
 * @brief Add a repeat reading: an angle already read keeps the nearer of
 *        the two.  With two sweeps, an echo missed once (a far reading at
 *        the box's edge) no longer makes the box narrower, so the width is
 *        the same from scan to scan.
 */
void
obstacle_scan_add_nearer (obstacle_scan_t * p_scan, uint32_t angle_deg,
                          float distance_cm)
{
    uint32_t idx;

    for (idx = 0U; idx < p_scan->count; idx++)
    {
        if (p_scan->points[idx].angle_deg == angle_deg)
        {
            distance_cm = rmath_minf(distance_cm,
                                     p_scan->points[idx].distance_cm);
        }
    }

    obstacle_scan_add(p_scan, angle_deg, distance_cm);
}

/*!
 * @brief Build the obstacle profile from one scan.
 *
 * The obstacle is the unbroken run of hits around the nearest hit.  Each
 * side's clearance is the nearest reading beside that run; if the run
 * reaches the end of the scan, that side cannot be seen and counts as
 * blocked (0).  The beam sees an edge half a beam width early, so the
 * run is trimmed by that much on each side before measuring the width.
 */
void
obstacle_profile_build (obstacle_scan_t const * p_scan,
                        obstacle_profile_t * p_profile)
{
    uint32_t closest = 0U;
    uint32_t first;
    uint32_t last;
    uint32_t idx;
    float    left_edge;
    float    right_edge;
    float    angle;

    *p_profile = (obstacle_profile_t) {0};
    p_profile->closest_cm         = ULTRASONIC_MAX_DISTANCE_CM;
    p_profile->left_clearance_cm  = ULTRASONIC_MAX_DISTANCE_CM;
    p_profile->right_clearance_cm = ULTRASONIC_MAX_DISTANCE_CM;

    for (idx = 0U; idx < p_scan->count; idx++)
    {
        if (obstacle_is_hit(&p_scan->points[idx]) &&
            ((!p_profile->b_detected) ||
             (p_scan->points[idx].distance_cm <
              p_scan->points[closest].distance_cm)))
        {
            closest = idx;
            p_profile->b_detected = true;
        }
    }

    if (!p_profile->b_detected)
    {
        /* Nothing in range: each side's clearance is its nearest reading. */
        for (idx = 0U; idx < p_scan->count; idx++)
        {
            angle = obstacle_relative_deg(p_scan->points[idx].angle_deg);

            if (angle < 0.0f)
            {
                p_profile->left_clearance_cm =
                    rmath_minf(p_profile->left_clearance_cm,
                               p_scan->points[idx].distance_cm);
            }
            else if (angle > 0.0f)
            {
                p_profile->right_clearance_cm =
                    rmath_minf(p_profile->right_clearance_cm,
                               p_scan->points[idx].distance_cm);
            }
            else
            {
                /* Straight ahead belongs to neither side. */
            }
        }
    }
    else
    {
        first = closest;
        last  = closest;

        while ((first > 0U) && obstacle_is_hit(&p_scan->points[first - 1U]))
        {
            first--;
        }

        while (((last + 1U) < p_scan->count) &&
               obstacle_is_hit(&p_scan->points[last + 1U]))
        {
            last++;
        }

        p_profile->left_clearance_cm  = (0U == first) ? 0.0f :
            obstacle_nearest_cm(p_scan, 0U, first - 1U);
        p_profile->right_clearance_cm = ((last + 1U) == p_scan->count) ? 0.0f :
            obstacle_nearest_cm(p_scan, last + 1U, p_scan->count - 1U);

        left_edge  = obstacle_relative_deg(p_scan->points[first].angle_deg) -
                     OBSTACLE_HALF_STEP_DEG + OBSTACLE_HALF_BEAM_DEG;
        right_edge = obstacle_relative_deg(p_scan->points[last].angle_deg) +
                     OBSTACLE_HALF_STEP_DEG - OBSTACLE_HALF_BEAM_DEG;

        if (left_edge > right_edge)
        {
            /* Narrower than the beam can resolve. */
            left_edge  = (left_edge + right_edge) * 0.5f;
            right_edge = left_edge;
        }

        p_profile->closest_cm        = p_scan->points[closest].distance_cm;
        p_profile->closest_angle_deg =
            obstacle_relative_deg(p_scan->points[closest].angle_deg);
        p_profile->left_edge_deg     = left_edge;
        p_profile->right_edge_deg    = right_edge;
        p_profile->center_angle_deg  = (left_edge + right_edge) * 0.5f;
        p_profile->lateral_offset_cm =
            obstacle_lateral_cm(p_profile->center_angle_deg,
                                p_profile->closest_cm);
        p_profile->width_cm          = rmath_maxf(0.0f,
            obstacle_lateral_cm(right_edge,
                                p_scan->points[last].distance_cm) -
            obstacle_lateral_cm(left_edge,
                                p_scan->points[first].distance_cm));
    }
}

/*!
 * @brief Pick the side to pass on.
 *
 * @return -1 = left, +1 = right, 0 = both blocked.
 */
int8_t
obstacle_choose_bypass (float left_clearance, float right_clearance)
{
    bool const b_left_clear  = (left_clearance > ULTRASONIC_THRESHOLD_CM);
    bool const b_right_clear = (right_clearance > ULTRASONIC_THRESHOLD_CM);
    int8_t     direction     = 0;

    if ((!b_left_clear) && (!b_right_clear))
    {
        direction = 0;
    }
    else if (!b_left_clear)
    {
        direction = 1;
    }
    else if (!b_right_clear)
    {
        direction = -1;
    }
    else
    {
        direction = (left_clearance >= right_clearance) ? -1 : 1;
    }

    return (direction);
}

/*!
 * @brief Watch the straight-ahead readings while driving.  Several close
 *        readings in a row are needed, so one bad echo cannot stop the car.
 *
 * @param[in,out] p_close_count  Close readings so far; start at 0.
 *
 * @return true when the car should stop.
 */
bool
obstacle_watch_update (uint32_t * p_close_count, float distance_cm)
{
    bool b_stop = false;

    if (distance_cm <= OBSTACLE_STOP_CM)
    {
        (*p_close_count)++;

        if (*p_close_count >= OBSTACLE_STOP_READINGS)
        {
            *p_close_count = 0U;
            b_stop         = true;
        }
    }
    else
    {
        *p_close_count = 0U;
    }

    return (b_stop);
}

/*!
 * @brief Servo angle that looks the way the car is turning.
 *
 * @param[in] lead  -1 (turning hard left) .. +1 (hard right), 0 straight.
 *
 * @return 60..120 deg in OBSTACLE_LEAD_STEP_DEG steps, so small steering
 *         changes do not keep the servo moving.
 */
uint32_t
obstacle_lead_angle (float lead)
{
    float const half_deg = (float) (ULTRASONIC_COARSE_RIGHT_DEG -
                                    ULTRASONIC_COARSE_CENTER_DEG);
    float const steps = (rmath_clampf(lead, -1.0f, 1.0f) * half_deg) /
                        (float) OBSTACLE_LEAD_STEP_DEG;
    int32_t const whole = (int32_t) ((steps >= 0.0f) ? (steps + 0.5f)
                                                     : (steps - 0.5f));

    return ((uint32_t) ((int32_t) ULTRASONIC_COARSE_CENTER_DEG +
                        (whole * (int32_t) OBSTACLE_LEAD_STEP_DEG)));
}

/*!
 * @brief How far ahead a reading taken at a servo angle is: the part of
 *        it along the car's heading.  The car is turning that way, so the
 *        reading counts as in its path.
 */
float
obstacle_ahead_cm (uint32_t angle_deg, float distance_cm)
{
    return (distance_cm *
            rmath_cosf(obstacle_relative_deg(angle_deg) * RMATH_DEG_TO_RAD));
}

/*!
 * @brief Which way to look, from the measured wheel speeds: toward the
 *        point OBSTACLE_LOOK_AHEAD_CM along the curve the car is on.
 *
 * On a circle, a point a distance L along it is L / (2 R) to the side of
 * the heading, with curvature 1 / R = turn rate / speed.  A spin on the
 * spot (speed near 0) looks fully toward the turn.
 *
 * @return -1 (look fully left) .. +1 (fully right), as obstacle_lead_angle().
 */
float
obstacle_lead_from_wheels (float left_m_s, float right_m_s)
{
    float const speed_m_s = rmath_maxf(rmath_absf(0.5f * (left_m_s +
                                                          right_m_s)),
                                       OBSTACLE_SPEED_MIN_M_S);
    float const turn_rad_s = (right_m_s - left_m_s) / MOTOR_TRACK_WIDTH_M;
    float const look_deg = 0.5f * (OBSTACLE_LOOK_AHEAD_CM * 0.01f) *
                           (turn_rad_s / speed_m_s) * RMATH_RAD_TO_DEG;
    float const max_deg = (float) (ULTRASONIC_COARSE_RIGHT_DEG -
                                   ULTRASONIC_COARSE_CENTER_DEG);

    /* Turning left (right wheel faster) looks left: lead below 0. */
    return (rmath_clampf(-look_deg / max_deg, -1.0f, 1.0f));
}

/*!
 * @brief While turning hard (lead at full: a tight curve, a junction spin,
 *        a line search), the beam sweeps past an obstacle too fast for
 *        obstacle_watch_update()'s readings in a row: one reading this
 *        close stops the car at once.
 *
 * @param[in] lead         As obstacle_lead_angle().
 * @param[in] distance_cm  Distance ahead (obstacle_ahead_cm()).
 */
bool
obstacle_hard_turn_stop (float lead, float distance_cm)
{
    return ((rmath_absf(lead) >= OBSTACLE_HARD_TURN_LEAD) &&
            (distance_cm <= OBSTACLE_HARD_TURN_CM));
}

/*!
 * @brief Decide what to do about a profile (stage 4).
 *
 * CONTINUE        nothing there, or it is entirely beside the car's path.
 * REVERSE_RETRY   in the path, and too close to spin (under 12 cm).
 * TURN_LEFT/RIGHT in the path: the side with room; if neither edge is
 *                 seen, the side with more room, at the widest offset.
 *
 * STOP is never chosen here: obstacle.c sends it while driving, when
 * obstacle_watch_update() says so, before the profile exists.
 */
void
obstacle_plan_build (obstacle_profile_t const * p_profile,
                     obstacle_plan_t * p_plan)
{
    float const left_cm  = obstacle_lateral_cm(p_profile->left_edge_deg,
                                               p_profile->closest_cm);
    float const right_cm = obstacle_lateral_cm(p_profile->right_edge_deg,
                                               p_profile->closest_cm);
    int8_t      side;

    p_plan->action           = OBSTACLE_ACTION_CONTINUE;
    p_plan->bypass_offset_cm = 0.0f;

    if (p_profile->b_detected && (left_cm < OBSTACLE_PATH_HALF_CM) &&
        (right_cm > -OBSTACLE_PATH_HALF_CM))
    {
        side = obstacle_choose_bypass(p_profile->left_clearance_cm,
                                      p_profile->right_clearance_cm);

        /* Plenty of room both ways: take the shorter way round, which
           strays least from the line. */
        if ((p_profile->left_clearance_cm >= OBSTACLE_ROOMY_CM) &&
            (p_profile->right_clearance_cm >= OBSTACLE_ROOMY_CM))
        {
            side = ((-left_cm) <= right_cm) ? -1 : 1;
        }

        if (p_profile->closest_cm < OBSTACLE_TOO_CLOSE_CM)
        {
            p_plan->action = OBSTACLE_ACTION_REVERSE_RETRY;
        }
        else if (0 == side)
        {
            /* Neither edge seen: the box is wider than the +/-30 deg scan
               covers.  Backing up would not help, so go round the side
               with more room (else away from its nearest point) as far
               out as allowed. */
            if (p_profile->left_clearance_cm != p_profile->right_clearance_cm)
            {
                side = (p_profile->left_clearance_cm >
                        p_profile->right_clearance_cm) ? -1 : 1;
            }
            else
            {
                side = (p_profile->closest_angle_deg <= 0.0f) ? 1 : -1;
            }

            p_plan->action = (side < 0) ? OBSTACLE_ACTION_TURN_LEFT
                                        : OBSTACLE_ACTION_TURN_RIGHT;
            p_plan->bypass_offset_cm = OBSTACLE_OFFSET_MAX_CM;
        }
        else
        {
            p_plan->action = (side < 0) ? OBSTACLE_ACTION_TURN_LEFT
                                        : OBSTACLE_ACTION_TURN_RIGHT;
            p_plan->bypass_offset_cm =
                rmath_clampf(((side < 0) ? -left_cm : right_cm) +
                             OBSTACLE_PATH_HALF_CM,
                             OBSTACLE_OFFSET_MIN_CM, OBSTACLE_OFFSET_MAX_CM);
        }
    }
}

/*** end of file ***/
