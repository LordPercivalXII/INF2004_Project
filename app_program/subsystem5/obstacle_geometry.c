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
#define OBSTACLE_PATH_HALF_CM   (10.5f) /* half the car (7.5) + 3 margin */
#define OBSTACLE_TOO_CLOSE_CM   (12.0f) /* no room to spin */
#define OBSTACLE_STOP_CM        (25.0f) /* car still rolls after the check */
#define OBSTACLE_STOP_READINGS  (3U)    /* close readings in a row */
#define OBSTACLE_ROOMY_CM       (50.0f)
#define OBSTACLE_OFFSET_MIN_CM  (10.0f)
#define OBSTACLE_OFFSET_MAX_CM  (30.0f) /* stays on the tile mat */

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
 * @brief Decide what to do about a profile (stage 4).
 *
 * CONTINUE        nothing there, or it is entirely beside the car's path.
 * REVERSE_RETRY   in the path, and too close to spin or no side is clear.
 * TURN_LEFT/RIGHT in the path with room on that side.
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

        if ((0 == side) || (p_profile->closest_cm < OBSTACLE_TOO_CLOSE_CM))
        {
            p_plan->action = OBSTACLE_ACTION_REVERSE_RETRY;
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
