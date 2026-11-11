/** @file obstacle_geometry.c
 *
 * @brief Buddy 5 stages 1-4 as pure functions (see obstacle_profile.h).
 *
 * Stage 1  coarse scan: 5 angles, 60..120 deg (obstacle.c takes them).
 * Stage 2  fine scan: obstacle_fine_range() says where the coarse scan
 *          hit something; obstacle.c rescans there in 5 deg steps.
 * Stage 3  obstacle_profile_build(): location, width, closest point and
 *          the clearance on each side.
 * Stage 4  obstacle_plan_build(): continue, turn left, turn right, or
 *          reverse and try again.  STOP comes from obstacle_watch_update()
 *          while driving, before there is a profile.
 */

#include "obstacle_profile.h"
#include "../common/robot_config.h"
#include "../common/rmath.h"

/* Driving: this close straight ahead, OBSTACLE_STOP_READINGS times in a
 * row, stops the car.  The car rolls a few cm further before it stops. */
#define OBSTACLE_STOP_CM        (20.0f)
#define OBSTACLE_STOP_READINGS  (3U)

/* Scanning: the nearest reading must be this close to count as an
 * obstacle (a little more than the stop distance plus one back-up). */
#define OBSTACLE_DETECT_CM      (40.0f)

/* Readings up to this far behind the nearest one are the same obstacle,
 * so a reading that wobbles by a cm cannot split it in two. */
#define OBSTACLE_BAND_CM        (10.0f)

/* The car's path: half its 15 cm width plus a 4 cm clearance each side. */
#define OBSTACLE_PATH_HALF_CM   (11.5f)

/* Closer than this the car backs up before deciding: turning on the spot
 * swings the front corners out about as far as the nose. */
#define OBSTACLE_TOO_CLOSE_CM   (15.0f)

#define OBSTACLE_OFFSET_MIN_CM  (10.0f)
#define OBSTACLE_OFFSET_MAX_CM  (30.0f) /* stays on the track mat */
#define OBSTACLE_CLEARANCE_MARGIN_CM (4.0f)

/* The beam is about 15 deg wide, so it starts to see an edge half a beam
 * before pointing at it; each reading covers half a step either side. */
#define OBSTACLE_HALF_STEP_DEG  ((float) ULTRASONIC_FINE_STEP_DEG * 0.5f)
#define OBSTACLE_HALF_BEAM_DEG  ((float) ULTRASONIC_BEAM_WIDTH_DEG * 0.5f)

/*!
 * @brief Servo angle to angle from straight ahead (- left, + right).
 */
static float
obstacle_relative_deg (uint32_t angle_deg)
{
    return ((float) angle_deg - (float) ULTRASONIC_COARSE_CENTER_DEG);
}

/*!
 * @brief Sideways distance of a point at this angle and distance.
 */
static float
obstacle_lateral_cm (float relative_deg, float distance_cm)
{
    return (distance_cm * rmath_sinf(relative_deg * RMATH_DEG_TO_RAD));
}

/*!
 * @brief Nearest reading among points [from, to].
 */
static float
obstacle_nearest_cm (obstacle_scan_t const * p_scan, uint32_t from,
                     uint32_t to)
{
    float    nearest = ULTRASONIC_MAX_DISTANCE_CM;
    uint32_t idx;

    for (idx = from; idx <= to; idx++)
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
 *        reading at the same angle replaces the first (the fine scan
 *        replaces the coarse reading there).
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
 * @brief Stage 2: where to fine-scan.  The region runs from one coarse
 *        step before the first hit to one step after the last, inside
 *        the servo's limits.
 *
 * @return false if the coarse scan hit nothing (no fine scan needed).
 */
bool
obstacle_fine_range (obstacle_scan_t const * p_coarse, uint32_t * p_from_deg,
                     uint32_t * p_to_deg)
{
    uint32_t first = ULTRASONIC_COARSE_RIGHT_DEG;
    uint32_t last  = ULTRASONIC_COARSE_LEFT_DEG;
    bool     b_hit = false;
    uint32_t idx;

    for (idx = 0U; idx < p_coarse->count; idx++)
    {
        if (p_coarse->points[idx].distance_cm <= OBSTACLE_DETECT_CM)
        {
            first = (p_coarse->points[idx].angle_deg < first)
                    ? p_coarse->points[idx].angle_deg : first;
            last  = (p_coarse->points[idx].angle_deg > last)
                    ? p_coarse->points[idx].angle_deg : last;
            b_hit = true;
        }
    }

    if (b_hit)
    {
        *p_from_deg = (first >= (ULTRASONIC_COARSE_LEFT_DEG +
                                 ULTRASONIC_FINE_MARGIN_DEG))
                      ? (first - ULTRASONIC_FINE_MARGIN_DEG)
                      : ULTRASONIC_COARSE_LEFT_DEG;
        *p_to_deg   = ((last + ULTRASONIC_FINE_MARGIN_DEG) <=
                       ULTRASONIC_COARSE_RIGHT_DEG)
                      ? (last + ULTRASONIC_FINE_MARGIN_DEG)
                      : ULTRASONIC_COARSE_RIGHT_DEG;
    }

    return (b_hit);
}

/*!
 * @brief Stage 3: the obstacle profile from one scan.
 *
 * The obstacle is the nearest reading, if it is within OBSTACLE_DETECT_CM,
 * plus every neighbouring reading within OBSTACLE_BAND_CM of it.  Its
 * edges are where that run of readings ends; the clearance on each side
 * is the nearest reading beyond the edge.
 */
void
obstacle_profile_build (obstacle_scan_t const * p_scan,
                        obstacle_profile_t * p_profile)
{
    uint32_t nearest = 0U;
    uint32_t first;
    uint32_t last;
    uint32_t idx;
    float    band_cm;
    float    left_edge;
    float    right_edge;

    *p_profile = (obstacle_profile_t) {0};
    p_profile->closest_cm         = ULTRASONIC_MAX_DISTANCE_CM;
    p_profile->left_clearance_cm  = ULTRASONIC_MAX_DISTANCE_CM;
    p_profile->right_clearance_cm = ULTRASONIC_MAX_DISTANCE_CM;

    if (0U == p_scan->count)
    {
        return;
    }

    for (idx = 1U; idx < p_scan->count; idx++)
    {
        if (p_scan->points[idx].distance_cm <
            p_scan->points[nearest].distance_cm)
        {
            nearest = idx;
        }
    }

    if (p_scan->points[nearest].distance_cm > OBSTACLE_DETECT_CM)
    {
        /* Nothing close: each side's clearance is its nearest reading. */
        for (idx = 0U; idx < p_scan->count; idx++)
        {
            if (obstacle_relative_deg(p_scan->points[idx].angle_deg) < 0.0f)
            {
                p_profile->left_clearance_cm =
                    rmath_minf(p_profile->left_clearance_cm,
                               p_scan->points[idx].distance_cm);
            }
            else if (obstacle_relative_deg(p_scan->points[idx].angle_deg) >
                     0.0f)
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

        return;
    }

    band_cm = p_scan->points[nearest].distance_cm + OBSTACLE_BAND_CM;
    first   = nearest;
    last    = nearest;

    while ((first > 0U) &&
           (p_scan->points[first - 1U].distance_cm <= band_cm))
    {
        first--;
    }

    while (((last + 1U) < p_scan->count) &&
           (p_scan->points[last + 1U].distance_cm <= band_cm))
    {
        last++;
    }

    left_edge  = obstacle_relative_deg(p_scan->points[first].angle_deg) -
                 OBSTACLE_HALF_STEP_DEG + OBSTACLE_HALF_BEAM_DEG;
    right_edge = obstacle_relative_deg(p_scan->points[last].angle_deg) +
                 OBSTACLE_HALF_STEP_DEG - OBSTACLE_HALF_BEAM_DEG;

    if (left_edge > right_edge)
    {
        /* Narrower than the beam: a single direction. */
        left_edge  = (left_edge + right_edge) * 0.5f;
        right_edge = left_edge;
    }

    p_profile->b_detected         = true;
    p_profile->closest_cm         = p_scan->points[nearest].distance_cm;
    p_profile->closest_angle_deg  =
        obstacle_relative_deg(p_scan->points[nearest].angle_deg);
    p_profile->left_edge_deg      = left_edge;
    p_profile->right_edge_deg     = right_edge;
    p_profile->center_angle_deg   = (left_edge + right_edge) * 0.5f;
    p_profile->lateral_offset_cm  =
        obstacle_lateral_cm(p_profile->center_angle_deg,
                            p_profile->closest_cm);
    p_profile->width_cm           =
        obstacle_lateral_cm(right_edge, p_profile->closest_cm) -
        obstacle_lateral_cm(left_edge, p_profile->closest_cm);
    p_profile->left_clearance_cm  = (0U == first) ? 0.0f :
        obstacle_nearest_cm(p_scan, 0U, first - 1U);
    p_profile->right_clearance_cm = ((last + 1U) == p_scan->count) ? 0.0f :
        obstacle_nearest_cm(p_scan, last + 1U, p_scan->count - 1U);
}

/*!
 * @brief Stage 4: decide what to do about a profile.
 *
 * CONTINUE        nothing there, or it is beside the car's path.
 * REVERSE_RETRY   in the path, and closer than OBSTACLE_TOO_CLOSE_CM, or
 *                 neither side has room for the required offset plus a
 *                 margin: back up and scan again.
 * TURN_LEFT/RIGHT in the path: go round the side with room (both: the
 *                 side that needs the smaller sideways move).  The offset
 *                 puts the car's side 4 cm clear of that edge (the
 *                 clearance is in OBSTACLE_PATH_HALF_CM).
 */
void
obstacle_plan_build (obstacle_profile_t const * p_profile,
                     obstacle_plan_t * p_plan)
{
    float const left_cm  = obstacle_lateral_cm(p_profile->left_edge_deg,
                                               p_profile->closest_cm);
    float const right_cm = obstacle_lateral_cm(p_profile->right_edge_deg,
                                               p_profile->closest_cm);
    float const left_offset  = OBSTACLE_PATH_HALF_CM - left_cm;
    float const right_offset = OBSTACLE_PATH_HALF_CM + right_cm;
    float const left_need_cm = rmath_clampf(left_offset,
                                            OBSTACLE_OFFSET_MIN_CM,
                                            OBSTACLE_OFFSET_MAX_CM) +
                               OBSTACLE_CLEARANCE_MARGIN_CM;
    float const right_need_cm = rmath_clampf(right_offset,
                                             OBSTACLE_OFFSET_MIN_CM,
                                             OBSTACLE_OFFSET_MAX_CM) +
                                OBSTACLE_CLEARANCE_MARGIN_CM;
    bool const  b_left_room  =
        (p_profile->left_clearance_cm > left_need_cm);
    bool const  b_right_room =
        (p_profile->right_clearance_cm > right_need_cm);
    bool        b_go_left;

    p_plan->action           = OBSTACLE_ACTION_CONTINUE;
    p_plan->bypass_offset_cm = 0.0f;

    if ((!p_profile->b_detected) || (left_cm >= OBSTACLE_PATH_HALF_CM) ||
        (right_cm <= -OBSTACLE_PATH_HALF_CM))
    {
        return;     /* nothing in the car's path */
    }

    if ((p_profile->closest_cm < OBSTACLE_TOO_CLOSE_CM) ||
        ((!b_left_room) && (!b_right_room)))
    {
        p_plan->action = OBSTACLE_ACTION_REVERSE_RETRY;
        return;
    }

    b_go_left = b_left_room &&
                ((!b_right_room) || (left_offset <= right_offset));

    p_plan->action           = b_go_left ? OBSTACLE_ACTION_TURN_LEFT
                                         : OBSTACLE_ACTION_TURN_RIGHT;
    /* OBSTACLE_PATH_HALF_CM already holds the clearance, so the offset
       puts the car's side 4 cm clear of the edge. */
    p_plan->bypass_offset_cm = rmath_clampf(b_go_left ? left_offset
                                                      : right_offset,
                                            OBSTACLE_OFFSET_MIN_CM,
                                            OBSTACLE_OFFSET_MAX_CM);
}

/*!
 * @brief While driving: count close readings straight ahead.
 *
 * A reading at the sensor's maximum is a missed echo (common at a box's
 * edge): it says nothing, so it neither counts nor resets the count.  A
 * real reading further than OBSTACLE_STOP_CM resets it, so one bad close
 * echo cannot stop the car.
 *
 * @return true once OBSTACLE_STOP_READINGS close readings came in a row.
 */
bool
obstacle_watch_update (uint32_t * p_close_count, float distance_cm)
{
    bool b_stop = false;

    if (distance_cm >= ULTRASONIC_MAX_DISTANCE_CM)
    {
        /* Missed echo: no news. */
    }
    else if (distance_cm <= OBSTACLE_STOP_CM)
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
 * @brief Kept for the controller, which calls it with every motion event.
 *        This version always looks straight ahead while driving.
 */
float
obstacle_lead_from_wheels (float left_m_s, float right_m_s)
{
    (void) left_m_s;
    (void) right_m_s;

    return (0.0f);
}

/*** end of file ***/
