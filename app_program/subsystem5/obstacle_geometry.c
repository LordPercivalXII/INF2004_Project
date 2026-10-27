/** @file obstacle_geometry.c
 *
 * @brief Scan bookkeeping: clearances, obstacle width, bypass side.
 *        Pure functions (unit-tested in subsystem5_test.c).
 */

#include "subsystem5.h"
#include "../common/robot_config.h"
#include "../common/rmath.h"

void
obstacle_scan_reset (obstacle_scan_t * p_scan)
{
    p_scan->left_clearance  = ULTRASONIC_MAX_DISTANCE_CM;
    p_scan->right_clearance = ULTRASONIC_MAX_DISTANCE_CM;
    p_scan->minimum_hit     = ULTRASONIC_MAX_DISTANCE_CM;
    p_scan->minimum_lateral = 0.0f;
    p_scan->maximum_lateral = 0.0f;
    p_scan->b_detected      = false;
    p_scan->b_has_hit       = false;
}

/*!
 * @brief Add one measurement at a servo angle to the scan.
 *
 * Angles below 90 deg are the left side, above are the right side.  A
 * hit's lateral offset is distance * sin(angle - 90 deg).
 */
void
obstacle_scan_include (obstacle_scan_t * p_scan, uint32_t angle_deg,
                       float distance_cm)
{
    float lateral;

    if ((angle_deg < ULTRASONIC_COARSE_CENTER_DEG) &&
        (distance_cm < p_scan->left_clearance))
    {
        p_scan->left_clearance = distance_cm;
    }
    else if ((angle_deg > ULTRASONIC_COARSE_CENTER_DEG) &&
             (distance_cm < p_scan->right_clearance))
    {
        p_scan->right_clearance = distance_cm;
    }
    else
    {
        /* Centre, or not closer than before. */
    }

    if (distance_cm <= ULTRASONIC_THRESHOLD_CM)
    {
        lateral = distance_cm *
                  rmath_sinf(((float) angle_deg -
                              (float) ULTRASONIC_COARSE_CENTER_DEG) *
                             RMATH_DEG_TO_RAD);
        p_scan->b_detected = true;

        if (!p_scan->b_has_hit)
        {
            p_scan->minimum_lateral = lateral;
            p_scan->maximum_lateral = lateral;
            p_scan->minimum_hit     = distance_cm;
            p_scan->b_has_hit       = true;
        }
        else
        {
            p_scan->minimum_lateral = rmath_minf(p_scan->minimum_lateral,
                                                 lateral);
            p_scan->maximum_lateral = rmath_maxf(p_scan->maximum_lateral,
                                                 lateral);
            p_scan->minimum_hit     = rmath_minf(p_scan->minimum_hit,
                                                 distance_cm);
        }
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
 * @brief Width = lateral spread of the hits + the beam footprint at the
 *        nearest hit.
 */
float
obstacle_estimate_width (obstacle_scan_t const * p_scan)
{
    float width = 0.0f;
    float half_beam_rad;

    if (p_scan->b_has_hit)
    {
        half_beam_rad = (float) ULTRASONIC_BEAM_WIDTH_DEG * 0.5f *
                        RMATH_DEG_TO_RAD;
        width = (p_scan->maximum_lateral - p_scan->minimum_lateral) +
                (2.0f * p_scan->minimum_hit * rmath_tanf(half_beam_rad));
    }

    return (width);
}

/*** end of file ***/
