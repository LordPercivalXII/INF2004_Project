/** @file obstacle.c
 *
 * @brief Buddy 5 scanner task.
 *
 * Driving (the controller is following the line): the sensor looks
 * straight ahead.  OBSTACLE_STOP_READINGS close readings in a row post
 * OBSTACLE_ACTION_STOP; a scan takes too long to do at driving speed.
 *
 * Stopped (the controller is avoiding and asks for a scan):
 *   stage 1  coarse scan, 5 angles 60..120 deg
 *   stage 2  fine scan, 5 deg steps, over the region the coarse scan hit
 *   stage 3  obstacle profile, stage 4 decision (obstacle_geometry.c)
 * and one SYSTEM_EVENT_OBSTACLE with the result.  A scan stops at once if
 * the controller no longer wants it (the car is moving again).
 *
 * Moving while avoiding: one straight-ahead reading at a time, for the
 * avoidance's safety stop.
 */

#include "subsystem5.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../controller/controller.h"

#define OBSTACLE_COARSE_COUNT   (5U)
#define OBSTACLE_FAULT_SLEEP_MS (1000)
#define OBSTACLE_SETTLE_MS      (300)   /* car at rest before scanning */
#define OBSTACLE_REPORT_EVERY   (5U)    /* driving readings per report */
#define OBSTACLE_TEXT_MAX       (80U)
#define OBSTACLE_POINT_TEXT_MAX (12U)   /* " 105:400" plus margin */
#define OBSTACLE_POINTS_MAX     (16U + (OBSTACLE_MAX_POINTS * \
                                        OBSTACLE_POINT_TEXT_MAX))

/* Stage 1: the two ends, straight ahead and the midpoints. */
static uint32_t const g_coarse_angles[OBSTACLE_COARSE_COUNT] =
{
    ULTRASONIC_COARSE_LEFT_DEG,
    (ULTRASONIC_COARSE_LEFT_DEG + ULTRASONIC_COARSE_CENTER_DEG) / 2U,
    ULTRASONIC_COARSE_CENTER_DEG,
    (ULTRASONIC_COARSE_CENTER_DEG + ULTRASONIC_COARSE_RIGHT_DEG) / 2U,
    ULTRASONIC_COARSE_RIGHT_DEG
};

/* Set by the controller (obstacle_set_look()); one aligned word, so the
 * write from the other core is seen whole. */
static volatile avoid_look_t g_look = AVOID_LOOK_SCAN;

void
obstacle_set_look (avoid_look_t look)
{
    g_look = look;
}

/*!
 * @brief Kept for the controller; this version always looks straight
 *        ahead while driving.
 */
void
obstacle_set_lead (float lead)
{
    (void) lead;
}

static int
obstacle_round (float value)
{
    return ((int) ((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f)));
}

/*!
 * @brief Post an event; report a full buffer once, not every time.
 */
static void
obstacle_publish (system_event_t const * p_event)
{
    static bool b_error_reported = false;

    if (!controller_post_event(p_event))
    {
        if (!b_error_reported)
        {
            tm_printf((UB *) "S5: system event buffer full; "
                      "event was not delivered\n");
            b_error_reported = true;
        }
    }
    else
    {
        b_error_reported = false;
    }
}

static bool
obstacle_car_is_avoiding (void)
{
    T_RFLG status = {0};

    return ((E_OK == tk_ref_flg(gh_system_flg, &status)) &&
            (0U != (status.flgptn & SYSTEM_BIT_OBSTACLE)));
}

/*!
 * @brief True while the controller wants a full scan: it is avoiding and
 *        the car is stopped waiting for one.
 */
static bool
obstacle_scan_wanted (void)
{
    return (obstacle_car_is_avoiding() &&
            ((AVOID_LOOK_SCAN == g_look) || (AVOID_LOOK_SCAN_FAR == g_look)));
}

/*!
 * @brief Point the servo and ping.  After a move the sensor is given
 *        ULTRASONIC_SERVO_SETTLE_MS (two 50 ms ticks, Tribunal 1) to stop
 *        swinging before the trigger, so a distance is never measured
 *        mid-rotation.  No wait when the servo is already there, so the
 *        straight-ahead watch while driving is not slowed down.
 */
static float
obstacle_measure_at (uint32_t angle_deg, uint32_t scan_number)
{
    if (servo_get_angle() != angle_deg)
    {
        servo_set_angle(angle_deg);
        (void) tk_dly_tsk((TMO) ULTRASONIC_SERVO_SETTLE_MS);
    }

    return (ultrasonic_measure_cm(scan_number));
}

/*!
 * @brief Straight-ahead reading for the controller (telemetry, and the
 *        avoidance's safety stop).
 */
static void
obstacle_post_distance (float distance_cm)
{
    system_event_t event = {0};

    event.type         = SYSTEM_EVENT_OBSTACLE;
    event.timestamp_ms = board_millis();
    event.data.obstacle.action             = OBSTACLE_ACTION_CONTINUE;
    event.data.obstacle.center_distance_cm = distance_cm;
    obstacle_publish(&event);
}

static void
obstacle_post_stop (float distance_cm)
{
    system_event_t event = {0};

    event.type         = SYSTEM_EVENT_OBSTACLE;
    event.timestamp_ms = board_millis();
    event.data.obstacle.b_detected          = true;
    event.data.obstacle.action              = OBSTACLE_ACTION_STOP;
    event.data.obstacle.scan_start_ms       = event.timestamp_ms;
    event.data.obstacle.center_distance_cm  = distance_cm;
    event.data.obstacle.closest_distance_cm = distance_cm;
    obstacle_publish(&event);

    tm_printf((UB *) "S5: STOP obstacle in path at %d cm, 0 deg\n",
              obstacle_round(distance_cm));
}

/*!
 * @brief Stages 1 and 2: a coarse scan, then a fine scan over the region
 *        it hit.  Each sweep starts from the end nearer the servo, so it
 *        never swings across and back.
 *
 * @param[out] p_complete  true if the controller wanted the scan to the
 *                         end (the car did not move off).
 *
 * @return The straight-ahead distance.
 */
static float
obstacle_scan (obstacle_scan_t * p_scan, uint32_t scan_number,
               bool * p_complete)
{
    bool const b_from_right =
        (servo_get_angle() > ULTRASONIC_COARSE_CENTER_DEG);
    float      center_cm = ULTRASONIC_MAX_DISTANCE_CM;
    uint32_t   from_deg;
    uint32_t   to_deg;
    uint32_t   angle;
    uint32_t   idx;
    uint32_t   steps;
    float      distance;

    obstacle_scan_reset(p_scan);

    if (AVOID_LOOK_SCAN_FAR == g_look)
    {
        /* Backed up from it: the same obstacle is further away now. */
        p_scan->detect_cm = OBSTACLE_DETECT_FAR_CM;
    }

    for (idx = 0U; (idx < OBSTACLE_COARSE_COUNT) && obstacle_scan_wanted();
         idx++)
    {
        angle    = b_from_right
                   ? g_coarse_angles[OBSTACLE_COARSE_COUNT - 1U - idx]
                   : g_coarse_angles[idx];
        distance = obstacle_measure_at(angle, scan_number);
        obstacle_scan_add(p_scan, angle, distance);

        if (ULTRASONIC_COARSE_CENTER_DEG == angle)
        {
            center_cm = distance;
        }
    }

    if (obstacle_scan_wanted() &&
        obstacle_fine_range(p_scan, &from_deg, &to_deg))
    {
        /* The coarse scan ended at the left end if it started right. */
        steps = (to_deg - from_deg) / ULTRASONIC_FINE_STEP_DEG;

        for (idx = 0U; (idx <= steps) && obstacle_scan_wanted(); idx++)
        {
            angle = b_from_right
                    ? (from_deg + (idx * ULTRASONIC_FINE_STEP_DEG))
                    : (to_deg - (idx * ULTRASONIC_FINE_STEP_DEG));
            obstacle_scan_add(p_scan, angle,
                              obstacle_measure_at(angle, scan_number));
        }
    }

    *p_complete = obstacle_scan_wanted();

    return (center_cm);
}

static char const *
obstacle_action_name (obstacle_action_t action)
{
    static char const * const names[] =
    {
        "CONTINUE", "STOP", "TURN_LEFT", "TURN_RIGHT", "REVERSE_RETRY"
    };

    return (((uint32_t) action < (sizeof(names) / sizeof(names[0])))
            ? names[action] : "?");
}

/*!
 * @brief Print the profile and every reading, so a saved console log
 *        shows what the scanner saw (the format scan_recorder.html
 *        reads).  Each line goes out in one tm_putstring() so other
 *        tasks' output cannot split it.
 */
static void
obstacle_print_profile (obstacle_scan_t const * p_scan,
                        obstacle_profile_t const * p_profile,
                        obstacle_plan_t const * p_plan)
{
    static char line[OBSTACLE_POINTS_MAX];
    uint32_t    used;
    uint32_t    idx;
    INT         len;

    (void) tm_sprintf((UB *) line,
                      (UB *) "S5: obstacle %d..%d deg, centre %d deg / %d cm, "
                      "closest %d cm at %d deg, width %d cm, "
                      "clear L %d R %d cm, action %s, offset %d cm\n",
                      obstacle_round(p_profile->left_edge_deg),
                      obstacle_round(p_profile->right_edge_deg),
                      obstacle_round(p_profile->center_angle_deg),
                      obstacle_round(p_profile->lateral_offset_cm),
                      obstacle_round(p_profile->closest_cm),
                      obstacle_round(p_profile->closest_angle_deg),
                      obstacle_round(p_profile->width_cm),
                      obstacle_round(p_profile->left_clearance_cm),
                      obstacle_round(p_profile->right_clearance_cm),
                      obstacle_action_name(p_plan->action),
                      obstacle_round(p_plan->bypass_offset_cm));
    (void) tm_putstring((UB *) line);

    len  = tm_sprintf((UB *) line, (UB *) "S5: points");
    used = (len > 0) ? (uint32_t) len : 0U;

    for (idx = 0U; (idx < p_scan->count) &&
                   ((used + OBSTACLE_POINT_TEXT_MAX) < OBSTACLE_POINTS_MAX);
         idx++)
    {
        len = tm_sprintf((UB *) &line[used], (UB *) " %u:%d",
                         (unsigned int) p_scan->points[idx].angle_deg,
                         obstacle_round(p_scan->points[idx].distance_cm));
        used += (len > 0) ? (uint32_t) len : 0U;
    }

    line[used]      = '\n';
    line[used + 1U] = '\0';
    (void) tm_putstring((UB *) line);
}

/*!
 * @brief Scan, profile, decide, and post the result.
 */
static void
obstacle_scan_and_report (obstacle_scan_t * p_scan, uint32_t scan_number)
{
    obstacle_profile_t profile;
    obstacle_plan_t    plan;
    system_event_t     event        = {0};
    uint32_t const     start_ms     = board_millis();
    bool               b_complete   = false;
    float const        center_cm    = obstacle_scan(p_scan, scan_number,
                                                    &b_complete);

    if (!b_complete)
    {
        return;     /* the car moved off: the readings are stale */
    }

    obstacle_profile_build(p_scan, &profile);
    obstacle_plan_build(&profile, &plan);

    event.type         = SYSTEM_EVENT_OBSTACLE;
    event.timestamp_ms = board_millis();
    event.data.obstacle.b_detected          = profile.b_detected;
    event.data.obstacle.b_scan_complete     = true;
    event.data.obstacle.action              = plan.action;
    event.data.obstacle.scan_start_ms       = start_ms;
    event.data.obstacle.bypass_offset_cm    = plan.bypass_offset_cm;
    event.data.obstacle.center_distance_cm  = center_cm;
    event.data.obstacle.estimated_width_cm  = profile.width_cm;
    event.data.obstacle.closest_distance_cm = profile.closest_cm;
    event.data.obstacle.center_angle_deg    = profile.center_angle_deg;
    event.data.obstacle.lateral_offset_cm   = profile.lateral_offset_cm;
    event.data.obstacle.left_clearance_cm   = profile.left_clearance_cm;
    event.data.obstacle.right_clearance_cm  = profile.right_clearance_cm;
    event.data.obstacle.bypass_direction    =
        (OBSTACLE_ACTION_TURN_LEFT == plan.action) ? -1 :
        ((OBSTACLE_ACTION_TURN_RIGHT == plan.action) ? 1 : 0);

    if (profile.b_detected)
    {
        obstacle_print_profile(p_scan, &profile, &plan);
    }
    else
    {
        tm_printf((UB *) "S5: scan clear, CONTINUE\n");
    }

    obstacle_publish(&event);
}

/*!
 * @brief Obstacle task.
 */
void
obstacle_task (INT stacd, void * p_exinf)
{
    static obstacle_scan_t scan;    /* static: keeps ~260 B off the stack */
    uint32_t               scan_number  = 0U;
    uint32_t               close_count  = 0U;
    uint32_t               report_count = 0U;
    uint32_t               drops;
    bool                   b_was_scanning = false;
    float                  distance;

    (void) stacd;
    (void) p_exinf;

    servo_init();

    if (!ultrasonic_init())
    {
        system_event_t event = {0};

        event.type         = SYSTEM_EVENT_FAULT;
        event.timestamp_ms = board_millis();
        tm_printf((UB *) "S5: ultrasonic hardware initialisation failed\n");
        obstacle_publish(&event);

        for (;;)
        {
            (void) tk_dly_tsk(OBSTACLE_FAULT_SLEEP_MS);
        }
    }

    for (;;)
    {
        if (obstacle_scan_wanted())
        {
            if (!b_was_scanning)
            {
                /* Let the car come to rest first. */
                (void) tk_dly_tsk(OBSTACLE_SETTLE_MS);
                b_was_scanning = true;
            }

            obstacle_scan_and_report(&scan, scan_number);
            scan_number++;
        }
        else
        {
            b_was_scanning = false;
            distance = obstacle_measure_at(ULTRASONIC_COARSE_CENTER_DEG,
                                           scan_number);

            if (obstacle_car_is_avoiding())
            {
                /* Moving round the obstacle: every reading counts. */
                obstacle_post_distance(distance);
            }
            else if (obstacle_watch_update(&close_count, distance))
            {
                obstacle_post_stop(distance);
            }
            else if (0U == (++report_count % OBSTACLE_REPORT_EVERY))
            {
                obstacle_post_distance(distance);
                scan_number++;
            }
            else
            {
                /* Keep watching. */
            }
        }

        drops = ultrasonic_take_drops();

        if (0U != drops)
        {
            tm_printf((UB *) "S5: dropped %u ultrasonic echo capture(s)\n",
                      (unsigned int) drops);
        }
    }
}

/*** end of file ***/
