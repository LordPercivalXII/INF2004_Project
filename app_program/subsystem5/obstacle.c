/** @file obstacle.c
 *
 * @brief Obstacle scanner task.  While driving, the sensor looks the way
 *        the car is turning (straight ahead on a straight); something
 *        close stops the car.
 *        Then: coarse scan, fine scan around hits, and one
 *        SYSTEM_EVENT_OBSTACLE per scan.  Free-running; paced by the servo
 *        settle delays and the minimum ping interval.
 */

#include "subsystem5.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../controller/controller.h"

#define OBSTACLE_COARSE_COUNT   (5U)
#define OBSTACLE_FAULT_SLEEP_MS (1000)
#define OBSTACLE_STOP_SETTLE_MS (300)   /* car at rest before rescanning */
#define OBSTACLE_STOP_LINE_MAX  (64U)
#define OBSTACLE_WATCH_REPORT_COUNT (5U)    /* readings per distance report */
#define OBSTACLE_GLANCE_EVERY   (3U)    /* off the side: every 3rd ahead */

/* Servo test, requested by testing/subsystem5/scan_recorder.ps1 pulsing
 * the USB DTR line.  Bench build only (real scanner, simulated motors):
 * it pauses scanning, which must never happen on a car that is driving. */
#define OBSTACLE_SERVO_TEST     (ROBOT_SIMULATION && S5_ULTRASONIC_HW && \
                                 TM_CONSOLE_USB_CDC)
#define OBSTACLE_TEST_COUNT     (5U)
#define OBSTACLE_TEST_HOLD_MS   (800)
#define OBSTACLE_PRINT_POINTS   (1)     /* 0 = profile line only */
#define OBSTACLE_POINT_TEXT_MAX (12U)   /* " 135:400" plus margin */
#define OBSTACLE_LINE_MAX       (16U + (OBSTACLE_MAX_POINTS * \
                                        OBSTACLE_POINT_TEXT_MAX))

/* Set by the controller (obstacle_set_look()); a single aligned word, so
 * the write from the other core is seen whole. */
static volatile avoid_look_t g_look = AVOID_LOOK_SCAN;

/* Set by the controller (obstacle_set_lead()): which way the car is
 * turning while it follows the line, -1 left .. +1 right. */
static volatile float g_lead = 0.0f;

/* 60 / 75 / 90 / 105 / 120 deg: edges, centre and the midpoints. */
static uint32_t const g_coarse_angles[OBSTACLE_COARSE_COUNT] =
{
    ULTRASONIC_COARSE_LEFT_DEG,
    (ULTRASONIC_COARSE_LEFT_DEG + ULTRASONIC_COARSE_CENTER_DEG) / 2U,
    ULTRASONIC_COARSE_CENTER_DEG,
    (ULTRASONIC_COARSE_CENTER_DEG + ULTRASONIC_COARSE_RIGHT_DEG) / 2U,
    ULTRASONIC_COARSE_RIGHT_DEG
};

/*!
 * @brief Post an event; report a full buffer once, not every scan.
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

#if OBSTACLE_SERVO_TEST
extern int tm_tusb_connected(void);     /* USB console: DTR from the host */

static bool gb_servo_test_requested = false;

/*!
 * @brief Note a DTR high-to-low pulse.  Polled at every reading, because
 *        a scan is much longer than the pulse.
 */
static void
obstacle_poll_servo_test (void)
{
    static bool b_dtr_was_high = false;
    bool const  b_dtr_high     = (0 != tm_tusb_connected());

    if (b_dtr_was_high && (!b_dtr_high))
    {
        gb_servo_test_requested = true;
    }

    b_dtr_was_high = b_dtr_high;
}
#endif

static float
obstacle_measure_at (uint32_t angle_deg, uint32_t scan_number)
{
#if OBSTACLE_SERVO_TEST
    obstacle_poll_servo_test();
#endif
    servo_set_angle(angle_deg);
    (void) tk_dly_tsk((TMO) ULTRASONIC_SERVO_SETTLE_MS);

    return (ultrasonic_measure_cm(scan_number));
}

/*!
 * @brief True during the avoidance manoeuvre, when the controller wants
 *        full scans (IF-05 status bits).
 */
static bool
obstacle_car_is_avoiding (void)
{
    T_RFLG status = {0};

    return ((E_OK == tk_ref_flg(gh_system_flg, &status)) &&
            (0U != (status.flgptn & SYSTEM_BIT_OBSTACLE)));
}

/*!
 * @brief Controller -> S5: where to point the scanner while avoiding.
 */
void
obstacle_set_look (avoid_look_t look)
{
    g_look = look;
}

/*!
 * @brief Controller -> S5: which way the car is turning while it follows
 *        the line; the sensor looks that way.
 */
void
obstacle_set_lead (float lead)
{
    g_lead = lead;
}

/*!
 * @brief True while the controller still wants full scans: always, unless
 *        it is going round an obstacle and has pointed the sensor.
 */
static bool
obstacle_wants_scan (void)
{
    return ((AVOID_LOOK_SCAN == g_look) || (!obstacle_car_is_avoiding()));
}

/*!
 * @brief Round to the nearest whole number (tm_printf has no %f).
 */
static int
obstacle_round (float value)
{
    return ((int) ((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f)));
}

/*!
 * @brief Tell the controller to stop now and report the reading that
 *        caused it; the profile comes later.
 */
static void
obstacle_post_stop (float distance_cm, int angle_deg)
{
    system_event_t event = {0};
    char           line[OBSTACLE_STOP_LINE_MAX];

    event.type         = SYSTEM_EVENT_OBSTACLE;
    event.timestamp_ms = board_millis();
    event.data.obstacle.b_detected          = true;
    event.data.obstacle.b_scan_complete     = false;
    event.data.obstacle.action              = OBSTACLE_ACTION_STOP;
    event.data.obstacle.scan_start_ms       = event.timestamp_ms;
    event.data.obstacle.center_distance_cm  = distance_cm;
    event.data.obstacle.closest_distance_cm = distance_cm;
    obstacle_publish(&event);

    (void) tm_sprintf((UB *) line,
                      (UB *) "S5: STOP obstacle in path at %d cm, %d deg\n",
                      obstacle_round(distance_cm), angle_deg);
    (void) tm_putstring((UB *) line);
}

/*!
 * @brief While driving, report the straight-ahead distance for telemetry.
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

/*!
 * @brief While going round the obstacle, report one reading toward its
 *        side for the thickness measurement (avoid_on_side_reading()).
 */
static void
obstacle_post_side (float distance_cm, int angle_deg)
{
    system_event_t event = {0};
    char           line[OBSTACLE_STOP_LINE_MAX];

    event.type         = SYSTEM_EVENT_OBSTACLE;
    event.timestamp_ms = board_millis();
    event.data.obstacle.action           = OBSTACLE_ACTION_CONTINUE;
    event.data.obstacle.b_side_reading   = true;
    event.data.obstacle.side_distance_cm = distance_cm;
    obstacle_publish(&event);

    (void) tm_sprintf((UB *) line, (UB *) "S5: side %d cm at %d deg\n",
                      obstacle_round(distance_cm), angle_deg);
    (void) tm_putstring((UB *) line);
}

/*!
 * @brief One full scan.  Abandoned part-way if the controller stops
 *        wanting scans (the car has started to move).
 *
 * @param[out] p_complete  true if every planned angle was read.
 *
 * @return The straight-ahead distance.
 */
static float
obstacle_scan (obstacle_scan_t * p_scan, uint32_t scan_number,
               bool * p_complete)
{
    uint32_t first_hit = ULTRASONIC_COARSE_RIGHT_DEG;
    uint32_t last_hit  = ULTRASONIC_COARSE_LEFT_DEG;
    float    center_cm = ULTRASONIC_MAX_DISTANCE_CM;
    bool     b_hit     = false;
    /* Sweep from the nearer end: no long swing back between scans. */
    bool const b_from_right =
        (servo_get_angle() > ULTRASONIC_COARSE_CENTER_DEG);
    uint32_t fine_start;
    uint32_t fine_end;
    uint32_t angle;
    uint32_t idx;
    float    distance;

    obstacle_scan_reset(p_scan);
    *p_complete = false;

    for (idx = 0U; (idx < OBSTACLE_COARSE_COUNT) && obstacle_wants_scan();
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

        if (distance <= ULTRASONIC_THRESHOLD_CM)
        {
            first_hit = (angle < first_hit) ? angle : first_hit;
            last_hit  = (angle > last_hit) ? angle : last_hit;
            b_hit     = true;
        }
    }

    if (b_hit)
    {
        fine_start = (first_hit > (ULTRASONIC_COARSE_LEFT_DEG +
                                   ULTRASONIC_FINE_MARGIN_DEG))
                     ? (first_hit - ULTRASONIC_FINE_MARGIN_DEG)
                     : ULTRASONIC_COARSE_LEFT_DEG;
        fine_end   = (last_hit < (ULTRASONIC_COARSE_RIGHT_DEG -
                                  ULTRASONIC_FINE_MARGIN_DEG))
                     ? (last_hit + ULTRASONIC_FINE_MARGIN_DEG)
                     : ULTRASONIC_COARSE_RIGHT_DEG;

        for (angle = fine_start;
             (angle <= fine_end) && obstacle_wants_scan();
             angle += ULTRASONIC_FINE_STEP_DEG)
        {
            obstacle_scan_add(p_scan, angle,
                              obstacle_measure_at(angle, scan_number));
        }
    }

    *p_complete = obstacle_wants_scan();

    return (center_cm);
}

#if OBSTACLE_SERVO_TEST
/*!
 * @brief Swing the servo between its limits slowly enough to watch, and
 *        report the distance at each stop.
 */
static void
obstacle_run_servo_test (uint32_t scan_number)
{
    static uint32_t const angles[OBSTACLE_TEST_COUNT] =
    {
        ULTRASONIC_COARSE_LEFT_DEG,
        ULTRASONIC_COARSE_RIGHT_DEG,
        ULTRASONIC_COARSE_LEFT_DEG,
        ULTRASONIC_COARSE_RIGHT_DEG,
        ULTRASONIC_COARSE_CENTER_DEG
    };
    char     line[OBSTACLE_STOP_LINE_MAX];
    uint32_t idx;

    gb_servo_test_requested = false;
    (void) tm_putstring((UB *) "S5: servo test start\n");

    for (idx = 0U; idx < OBSTACLE_TEST_COUNT; idx++)
    {
        servo_set_angle(angles[idx]);
        (void) tk_dly_tsk(OBSTACLE_TEST_HOLD_MS);
        (void) tm_sprintf((UB *) line,
                          (UB *) "S5: servo test at %d deg, reads %d cm\n",
                          (int) angles[idx] -
                          (int) ULTRASONIC_COARSE_CENTER_DEG,
                          obstacle_round(ultrasonic_measure_cm(scan_number)));
        (void) tm_putstring((UB *) line);
    }

    (void) tm_putstring((UB *) "S5: servo test done\n");
}
#endif

static char const *
obstacle_action_name (obstacle_action_t action)
{
    char const * p_name = "CONTINUE";

    if (OBSTACLE_ACTION_TURN_LEFT == action)
    {
        p_name = "TURN_LEFT";
    }
    else if (OBSTACLE_ACTION_TURN_RIGHT == action)
    {
        p_name = "TURN_RIGHT";
    }
    else if (OBSTACLE_ACTION_REVERSE_RETRY == action)
    {
        p_name = "REVERSE_RETRY";
    }
    else if (OBSTACLE_ACTION_STOP == action)
    {
        p_name = "STOP";
    }
    else
    {
        /* CONTINUE. */
    }

    return (p_name);
}

/*!
 * @brief Print the profile, and with OBSTACLE_PRINT_POINTS every reading,
 *        so the obstacle's shape can be checked on the console.
 *
 * Each line is built in one buffer and written with a single
 * tm_putstring(), like telemetry.c, so other tasks' output cannot land
 * in the middle of it.
 */
static void
obstacle_print_profile (obstacle_scan_t const * p_scan,
                        obstacle_profile_t const * p_profile,
                        obstacle_action_t action)
{
    static char line[OBSTACLE_LINE_MAX];
    uint32_t    used = 0U;
    uint32_t    idx;
    INT         len;

    (void) tm_sprintf((UB *) line,
                      (UB *) "S5: obstacle %d..%d deg, centre %d deg / %d cm, "
                      "closest %d cm at %d deg, width %d cm, "
                      "clear L %d R %d cm, action %s\n",
                      obstacle_round(p_profile->left_edge_deg),
                      obstacle_round(p_profile->right_edge_deg),
                      obstacle_round(p_profile->center_angle_deg),
                      obstacle_round(p_profile->lateral_offset_cm),
                      obstacle_round(p_profile->closest_cm),
                      obstacle_round(p_profile->closest_angle_deg),
                      obstacle_round(p_profile->width_cm),
                      obstacle_round(p_profile->left_clearance_cm),
                      obstacle_round(p_profile->right_clearance_cm),
                      obstacle_action_name(action));
    (void) tm_putstring((UB *) line);

#if OBSTACLE_PRINT_POINTS
    len  = tm_sprintf((UB *) line, (UB *) "S5: points");
    used = (len > 0) ? (uint32_t) len : 0U;

    for (idx = 0U; (idx < p_scan->count) &&
                   ((used + OBSTACLE_POINT_TEXT_MAX) < OBSTACLE_LINE_MAX);
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
#else
    (void) p_scan;
    (void) used;
    (void) idx;
    (void) len;
#endif
}

/*!
 * @brief Obstacle task.
 */
void
obstacle_task (INT stacd, void * p_exinf)
{
    static obstacle_scan_t scan;    /* static: keeps ~260 B off the stack */
    obstacle_profile_t     profile;
    obstacle_plan_t        plan;
    system_event_t         event;
    uint32_t               scan_number = 0U;
    uint32_t               scan_start_ms;
    uint32_t               drops;
    uint32_t               close_count = 0U;
    uint32_t               watch_count = 0U;
    float                  center_cm;
    avoid_look_t           look;
    uint32_t               look_angle;
    uint32_t               side_count  = 0U;
    float                  side_cm     = ULTRASONIC_MAX_DISTANCE_CM;
    bool                   b_complete;
    bool                   b_hw_ok;

    (void) stacd;
    (void) p_exinf;

    servo_init();
    b_hw_ok = ultrasonic_init();

    if (!b_hw_ok)
    {
        event = (system_event_t) {0};
        event.type         = SYSTEM_EVENT_FAULT;
        event.timestamp_ms = board_millis();
        tm_printf((UB *) "S5: ultrasonic hardware initialisation failed\n");
        obstacle_publish(&event);
    }

    while (!b_hw_ok)
    {
        (void) tk_dly_tsk(OBSTACLE_FAULT_SLEEP_MS);
    }

    for (;;)
    {
#if OBSTACLE_SERVO_TEST
        if (gb_servo_test_requested)
        {
            obstacle_run_servo_test(scan_number);
        }
#endif

        look = g_look;

        if ((AVOID_LOOK_LEFT != look) && (AVOID_LOOK_RIGHT != look))
        {
            /* The next side watch starts without contact. */
            side_cm    = ULTRASONIC_MAX_DISTANCE_CM;
            side_count = 0U;
        }

        if (obstacle_car_is_avoiding() && (AVOID_LOOK_SCAN != look))
        {
            /* Going round the obstacle: hold the sensor where the
               controller wants it, one reading at a time. */
            if (AVOID_LOOK_AHEAD == look)
            {
                obstacle_post_distance(
                    obstacle_measure_at(ULTRASONIC_COARSE_CENTER_DEG,
                                        scan_number));
            }
            else if ((side_cm > AVOID_SIDE_SEEN_CM) &&
                     (0U == (++side_count % OBSTACLE_GLANCE_EVERY)))
            {
                /* Not touching the obstacle's side: glance ahead for
                   anything else in the way. */
                obstacle_post_distance(
                    obstacle_measure_at(ULTRASONIC_COARSE_CENTER_DEG,
                                        scan_number));
            }
            else
            {
                /* On the obstacle's side: stay on it, every reading counts
                   toward where its rear edge is. */
                look_angle = (AVOID_LOOK_LEFT == look)
                             ? ULTRASONIC_COARSE_LEFT_DEG
                             : ULTRASONIC_COARSE_RIGHT_DEG;
                side_cm    = obstacle_measure_at(look_angle, scan_number);
                obstacle_post_side(side_cm, (int) look_angle -
                                            (int) ULTRASONIC_COARSE_CENTER_DEG);
            }

            continue;
        }

        if (!obstacle_car_is_avoiding())
        {
            /* Driving: look the way the car is turning (straight ahead on
               a straight), no sweeping, until something is close.  The
               simulated sensor changes with scan_number, which must hold
               still for several readings to give a stop. */
            look_angle = obstacle_lead_angle(g_lead);
            center_cm  = obstacle_ahead_cm(look_angle,
                             obstacle_measure_at(look_angle, scan_number));

            if (!obstacle_watch_update(&close_count, center_cm))
            {
                watch_count++;

                if (0U == (watch_count % OBSTACLE_WATCH_REPORT_COUNT))
                {
                    obstacle_post_distance(center_cm);
                    scan_number++;
                }

                continue;
            }

            /* Let the car come to rest, then profile from where it is. */
            obstacle_post_stop(center_cm, (int) look_angle -
                                          (int) ULTRASONIC_COARSE_CENTER_DEG);
            (void) tk_dly_tsk(OBSTACLE_STOP_SETTLE_MS);
        }

        scan_start_ms = board_millis();
        center_cm     = obstacle_scan(&scan, scan_number, &b_complete);

        if (!b_complete)
        {
            /* The car moved off mid-scan: the readings are stale. */
            continue;
        }

        obstacle_profile_build(&scan, &profile);
        obstacle_plan_build(&profile, &plan);

        event = (system_event_t) {0};
        event.type         = SYSTEM_EVENT_OBSTACLE;
        event.timestamp_ms = board_millis();
        event.data.obstacle.center_distance_cm  = center_cm;
        event.data.obstacle.b_detected          = profile.b_detected;
        event.data.obstacle.b_scan_complete     = true;
        event.data.obstacle.action              = plan.action;
        event.data.obstacle.scan_start_ms       = scan_start_ms;
        event.data.obstacle.bypass_offset_cm    = plan.bypass_offset_cm;
        event.data.obstacle.estimated_width_cm  = profile.width_cm;
        event.data.obstacle.closest_distance_cm = profile.closest_cm;
        event.data.obstacle.center_angle_deg    = profile.center_angle_deg;
        event.data.obstacle.lateral_offset_cm   = profile.lateral_offset_cm;
        event.data.obstacle.left_clearance_cm   =
            (profile.left_clearance_cm <= ULTRASONIC_THRESHOLD_CM)
            ? 0.0f : profile.left_clearance_cm;
        event.data.obstacle.right_clearance_cm  =
            (profile.right_clearance_cm <= ULTRASONIC_THRESHOLD_CM)
            ? 0.0f : profile.right_clearance_cm;
        event.data.obstacle.bypass_direction    = profile.b_detected
            ? obstacle_choose_bypass(event.data.obstacle.left_clearance_cm,
                                     event.data.obstacle.right_clearance_cm)
            : 0;

        if (profile.b_detected)
        {
            obstacle_print_profile(&scan, &profile, plan.action);
        }

        obstacle_publish(&event);
        scan_number++;

        drops = ultrasonic_take_drops();

        if (0U != drops)
        {
            tm_printf((UB *) "S5: dropped %u ultrasonic echo capture(s)\n",
                      (unsigned int) drops);
        }
    }
}

/*** end of file ***/
