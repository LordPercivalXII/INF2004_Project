/** @file obstacle.c
 *
 * @brief Obstacle scanner task: coarse scan, fine scan around hits,
 *        then one SYSTEM_EVENT_OBSTACLE per scan.  Free-running; paced
 *        by the servo settle delays and the minimum ping interval.
 */

#include "subsystem5.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../controller/controller.h"

#define OBSTACLE_COARSE_COUNT   (3U)
#define OBSTACLE_FAULT_SLEEP_MS (1000)

static uint32_t const g_coarse_angles[OBSTACLE_COARSE_COUNT] =
{
    ULTRASONIC_COARSE_LEFT_DEG,
    ULTRASONIC_COARSE_CENTER_DEG,
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

static float
obstacle_measure_at (uint32_t angle_deg, uint32_t scan_number)
{
    servo_set_angle(angle_deg);
    (void) tk_dly_tsk((TMO) ULTRASONIC_SERVO_SETTLE_MS);

    return (ultrasonic_measure_cm(scan_number));
}

/*!
 * @brief One full scan.  Returns the centre distance.
 */
static float
obstacle_scan (obstacle_scan_t * p_scan, uint32_t scan_number)
{
    uint32_t first_hit = ULTRASONIC_COARSE_RIGHT_DEG;
    uint32_t last_hit  = ULTRASONIC_COARSE_LEFT_DEG;
    float    center_cm = ULTRASONIC_MAX_DISTANCE_CM;
    uint32_t fine_start;
    uint32_t fine_end;
    uint32_t angle;
    uint32_t idx;
    float    distance;

    obstacle_scan_reset(p_scan);

    for (idx = 0U; idx < OBSTACLE_COARSE_COUNT; idx++)
    {
        angle    = g_coarse_angles[idx];
        distance = obstacle_measure_at(angle, scan_number);
        obstacle_scan_include(p_scan, angle, distance);

        if (ULTRASONIC_COARSE_CENTER_DEG == angle)
        {
            center_cm = distance;
        }

        if (distance <= ULTRASONIC_THRESHOLD_CM)
        {
            first_hit = (angle < first_hit) ? angle : first_hit;
            last_hit  = (angle > last_hit) ? angle : last_hit;
        }
    }

    if (p_scan->b_detected)
    {
        fine_start = (first_hit > (ULTRASONIC_COARSE_LEFT_DEG +
                                   ULTRASONIC_FINE_MARGIN_DEG))
                     ? (first_hit - ULTRASONIC_FINE_MARGIN_DEG)
                     : ULTRASONIC_COARSE_LEFT_DEG;
        fine_end   = (last_hit < (ULTRASONIC_COARSE_RIGHT_DEG -
                                  ULTRASONIC_FINE_MARGIN_DEG))
                     ? (last_hit + ULTRASONIC_FINE_MARGIN_DEG)
                     : ULTRASONIC_COARSE_RIGHT_DEG;

        for (angle = fine_start; angle <= fine_end;
             angle += ULTRASONIC_FINE_STEP_DEG)
        {
            obstacle_scan_include(p_scan, angle,
                                  obstacle_measure_at(angle, scan_number));
        }
    }

    return (center_cm);
}

/*!
 * @brief Obstacle task.
 */
void
obstacle_task (INT stacd, void * p_exinf)
{
    obstacle_scan_t scan;
    system_event_t  event;
    uint32_t        scan_number = 0U;
    uint32_t        drops;
    bool            b_hw_ok;

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
        event = (system_event_t) {0};
        event.data.obstacle.center_distance_cm =
            obstacle_scan(&scan, scan_number);

        event.type         = SYSTEM_EVENT_OBSTACLE;
        event.timestamp_ms = board_millis();
        event.data.obstacle.b_detected         = scan.b_detected;
        event.data.obstacle.b_scan_complete    = true;
        event.data.obstacle.estimated_width_cm =
            obstacle_estimate_width(&scan);
        event.data.obstacle.left_clearance_cm  =
            (scan.left_clearance <= ULTRASONIC_THRESHOLD_CM)
            ? 0.0f : scan.left_clearance;
        event.data.obstacle.right_clearance_cm =
            (scan.right_clearance <= ULTRASONIC_THRESHOLD_CM)
            ? 0.0f : scan.right_clearance;
        event.data.obstacle.bypass_direction   = scan.b_detected
            ? obstacle_choose_bypass(event.data.obstacle.left_clearance_cm,
                                     event.data.obstacle.right_clearance_cm)
            : 0;

        obstacle_publish(&event);
        servo_set_angle(ULTRASONIC_COARSE_CENTER_DEG);
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
