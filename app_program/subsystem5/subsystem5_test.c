/** @file subsystem5_test.c
 *
 * @brief On-target tests for subsystem 5 (obstacle).  make TEST=1.
 */

#include "subsystem5.h"
#include "avoidance.h"
#include "../common/robot_config.h"
#include "../test_report.h"

#if APP_TEST

#define S5_TEST_FAR_CM  (200.0f)
#define S5_TEST_NEAR_CM (24.0f)

/*!
 * @brief Fill a full fine sweep: near inside [hit_from, hit_to],
 *        far everywhere else.
 */
static void
s5_test_sweep (obstacle_scan_t * p_scan, uint32_t hit_from, uint32_t hit_to)
{
    uint32_t angle;

    obstacle_scan_reset(p_scan);

    for (angle = ULTRASONIC_COARSE_LEFT_DEG;
         angle <= ULTRASONIC_COARSE_RIGHT_DEG;
         angle += ULTRASONIC_FINE_STEP_DEG)
    {
        obstacle_scan_add(p_scan, angle,
                          ((angle >= hit_from) && (angle <= hit_to))
                          ? S5_TEST_NEAR_CM : S5_TEST_FAR_CM);
    }
}

void
subsystem5_test (void)
{
    static obstacle_scan_t scan;
    static avoid_t         avoid;
    obstacle_profile_t     profile;
    obstacle_plan_t        plan;
    obstacle_event_t       obstacle;
    avoid_input_t          input;
    avoid_output_t         output;
    avoid_status_t         status;
    uint32_t               step;
    bool                   b_ok;

    TEST_INIT();

    TEST_CHECK("S5-U01",
               ((0 == obstacle_choose_bypass(10.0f, 10.0f)) &&
                (1 == obstacle_choose_bypass(10.0f, 80.0f)) &&
                (-1 == obstacle_choose_bypass(80.0f, 10.0f)) &&
                (-1 == obstacle_choose_bypass(90.0f, 60.0f))),
               "bypass side: blocked / right / left / clearer");

    /* Obstacle straight ahead, hits 80-100 deg. */
    s5_test_sweep(&scan, 80U, 100U);
    obstacle_profile_build(&scan, &profile);
    TEST_CHECK("S5-U10",
               (profile.b_detected &&
                (profile.left_clearance_cm > 199.0f) &&
                (profile.right_clearance_cm > 199.0f)),
               "obstacle ahead: both sides clear, not blocked by own hits");

    TEST_CHECK("S5-U11",
               ((profile.width_cm > 3.9f) && (profile.width_cm < 4.5f) &&
                (profile.center_angle_deg > -0.5f) &&
                (profile.center_angle_deg < 0.5f) &&
                (profile.closest_cm < 24.5f)),
               "obstacle ahead: centre 0 deg, width ~4.2 cm, closest 24 cm");

    /* 13 points 60..120 deg; 90 deg is index 6. */
    obstacle_scan_add(&scan, 90U, 50.0f);
    TEST_CHECK("S5-U12",
               ((13U == scan.count) && (90U == scan.points[6].angle_deg) &&
                (50.0f == scan.points[6].distance_cm)),
               "repeat angle replaces the reading, no new point");

    /* Obstacle to the right, hits 100-115 deg. */
    s5_test_sweep(&scan, 100U, 115U);
    obstacle_profile_build(&scan, &profile);
    TEST_CHECK("S5-U06",
               (profile.b_detected &&
                (profile.center_angle_deg > 17.0f) &&
                (profile.center_angle_deg < 18.0f) &&
                (profile.lateral_offset_cm > 6.5f) &&
                (profile.lateral_offset_cm < 8.0f)),
               "obstacle right: centre +17.5 deg, offset ~+7 cm");

    /* Obstacle reaching the left end of the scan. */
    s5_test_sweep(&scan, 60U, 70U);
    obstacle_profile_build(&scan, &profile);
    TEST_CHECK("S5-U07",
               ((0.0f == profile.left_clearance_cm) &&
                (1 == obstacle_choose_bypass(profile.left_clearance_cm,
                                             profile.right_clearance_cm))),
               "obstacle at scan edge: that side blocked, pass right");

    s5_test_sweep(&scan, 0U, 0U);
    obstacle_profile_build(&scan, &profile);
    TEST_CHECK("S5-U08",
               ((!profile.b_detected) && (0.0f == profile.width_cm)),
               "nothing in range: no obstacle, width 0");

    /* Stage 4: the decision. */
    obstacle_plan_build(&profile, &plan);
    b_ok = (OBSTACLE_ACTION_CONTINUE == plan.action);
    s5_test_sweep(&scan, 90U, 110U);    /* in the path, nearer the right */
    obstacle_profile_build(&scan, &profile);
    obstacle_plan_build(&profile, &plan);
    b_ok = b_ok && (OBSTACLE_ACTION_TURN_LEFT == plan.action) &&
           (plan.bypass_offset_cm >= 10.0f);
    s5_test_sweep(&scan, 70U, 90U);     /* in the path, nearer the left */
    obstacle_profile_build(&scan, &profile);
    obstacle_plan_build(&profile, &plan);
    b_ok = b_ok && (OBSTACLE_ACTION_TURN_RIGHT == plan.action);
    s5_test_sweep(&scan, 60U, 120U);    /* wall: no side is clear */
    obstacle_profile_build(&scan, &profile);
    obstacle_plan_build(&profile, &plan);
    b_ok = b_ok && (OBSTACLE_ACTION_REVERSE_RETRY == plan.action);
    TEST_CHECK("S5-U13", b_ok,
               "decision: continue, left, right, reverse from the profile");

    /* Two close readings, one bad echo, then three close readings. */
    step = 0U;
    b_ok = (!obstacle_watch_update(&step, 20.0f)) &&
           (!obstacle_watch_update(&step, 20.0f)) &&
           (!obstacle_watch_update(&step, 400.0f)) &&
           (!obstacle_watch_update(&step, 20.0f)) &&
           (!obstacle_watch_update(&step, 20.0f)) &&
           obstacle_watch_update(&step, 20.0f);
    TEST_CHECK("S5-U17", b_ok,
               "driving: three close readings in a row stop the car");

    /* Looking into a turn: steering sets the servo angle in 5 deg steps,
       capped at the 60..120 deg cable limits; a reading at 30 deg is
       counted by its part along the heading (cos 30 = 0.866). */
    b_ok = (90U == obstacle_lead_angle(0.0f)) &&
           (90U == obstacle_lead_angle(0.05f)) &&
           (105U == obstacle_lead_angle(0.5f)) &&
           (75U == obstacle_lead_angle(-0.5f)) &&
           (120U == obstacle_lead_angle(1.5f)) &&
           (60U == obstacle_lead_angle(-1.0f)) &&
           (obstacle_ahead_cm(90U, 20.0f) > 19.99f) &&
           (obstacle_ahead_cm(60U, 20.0f) > 17.2f) &&
           (obstacle_ahead_cm(60U, 20.0f) < 17.4f);
    TEST_CHECK("S5-U21", b_ok,
               "driving: the sensor looks the way the car is turning");

    /* Aim from the wheel speeds: at 0.15 m/s with the right wheel 2 cm/s
       faster, the curvature is 1.07 /m, so a point 25 cm along the curve
       is 7.6 deg left: lead -0.25.  A 0.1 m/s difference is beyond the
       30 deg limit; a spin on the spot looks fully toward the turn. */
    b_ok = (obstacle_lead_from_wheels(0.15f, 0.15f) == 0.0f) &&
           (obstacle_lead_from_wheels(0.0f, 0.0f) == 0.0f) &&
           (obstacle_lead_from_wheels(0.14f, 0.16f) < -0.24f) &&
           (obstacle_lead_from_wheels(0.14f, 0.16f) > -0.27f) &&
           (obstacle_lead_from_wheels(0.16f, 0.14f) > 0.24f) &&
           (obstacle_lead_from_wheels(0.10f, 0.20f) == -1.0f) &&
           (obstacle_lead_from_wheels(0.18f, -0.18f) == 1.0f);
    TEST_CHECK("S5-U26", b_ok,
               "driving: aim from the wheel speeds, along the curve");

    /* Turning hard (lead at full either way): one reading within 12 cm
       stops the car; further away, or on a gentle curve, it still takes
       several readings in a row. */
    b_ok = obstacle_hard_turn_stop(1.0f, 10.0f) &&
           obstacle_hard_turn_stop(-1.0f, 12.0f) &&
           (!obstacle_hard_turn_stop(1.0f, 15.0f)) &&
           (!obstacle_hard_turn_stop(0.5f, 10.0f)) &&
           (!obstacle_hard_turn_stop(0.0f, 5.0f));
    TEST_CHECK("S5-U25", b_ok,
               "turning hard: one very close reading stops the car");

    /* Stage 5: walk the manoeuvre through every phase, passing right.
       The obstacle's front face is 20 cm ahead; heading is + to the left. */
    avoid = (avoid_t) {0};
    avoid_start(&avoid, 0U, 0.0f);
    input = (avoid_input_t) {0U, 0.0f, 0.0f, false, true};
    (void) avoid_step(&avoid, &input, &output);
    b_ok = (AVOID_OUT_STOP == output.type) &&
           (AVOID_LOOK_SCAN == avoid_look(&avoid));
    obstacle = (obstacle_event_t) {0};
    obstacle.b_scan_complete     = true;
    obstacle.action              = OBSTACLE_ACTION_TURN_RIGHT;
    obstacle.bypass_offset_cm    = 12.0f;
    obstacle.closest_distance_cm = 20.0f;
    avoid_on_scan(&avoid, &obstacle);
    (void) avoid_step(&avoid, &input, &output);     /* to TURN_OUT */
    (void) avoid_step(&avoid, &input, &output);
    b_ok = b_ok && (AVOID_OUT_TURN == output.type) &&
           (output.turn_deg < -44.0f) &&            /* right is negative */
           (AVOID_LOOK_AHEAD == avoid_look(&avoid));
    input.b_turn_done = false;                      /* never reported: */
    input.heading_deg = -45.0f;                     /* the heading ends it */
    (void) avoid_step(&avoid, &input, &output);     /* to LEG_OUT */
    (void) avoid_step(&avoid, &input, &output);
    b_ok = b_ok && (AVOID_OUT_DRIVE == output.type);
    input.distance_m = 0.2f;                        /* 14 cm out */
    (void) avoid_step(&avoid, &input, &output);     /* to TURN_PARALLEL */
    (void) avoid_step(&avoid, &input, &output);
    b_ok = b_ok && (output.turn_deg > 44.0f) && (output.turn_deg < 46.0f);
    input.heading_deg = 0.0f;
    (void) avoid_step(&avoid, &input, &output);     /* to LEG_PAST */
    (void) avoid_step(&avoid, &input, &output);
    b_ok = b_ok && (AVOID_PHASE_LEG_PAST == avoid.phase) &&
           (AVOID_OUT_DRIVE == output.type) &&
           (AVOID_LOOK_LEFT == avoid_look(&avoid)); /* obstacle on left */

    /* The sensor, held 30 deg left, sees the obstacle's side twice, then
       passes its rear edge: 24 + 25 cos 30 = 45.8 cm along, so 26 cm
       thick behind the face at 20 cm. */
    input.distance_m = 0.30f;
    avoid_on_side_reading(&avoid, 25.0f);
    (void) avoid_step(&avoid, &input, &output);
    input.distance_m = 0.31f;
    avoid_on_side_reading(&avoid, 400.0f);
    (void) avoid_step(&avoid, &input, &output);
    input.distance_m = 0.32f;
    avoid_on_side_reading(&avoid, 400.0f);
    (void) avoid_step(&avoid, &input, &output);
    b_ok = b_ok && avoid.b_rear_found &&
           (AVOID_PHASE_LEG_PAST == avoid.phase) && /* rear not clear yet */
           (AVOID_LOOK_AHEAD == avoid_look(&avoid));    /* side done */
    TEST_CHECK("S5-U18",
               (avoid.b_rear_found && (avoid.thickness_cm > 25.0f) &&
                (avoid.thickness_cm < 26.5f)),
               "passing: side readings measure the thickness, ~26 cm");
    input.distance_m = 0.66f;                       /* 14 cm past: not yet */
    (void) avoid_step(&avoid, &input, &output);
    b_ok = b_ok && (AVOID_PHASE_LEG_PAST == avoid.phase);
    input.distance_m = 0.69f;                       /* 17.3 cm past the rear */
    (void) avoid_step(&avoid, &input, &output);     /* to TURN_IN */
    (void) avoid_step(&avoid, &input, &output);
    b_ok = b_ok && (output.turn_deg > 69.0f) && (output.turn_deg < 71.0f);
    input.heading_deg = 70.0f;
    (void) avoid_step(&avoid, &input, &output);     /* to SEARCH */
    (void) avoid_step(&avoid, &input, &output);
    b_ok = b_ok && (AVOID_PHASE_SEARCH == avoid.phase) &&
           (AVOID_LOOK_AHEAD == avoid_look(&avoid));
#if !ROBOT_SIMULATION
    input.b_line_detected = true;                   /* cross 2.5 cm of tape */
    for (step = 0U; step < 5U; step++)
    {
        input.distance_m += 0.005f;
        (void) avoid_step(&avoid, &input, &output);
    }
    input.b_line_detected = false;
    (void) avoid_step(&avoid, &input, &output);
#else
    input.b_line_detected = true;                   /* line never ends */
    for (step = 0U; step < 3U; step++)
    {
        (void) avoid_step(&avoid, &input, &output);
    }
    input.b_line_detected = false;
#endif
    b_ok = b_ok && (AVOID_PHASE_ADVANCE == avoid.phase);
    input.distance_m += 0.1f;
    (void) avoid_step(&avoid, &input, &output);     /* to ALIGN */
    (void) avoid_step(&avoid, &input, &output);
    b_ok = b_ok && (AVOID_OUT_TURN == output.type) &&
           (output.turn_deg < 0.0f);                /* turns right */
    input.b_turn_done     = false;                  /* turn running */
    input.b_line_detected = true;
    for (step = 0U; step < 3U; step++)
    {
        (void) avoid_step(&avoid, &input, &output);
    }
    b_ok = b_ok && avoid_is_following(&avoid) &&
           (AVOID_OUT_STOP == output.type);         /* turn cancelled */
    input.distance_m += 0.11f;
    TEST_CHECK("S5-U15",
               (b_ok && (AVOID_DONE == avoid_step(&avoid, &input, &output))),
               "manoeuvre: stop, bypass, find line, align, verify, done");

    /* Passing left, subsystem 2 turns only 35 of the 45 deg and says it
       is done: the turn back aims at the original heading, so 35 back. */
    avoid = (avoid_t) {0};
    avoid_start(&avoid, 0U, 0.0f);
    input = (avoid_input_t) {0U, 0.0f, 0.0f, false, true};
    (void) avoid_step(&avoid, &input, &output);
    obstacle.action = OBSTACLE_ACTION_TURN_LEFT;
    avoid_on_scan(&avoid, &obstacle);
    (void) avoid_step(&avoid, &input, &output);     /* to TURN_OUT */
    (void) avoid_step(&avoid, &input, &output);
    b_ok = (output.turn_deg > 44.0f);
    input.heading_deg = 35.0f;
    (void) avoid_step(&avoid, &input, &output);     /* to LEG_OUT */
    (void) avoid_step(&avoid, &input, &output);
    input.distance_m = 0.25f;                       /* 14 cm out */
    (void) avoid_step(&avoid, &input, &output);     /* to TURN_PARALLEL */
    (void) avoid_step(&avoid, &input, &output);
    TEST_CHECK("S5-U19",
               (b_ok && (AVOID_PHASE_TURN_PARALLEL == avoid.phase) &&
                (output.turn_deg < -34.0f) && (output.turn_deg > -36.0f) &&
                (avoid.lateral_cm > 13.0f)),
               "turns aim at a heading: a 35 deg turn-out is undone by 35");

    /* Driving past, something 10 cm ahead: stop, wait for three clear
       readings, carry on with the same step.  If it is still there after
       a second, it is a second obstacle: scan it (S5-U27 goes on). */
    input.heading_deg = 0.0f;
    (void) avoid_step(&avoid, &input, &output);     /* to LEG_PAST */
    (void) avoid_step(&avoid, &input, &output);
    b_ok = (AVOID_PHASE_LEG_PAST == avoid.phase) &&
           (AVOID_LOOK_RIGHT == avoid_look(&avoid)); /* obstacle on right */
    avoid_on_ahead_reading(&avoid, 10.0f);
    (void) avoid_step(&avoid, &input, &output);
    b_ok = b_ok && (AVOID_PHASE_BLOCKED == avoid.phase) &&
           (AVOID_OUT_STOP == output.type);
    for (step = 0U; step < 3U; step++)
    {
        b_ok = b_ok && (AVOID_PHASE_BLOCKED == avoid.phase);
        input.now_ms += 100U;
        avoid_on_ahead_reading(&avoid, 80.0f);
        (void) avoid_step(&avoid, &input, &output);
    }
    b_ok = b_ok && (AVOID_PHASE_LEG_PAST == avoid.phase) &&
           (AVOID_OUT_DRIVE == output.type);
    avoid_on_ahead_reading(&avoid, 10.0f);
    (void) avoid_step(&avoid, &input, &output);
    input.now_ms += 1000U;
    (void) avoid_step(&avoid, &input, &output);     /* to WAIT_SCAN */
    TEST_CHECK("S5-U20",
               (b_ok && avoid.b_second &&
                (AVOID_PHASE_WAIT_SCAN == avoid.phase) &&
                (AVOID_OUT_STOP == output.type) &&
                (AVOID_LOOK_SCAN == avoid_look(&avoid))),
               "something close ahead: stop, carry on when clear, else "
               "scan it as a second obstacle");

    /* The second obstacle's scan, taken 25 cm along and ~14 cm out (left).
       Same side: go round it further out, measured from the original
       line (12 + 14 cm out, front face 10 + 25 cm along).  The other side
       would cut back across the first obstacle: give up. */
    {
        avoid_t const    before = avoid;
        float const      lateral = avoid.lateral_cm;
        float const      along   = avoid.along_cm;
        obstacle_event_t second  = obstacle;

        second.action              = OBSTACLE_ACTION_TURN_LEFT;
        second.bypass_offset_cm    = 12.0f;
        second.closest_distance_cm = 10.0f;
        second.scan_start_ms       = input.now_ms;
        avoid_on_scan(&avoid, &second);
        (void) avoid_step(&avoid, &input, &output);
        b_ok = (AVOID_PHASE_TURN_OUT == avoid.phase) &&
               (avoid.decision_offset_cm > (lateral + 11.9f)) &&
               (avoid.decision_offset_cm < (lateral + 12.1f)) &&
               (avoid.decision_closest_cm > (along + 9.9f)) &&
               (avoid.decision_closest_cm < (along + 10.1f)) &&
               (avoid.lateral_cm == lateral);   /* line still at 0 */
        (void) avoid_step(&avoid, &input, &output);
        b_ok = b_ok && (AVOID_OUT_TURN == output.type);

        avoid = before;
        second.action = OBSTACLE_ACTION_TURN_RIGHT;
        avoid_on_scan(&avoid, &second);
        TEST_CHECK("S5-U27",
                   (b_ok && (AVOID_FAILED_BLOCKED ==
                             avoid_step(&avoid, &input, &output))),
                   "second obstacle: further out on the same side, else "
                   "give up");
    }

    /* Passing left a box too shallow to see from the side: two misses
       while driving past bound its rear where the beam crosses its near
       edge.  Offset 15 puts the edge 5.5 cm out (offset = edge + half the
       13 cm car + 3); 22 cm at 45 deg puts the car 15.6 cm out and along,
       so the beam (30 deg) crosses the edge 10.1 / tan 30 = 17.4 cm
       further: rear <= 33.0, depth <= 20.0. */
    avoid = (avoid_t) {0};
    avoid_start(&avoid, 0U, 0.0f);
    input = (avoid_input_t) {0U, 0.0f, 0.0f, false, true};
    (void) avoid_step(&avoid, &input, &output);
    obstacle.action              = OBSTACLE_ACTION_TURN_LEFT;
    obstacle.bypass_offset_cm    = 15.0f;
    obstacle.closest_distance_cm = 13.0f;
    avoid_on_scan(&avoid, &obstacle);
    (void) avoid_step(&avoid, &input, &output);     /* to TURN_OUT */
    (void) avoid_step(&avoid, &input, &output);
    input.heading_deg = 45.0f;
    (void) avoid_step(&avoid, &input, &output);     /* to LEG_OUT */
    (void) avoid_step(&avoid, &input, &output);
    input.distance_m = 0.22f;
    (void) avoid_step(&avoid, &input, &output);     /* to TURN_PARALLEL */
    (void) avoid_step(&avoid, &input, &output);
    b_ok = (AVOID_LOOK_RIGHT == avoid_look(&avoid));    /* aimed early */
    input.heading_deg = 0.0f;
    (void) avoid_step(&avoid, &input, &output);     /* to LEG_PAST */
    (void) avoid_step(&avoid, &input, &output);
    avoid_on_side_reading(&avoid, 400.0f);
    (void) avoid_step(&avoid, &input, &output);
    input.distance_m = 0.23f;
    avoid_on_side_reading(&avoid, 400.0f);
    (void) avoid_step(&avoid, &input, &output);
    TEST_CHECK("S5-U24",
               (b_ok && avoid.b_rear_found && avoid.b_rear_bounded &&
                (avoid.thickness_cm > 19.5f) && (avoid.thickness_cm < 20.5f) &&
                (AVOID_PHASE_LEG_PAST == avoid.phase)),
               "shallow box: side never seen, rear bounded at ~20 cm");

    /* A dead end: reversing and driving back must not loop forever. */
    avoid = (avoid_t) {0};
    obstacle.action = OBSTACLE_ACTION_REVERSE_RETRY;
    status = AVOID_RUNNING;
    for (step = 0U; (step < 6U) && (AVOID_RUNNING == status); step++)
    {
        input = (avoid_input_t) {step * 1000U, 0.0f, 0.0f, false, true};
        avoid_start(&avoid, input.now_ms, 0.0f);
        obstacle.scan_start_ms = input.now_ms;
        (void) avoid_step(&avoid, &input, &output);
        avoid_on_scan(&avoid, &obstacle);
        status = avoid_step(&avoid, &input, &output);
    }
    TEST_CHECK("S5-U16",
               ((AVOID_FAILED_BOXED_IN == status) && (4U == step)),
               "three reverses at one spot, then gives up as boxed in");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */

/*** end of file ***/
