/** @file subsystem5_test.c
 *
 * @brief Tests for subsystem 5 (obstacle).  On the car: make TEST=1.
 *        The same file runs on the PC (pure logic only).
 *        Test IDs: testing/subsystem5/TEST_LOG.md.
 */

#include "subsystem5.h"
#include "avoidance.h"
#include "../common/robot_config.h"
#include "../test_report.h"

#if APP_TEST

#define S5_TEST_FAR_CM      (200.0f)
#define S5_TEST_NEAR_CM     (24.0f)
#define S5_TEST_STEP_MS     (50U)
#define S5_TEST_STEP_M      (0.01f)     /* model car: 1 cm per step   */
#define S5_TEST_TURN_STEP   (5.0f)      /* model car: 5 deg per step  */
#define S5_TEST_MAX_STEPS   (2000U)
#define S5_TEST_PHASES_MAX  (24U)
#define AVOID_TEST_FOLLOW_M_S (0.15f)    /* controller following   */

/*!
 * @brief Fill a full fine sweep (60..120 deg, 5 deg steps): near_cm
 *        inside [hit_from, hit_to], far everywhere else.
 */
static void
s5_test_sweep (obstacle_scan_t * p_scan, uint32_t hit_from, uint32_t hit_to,
               float near_cm)
{
    uint32_t angle;

    obstacle_scan_reset(p_scan);

    for (angle = ULTRASONIC_COARSE_LEFT_DEG;
         angle <= ULTRASONIC_COARSE_RIGHT_DEG;
         angle += ULTRASONIC_FINE_STEP_DEG)
    {
        obstacle_scan_add(p_scan, angle,
                          ((angle >= hit_from) && (angle <= hit_to))
                          ? near_cm : S5_TEST_FAR_CM);
    }
}

/*!
 * @brief Profile and decide in one go.
 */
static obstacle_plan_t
s5_test_plan (obstacle_scan_t const * p_scan)
{
    obstacle_profile_t profile;
    obstacle_plan_t    plan;

    obstacle_profile_build(p_scan, &profile);
    obstacle_plan_build(&profile, &plan);

    return (plan);
}

/*!
 * @brief A complete scan result for the avoidance, taken now.
 */
static obstacle_event_t
s5_test_result (obstacle_action_t action, float offset_cm, float closest_cm,
                uint32_t now_ms)
{
    obstacle_event_t result = {0};

    result.b_detected          = true;
    result.b_scan_complete     = true;
    result.action              = action;
    result.scan_start_ms       = now_ms;
    result.bypass_offset_cm    = offset_cm;
    result.closest_distance_cm = closest_cm;

    return (result);
}

/*!
 * @brief A model car for the avoidance: drives 1 cm and turns 5 deg per
 *        step, exactly as told.  With b_line it also has a line along
 *        the original path (lateral 0, 2 cm wide): the sensors see it
 *        while crossing it in SEARCH, once ALIGN has turned 20 deg, and
 *        in VERIFY.  Runs until the phase is `until`, the avoidance
 *        ends, or S5_TEST_MAX_STEPS.  Records every phase it passes.
 */
typedef struct
{
    avoid_input_t  input;
    float          speed;
    float          turn_left_deg;   /* still to turn, + = left */
    float          align_turned_deg;
    avoid_phase_t  phases[S5_TEST_PHASES_MAX];
    uint32_t       phase_count;
    float          min_speed;       /* most negative speed seen  */
} s5_model_t;

static avoid_status_t
s5_test_run (avoid_t * p_avoid, s5_model_t * p_car, avoid_phase_t until,
             bool b_line)
{
    avoid_status_t status = AVOID_RUNNING;
    avoid_output_t output;
    uint32_t       step;
    float          turn;

    for (step = 0U; (step < S5_TEST_MAX_STEPS) &&
                    (AVOID_RUNNING == status) &&
                    (p_avoid->phase != until); step++)
    {
        p_car->input.b_line_detected = b_line &&
            (((AVOID_PHASE_SEARCH == p_avoid->phase) &&
              (p_avoid->lateral_cm < 1.5f) &&
              (p_avoid->lateral_cm > -1.5f)) ||
             ((AVOID_PHASE_ALIGN == p_avoid->phase) &&
              (p_car->align_turned_deg >= 20.0f)) ||
             (AVOID_PHASE_VERIFY == p_avoid->phase));

        status = avoid_step(p_avoid, &p_car->input, &output);

        if ((0U == p_car->phase_count) ||
            (p_car->phases[p_car->phase_count - 1U] != p_avoid->phase))
        {
            if (p_car->phase_count < S5_TEST_PHASES_MAX)
            {
                p_car->phases[p_car->phase_count] = p_avoid->phase;
                p_car->phase_count++;
            }
        }

        if (AVOID_OUT_TURN == output.type)
        {
            p_car->turn_left_deg   = output.turn_deg;
            p_car->speed           = 0.0f;
            p_car->align_turned_deg = 0.0f;
        }
        else if (AVOID_OUT_DRIVE == output.type)
        {
            p_car->speed         = output.left_speed;
            p_car->turn_left_deg = 0.0f;
            p_car->min_speed     = (output.left_speed < p_car->min_speed)
                                   ? output.left_speed : p_car->min_speed;
        }
        else if (AVOID_OUT_STOP == output.type)
        {
            p_car->speed         = 0.0f;
            p_car->turn_left_deg = 0.0f;
        }
        else
        {
            /* Keep going. */
        }

        if (avoid_is_following(p_avoid))
        {
            p_car->speed = AVOID_TEST_FOLLOW_M_S;   /* the controller */
        }

        /* Move the model car. */
        turn = p_car->turn_left_deg;
        turn = (turn > S5_TEST_TURN_STEP) ? S5_TEST_TURN_STEP :
               ((turn < -S5_TEST_TURN_STEP) ? -S5_TEST_TURN_STEP : turn);
        p_car->input.heading_deg  += turn;
        p_car->turn_left_deg      -= turn;
        p_car->align_turned_deg   += (turn >= 0.0f) ? turn : -turn;
        p_car->input.b_turn_done   = (0.0f == p_car->turn_left_deg);
        p_car->input.distance_m   += (p_car->speed > 0.0f) ? S5_TEST_STEP_M :
                                     ((p_car->speed < 0.0f) ? -S5_TEST_STEP_M
                                                            : 0.0f);
        p_car->input.now_ms       += S5_TEST_STEP_MS;
    }

    return (status);
}

/*!
 * @brief Start an avoidance at standstill and feed it one scan result.
 */
static void
s5_test_begin (avoid_t * p_avoid, s5_model_t * p_car,
               obstacle_action_t action, float offset_cm, float closest_cm)
{
    obstacle_event_t result;
    avoid_output_t   output;

    *p_car = (s5_model_t) {0};
    p_car->input.b_turn_done = true;
    *p_avoid = (avoid_t) {0};
    avoid_start(p_avoid, 0U, 0.0f);
    (void) avoid_step(p_avoid, &p_car->input, &output);     /* stop */
    p_car->phases[0]   = p_avoid->phase;
    p_car->phase_count = 1U;
    result = s5_test_result(action, offset_cm, closest_cm, 0U);
    avoid_on_scan(p_avoid, &result);
}

void
subsystem5_test (void)
{
    static obstacle_scan_t scan;
    static avoid_t         avoid;
    static s5_model_t      car;
    obstacle_profile_t     profile;
    obstacle_plan_t        plan;
    obstacle_plan_t        plan2;
    obstacle_event_t       result;
    avoid_output_t         output;
    avoid_status_t         status;
    uint32_t               count;
    uint32_t               from_deg = 0U;
    uint32_t               to_deg   = 0U;
    uint32_t               idx;
    bool                   b_ok;

    TEST_INIT();

    /* ---- Stage 2: where to fine-scan ---------------------------------- */

    obstacle_scan_reset(&scan);
    obstacle_scan_add(&scan, 60U, S5_TEST_FAR_CM);
    obstacle_scan_add(&scan, 75U, S5_TEST_FAR_CM);
    obstacle_scan_add(&scan, 90U, 25.0f);
    obstacle_scan_add(&scan, 105U, S5_TEST_FAR_CM);
    obstacle_scan_add(&scan, 120U, S5_TEST_FAR_CM);
    b_ok = obstacle_fine_range(&scan, &from_deg, &to_deg) &&
           (75U == from_deg) && (105U == to_deg);
    obstacle_scan_add(&scan, 90U, S5_TEST_FAR_CM);
    obstacle_scan_add(&scan, 60U, 30.0f);
    b_ok = b_ok && obstacle_fine_range(&scan, &from_deg, &to_deg) &&
           (60U == from_deg) && (75U == to_deg);
    obstacle_scan_add(&scan, 60U, S5_TEST_FAR_CM);
    b_ok = b_ok && (!obstacle_fine_range(&scan, &from_deg, &to_deg));
    TEST_CHECK("S5-U31", b_ok,
               "fine scan: the coarse hit's region, one step either side, "
               "inside 60..120; none if nothing was hit");

    /* ---- Stage 3: the profile ------------------------------------------ */

    s5_test_sweep(&scan, 80U, 100U, S5_TEST_NEAR_CM);
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

    obstacle_scan_add(&scan, 90U, 50.0f);
    TEST_CHECK("S5-U12",
               ((13U == scan.count) && (90U == scan.points[6].angle_deg) &&
                (50.0f == scan.points[6].distance_cm)),
               "repeat angle replaces the reading, no new point");

    s5_test_sweep(&scan, 100U, 115U, S5_TEST_NEAR_CM);
    obstacle_profile_build(&scan, &profile);
    TEST_CHECK("S5-U06",
               (profile.b_detected &&
                (profile.center_angle_deg > 17.0f) &&
                (profile.center_angle_deg < 18.0f) &&
                (profile.lateral_offset_cm > 6.5f) &&
                (profile.lateral_offset_cm < 7.7f)),
               "obstacle right: centre +17.5 deg, offset ~+7 cm");

    s5_test_sweep(&scan, 60U, 80U, S5_TEST_NEAR_CM);   /* into the path */
    obstacle_profile_build(&scan, &profile);
    plan = s5_test_plan(&scan);
    TEST_CHECK("S5-U07",
               ((0.0f == profile.left_clearance_cm) &&
                (OBSTACLE_ACTION_TURN_RIGHT == plan.action)),
               "obstacle at the scan edge: that side blocked, pass right");

    s5_test_sweep(&scan, 0U, 0U, S5_TEST_NEAR_CM);
    obstacle_profile_build(&scan, &profile);
    TEST_CHECK("S5-U08",
               ((!profile.b_detected) && (0.0f == profile.width_cm)),
               "nothing in range: no obstacle, width 0");

    /* A real scan (2026-11-05 log): a wide box at 28-30 cm, its right edge
       between 105 and 110 deg.  Once with 90 deg at 30 cm, once at 31:
       the box and the decision stay the same. */
    {
        static uint32_t const angles[] = {60U, 65U, 70U, 75U, 80U, 85U, 90U,
                                          95U, 100U, 105U, 110U, 115U,
                                          120U};
        static float const    dist[]   = {29.0f, 29.0f, 28.0f, 28.0f, 29.0f,
                                          29.0f, 30.0f, 29.0f, 29.0f, 30.0f,
                                          35.0f, 109.0f, 109.0f};

        obstacle_scan_reset(&scan);

        for (idx = 0U; idx < (sizeof(angles) / sizeof(angles[0])); idx++)
        {
            obstacle_scan_add(&scan, angles[idx], dist[idx]);
        }

        plan = s5_test_plan(&scan);
        obstacle_scan_add(&scan, 90U, 31.0f);
        plan2 = s5_test_plan(&scan);
        TEST_CHECK("S5-U32",
                   ((OBSTACLE_ACTION_TURN_RIGHT == plan.action) &&
                    (OBSTACLE_ACTION_TURN_RIGHT == plan2.action) &&
                    (plan.bypass_offset_cm == plan2.bypass_offset_cm)),
                   "real scan at 28-30 cm: a 1 cm wobble does not change "
                   "the box or the decision");
    }

    /* ---- Stage 4: the decision ----------------------------------------- */

    /* Box on the right of the path: left needs the smaller move. */
    s5_test_sweep(&scan, 90U, 110U, S5_TEST_NEAR_CM);
    plan  = s5_test_plan(&scan);
    s5_test_sweep(&scan, 70U, 90U, S5_TEST_NEAR_CM);
    plan2 = s5_test_plan(&scan);
    TEST_CHECK("S5-U01",
               ((OBSTACLE_ACTION_TURN_LEFT == plan.action) &&
                (OBSTACLE_ACTION_TURN_RIGHT == plan2.action) &&
                (plan.bypass_offset_cm >= 10.0f)),
               "bypass side: the side needing the smaller move");

    /* Beside the path: edge at 27.5 deg, 35 cm away = ~16 cm to the side,
       clear of the car's half width + 4 cm (11.5 cm). */
    s5_test_sweep(&scan, 115U, 120U, 35.0f);
    b_ok  = (OBSTACLE_ACTION_CONTINUE == s5_test_plan(&scan).action);
    s5_test_sweep(&scan, 0U, 0U, S5_TEST_NEAR_CM);      /* nothing */
    b_ok  = b_ok && (OBSTACLE_ACTION_CONTINUE == s5_test_plan(&scan).action);
    s5_test_sweep(&scan, 90U, 110U, S5_TEST_NEAR_CM);
    b_ok  = b_ok && (OBSTACLE_ACTION_TURN_LEFT == s5_test_plan(&scan).action);
    s5_test_sweep(&scan, 70U, 90U, S5_TEST_NEAR_CM);
    b_ok  = b_ok && (OBSTACLE_ACTION_TURN_RIGHT == s5_test_plan(&scan).action);
    s5_test_sweep(&scan, 60U, 120U, S5_TEST_NEAR_CM);   /* no side clear */
    b_ok  = b_ok &&
            (OBSTACLE_ACTION_REVERSE_RETRY == s5_test_plan(&scan).action);
    s5_test_sweep(&scan, 85U, 95U, 12.0f);              /* too close */
    b_ok  = b_ok &&
            (OBSTACLE_ACTION_REVERSE_RETRY == s5_test_plan(&scan).action);
    TEST_CHECK("S5-U13", b_ok,
               "decision: continue (beside / nothing), left, right, "
               "reverse (no room / too close)");

    /* ---- Driving: the stop check --------------------------------------- */

    count = 0U;
    b_ok = (!obstacle_watch_update(&count, 18.0f)) &&
           (!obstacle_watch_update(&count, 18.0f)) &&
           (!obstacle_watch_update(&count, 60.0f)) &&   /* real, far */
           (!obstacle_watch_update(&count, 18.0f)) &&
           (!obstacle_watch_update(&count, 18.0f)) &&
           obstacle_watch_update(&count, 18.0f);
    TEST_CHECK("S5-U17", b_ok,
               "driving: three close readings in a row stop the car; a "
               "far reading between them starts the count again");

    count = 0U;
    b_ok = (!obstacle_watch_update(&count, 18.0f)) &&
           (!obstacle_watch_update(&count, ULTRASONIC_MAX_DISTANCE_CM)) &&
           (!obstacle_watch_update(&count, 17.0f)) &&
           obstacle_watch_update(&count, 16.0f);
    TEST_CHECK("S5-U30", b_ok,
               "driving: a missed echo neither counts nor restarts the "
               "count, so a narrow box still stops the car in time");

    /* ---- Stage 4 -> 5: the avoidance ----------------------------------- */

    /* Short turn-out: the turn reports done at 35 deg; the next turn aims
       at the original heading, so it turns back 35, not 45. */
    s5_test_begin(&avoid, &car, OBSTACLE_ACTION_TURN_RIGHT, 12.0f, 30.0f);
    (void) s5_test_run(&avoid, &car, AVOID_PHASE_TURN_OUT, false);
    car.input.b_turn_done = false;
    (void) avoid_step(&avoid, &car.input, &output);     /* turn -45 */
    car.input.heading_deg = -35.0f;
    car.input.b_turn_done = true;
    (void) avoid_step(&avoid, &car.input, &output);     /* to LEG_OUT */
    car.input.distance_m += 0.25f;                      /* 14 cm out */
    (void) avoid_step(&avoid, &car.input, &output);     /* to TURN_PARALLEL */
    (void) avoid_step(&avoid, &car.input, &output);
    TEST_CHECK("S5-U19",
               ((AVOID_PHASE_TURN_PARALLEL == avoid.phase) &&
                (AVOID_OUT_TURN == output.type) &&
                (output.turn_deg > 34.0f) && (output.turn_deg < 36.0f)),
               "turns aim at a heading: a short turn-out is undone exactly");

    /* Too close for the way out: at 60 deg a 30 cm offset moves the car
       17.3 cm forward, + 5 cm nose + 4 cm clearance = 26.3 cm needed;
       the box is at 16, so back up ~10 cm. */
    s5_test_begin(&avoid, &car, OBSTACLE_ACTION_TURN_LEFT, 30.0f, 16.0f);
    status = s5_test_run(&avoid, &car, AVOID_PHASE_TURN_OUT, false);
    TEST_CHECK("S5-U33",
               ((AVOID_RUNNING == status) &&
                (AVOID_PHASE_TURN_OUT == avoid.phase) &&
                (car.min_speed < 0.0f) &&
                (avoid.along_cm < -9.5f) && (avoid.along_cm > -11.5f)),
               "too close for the way out: backs up just enough (~10 cm, "
               "keeping 4 cm clear) before turning out");

    /* A hand put in front while driving past, then taken away: stop,
       wait, carry on, and still finish the bypass. */
    s5_test_begin(&avoid, &car, OBSTACLE_ACTION_TURN_RIGHT, 12.0f, 30.0f);
    (void) s5_test_run(&avoid, &car, AVOID_PHASE_LEG_PAST, false);
    (void) avoid_step(&avoid, &car.input, &output);     /* drive */
    avoid_on_ahead_reading(&avoid, 40.0f);
    status = avoid_step(&avoid, &car.input, &output);
    b_ok = (AVOID_RUNNING == status) && (AVOID_PHASE_LEG_PAST == avoid.phase);
    avoid_on_ahead_reading(&avoid, 8.0f);
    status = avoid_step(&avoid, &car.input, &output);
    b_ok = b_ok && (AVOID_RUNNING == status) &&
           (AVOID_PHASE_BLOCKED == avoid.phase) &&
           (AVOID_OUT_STOP == output.type);
    car.speed = 0.0f;
    (void) avoid_step(&avoid, &car.input, &output);     /* no reading */
    for (idx = 0U; idx < 4U; idx++)
    {
        b_ok = b_ok && (AVOID_PHASE_BLOCKED == avoid.phase);
        avoid_on_ahead_reading(&avoid, (1U == idx)
                               ? ULTRASONIC_MAX_DISTANCE_CM : 60.0f);
        (void) avoid_step(&avoid, &car.input, &output);
    }
    b_ok = b_ok && (AVOID_PHASE_LEG_PAST == avoid.phase);
    status = s5_test_run(&avoid, &car, AVOID_PHASE_IDLE, true);
    TEST_CHECK("S5-U37",
               (b_ok && (AVOID_DONE == status)),
               "going round: within 9 cm ahead stops and waits (40 cm does "
               "not); 3 clear readings carry on (a missed echo between "
               "them neither counts nor restarts), and the bypass still "
               "finishes");

    /* Something that stays: a reading between 8 and 15 cm restarts the
       clear count; after 8 s it gives up (blocked), not before. */
    s5_test_begin(&avoid, &car, OBSTACLE_ACTION_TURN_RIGHT, 12.0f, 30.0f);
    (void) s5_test_run(&avoid, &car, AVOID_PHASE_LEG_OUT, false);
    (void) avoid_step(&avoid, &car.input, &output);     /* drive */
    avoid_on_ahead_reading(&avoid, 7.0f);
    (void) avoid_step(&avoid, &car.input, &output);
    count = car.input.now_ms;                           /* blocked at */
    b_ok = (AVOID_PHASE_BLOCKED == avoid.phase);
    {
        static float const readings[] = {60.0f, 60.0f, 12.0f, 60.0f, 60.0f};

        for (idx = 0U; idx < (sizeof(readings) / sizeof(readings[0])); idx++)
        {
            avoid_on_ahead_reading(&avoid, readings[idx]);
            car.input.now_ms += S5_TEST_STEP_MS;
            status = avoid_step(&avoid, &car.input, &output);
            b_ok = b_ok && (AVOID_RUNNING == status) &&
                   (AVOID_PHASE_BLOCKED == avoid.phase);
        }
    }
    car.input.now_ms = count + 7950U;
    status = avoid_step(&avoid, &car.input, &output);
    b_ok = b_ok && (AVOID_RUNNING == status);
    car.input.now_ms = count + 8000U;
    status = avoid_step(&avoid, &car.input, &output);
    TEST_CHECK("S5-U38",
               (b_ok && (AVOID_FAILED_BLOCKED == status)),
               "blocked: a reading within 15 cm restarts the clear count; "
               "still there after 8 s, gives up (blocked), not before");

    /* Turning out: 12 cm ahead does not stop a turn, 7 cm (inside the
       9 cm limit) does.  When it
       has gone, the turn finishes only what is left (60 - 20 = 40 deg). */
    s5_test_begin(&avoid, &car, OBSTACLE_ACTION_TURN_RIGHT, 12.0f, 30.0f);
    (void) s5_test_run(&avoid, &car, AVOID_PHASE_TURN_OUT, false);
    (void) avoid_step(&avoid, &car.input, &output);     /* turn -60 */
    b_ok = (AVOID_OUT_TURN == output.type);
    car.input.heading_deg = -20.0f;
    car.input.b_turn_done = false;
    avoid_on_ahead_reading(&avoid, 12.0f);
    (void) avoid_step(&avoid, &car.input, &output);
    b_ok = b_ok && (AVOID_PHASE_TURN_OUT == avoid.phase);
    avoid_on_ahead_reading(&avoid, 7.0f);
    (void) avoid_step(&avoid, &car.input, &output);
    b_ok = b_ok && (AVOID_PHASE_BLOCKED == avoid.phase) &&
           (AVOID_OUT_STOP == output.type);
    car.input.b_turn_done = true;                       /* cancelled */
    for (idx = 0U; idx < 3U; idx++)
    {
        avoid_on_ahead_reading(&avoid, 60.0f);
        (void) avoid_step(&avoid, &car.input, &output);
    }
    b_ok = b_ok && (AVOID_PHASE_TURN_OUT == avoid.phase);
    (void) avoid_step(&avoid, &car.input, &output);
    TEST_CHECK("S5-U39",
               (b_ok && (AVOID_OUT_TURN == output.type) &&
                (output.turn_deg < -39.0f) && (output.turn_deg > -41.0f)),
               "turning: within 9 cm ahead stops the turn (12 cm does "
               "not); when clear it turns only the 40 deg left");

    /* Back on the line, confirming it: something 18 cm ahead is a new
       obstacle, so stop and wait for a scan. */
    s5_test_begin(&avoid, &car, OBSTACLE_ACTION_TURN_RIGHT, 12.0f, 30.0f);
    (void) s5_test_run(&avoid, &car, AVOID_PHASE_VERIFY, true);
    b_ok = (AVOID_PHASE_VERIFY == avoid.phase);
    avoid_on_ahead_reading(&avoid, 40.0f);
    status = avoid_step(&avoid, &car.input, &output);
    b_ok = b_ok && (AVOID_RUNNING == status) &&
           (AVOID_PHASE_VERIFY == avoid.phase);
    avoid_on_ahead_reading(&avoid, 18.0f);
    status = avoid_step(&avoid, &car.input, &output);
    (void) avoid_step(&avoid, &car.input, &output);
    TEST_CHECK("S5-U36",
               (b_ok && (AVOID_RUNNING == status) &&
                (AVOID_PHASE_WAIT_SCAN == avoid.phase) &&
                (AVOID_OUT_STOP == output.type) &&
                (AVOID_LOOK_SCAN == avoid_look(&avoid))),
               "confirming the line: something within 20 cm ahead is a new "
               "obstacle: stop and scan it; 40 cm is not");

    /* No line anywhere: search, two full sweeps, then give up. */
    s5_test_begin(&avoid, &car, OBSTACLE_ACTION_TURN_RIGHT, 12.0f, 30.0f);
    status = s5_test_run(&avoid, &car, AVOID_PHASE_IDLE, false);
    TEST_CHECK("S5-U35",
               ((AVOID_FAILED_NO_LINE == status) && (2U == avoid.sweep_count)),
               "line never found: two full sweeps, then line not found");

    /* Dead end: every scan says reverse.  Three back-ups at one spot,
       then boxed in, even when the controller restarts the avoidance. */
    status = AVOID_RUNNING;
    avoid  = (avoid_t) {0};
    car    = (s5_model_t) {0};
    for (idx = 0U; (idx < 6U) && (AVOID_RUNNING == status); idx++)
    {
        car.input.distance_m = 0.0f;
        avoid_start(&avoid, car.input.now_ms, 0.0f);
        (void) avoid_step(&avoid, &car.input, &output);     /* stop */
        result = s5_test_result(OBSTACLE_ACTION_REVERSE_RETRY, 0.0f, 12.0f,
                                car.input.now_ms);
        avoid_on_scan(&avoid, &result);
        status = avoid_step(&avoid, &car.input, &output);   /* decide */
        if (AVOID_RUNNING == status)
        {
            status = s5_test_run(&avoid, &car, AVOID_PHASE_WAIT_SCAN, false);
        }
    }
    TEST_CHECK("S5-U16",
               ((AVOID_FAILED_BOXED_IN == status) && (4U == idx)),
               "three reverses at one spot, then gives up as boxed in");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */

/*** end of file ***/
