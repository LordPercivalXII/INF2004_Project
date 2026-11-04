/** @file avoidance.c
 *
 * @brief Avoidance manoeuvre and line recovery (see avoidance.h).
 *
 * The first-guess distances and speeds below were not measured on the
 * car: tune them on the track (test S5-M01).
 */

#include "avoidance.h"
#include "../common/robot_config.h"
#include "../common/rmath.h"

/* Every straight move uses the speed subsystem 2 has tested on the floor
 * (S2-M02), and every rotation is subsystem 2's closed-loop MOTION_TURN
 * (S2-M03), so no step runs slow enough to stall the motors. */
#define AVOID_LEG_SPEED_M_S         (0.20f)
#define AVOID_SEARCH_SPEED_M_S      (0.20f)
#define AVOID_TURN_DEG              (45.0f)
#define AVOID_ALIGN_OVERSHOOT_DEG   (45.0f) /* past the original heading */
#define AVOID_SWEEP_DEG             (360.0f)    /* MOTION_TURN maximum */
#define AVOID_HEADING_TOL_DEG       (4.0f)  /* a turn this close is done */
#define AVOID_SIN_45                (0.7071f)
#define AVOID_REVERSE_CM            (12.0f)
#define AVOID_REVERSE_MAX           (3U)
#define AVOID_RETRY_FORGET_CM       (40.0f) /* drove clear of that spot */
#define AVOID_OBSTACLE_DEPTH_CM     (15.0f) /* if its side is never seen */
#define AVOID_DEPTH_MAX_CM          (60.0f) /* stop waiting for the rear */
#define AVOID_CAR_LENGTH_CM         (16.0f)
#define AVOID_PAST_MARGIN_CM        (5.0f)
#define AVOID_OUT_SLACK             (1.5f)  /* LEG_OUT path limit factor */
#define AVOID_SIDE_LOOK_DEG         ((float) (ULTRASONIC_COARSE_RIGHT_DEG - \
                                              ULTRASONIC_COARSE_CENTER_DEG))
#define AVOID_SIDE_BEHIND_CM        (5.0f)  /* hits before the front face */
#define AVOID_AHEAD_STOP_CM         (12.0f) /* this close ahead: stop */
#define AVOID_AHEAD_CLEAR_READINGS  (3U)    /* clear readings to carry on */
#define AVOID_BLOCKED_TIMEOUT_MS    (8000U)
#define AVOID_SIDE_GAP_READINGS     (2U)    /* far readings = rear passed */
#define AVOID_SEARCH_OVERSHOOT_CM   (10.0f) /* past the line, not seen */
#define AVOID_SEARCH_MAX_CM         (60.0f) /* stays on the tile mat */
#define AVOID_SENSOR_AHEAD_CM       (8.0f)  /* IR sensors ahead of axle */
#define AVOID_TAPE_MIN_CM           (1.0f)  /* 1.8 cm tape, thin bars out */
#define AVOID_TAPE_MAX_CM           (5.0f)  /* wide black patches out */
#define AVOID_BLACK_SAMPLES         (3U)    /* consecutive line events */
#define AVOID_SWEEP_MAX             (2U)
#define AVOID_VERIFY_CM             (10.0f)
#define AVOID_VERIFY_LOST_MS        (600U)
#define AVOID_SCAN_TIMEOUT_MS       (8000U)
#define AVOID_TURN_TIMEOUT_MS       (4000U)
#define AVOID_SWEEP_TIMEOUT_MS      (8000U)
#define AVOID_DRIVE_TIMEOUT_MS      (8000U)
#define AVOID_CM_PER_M              (100.0f)

/* The simulated line never leaves the sensors, so a black run never ends. */
#ifndef AVOID_SIM_LINE
#define AVOID_SIM_LINE              (ROBOT_SIMULATION)
#endif

static void
avoid_enter (avoid_t * p_avoid, avoid_phase_t phase,
             avoid_input_t const * p_input)
{
    p_avoid->phase          = phase;
    p_avoid->phase_start_ms = p_input->now_ms;
    p_avoid->phase_start_m  = p_input->distance_m;
    p_avoid->white_since_ms = p_input->now_ms;
    p_avoid->black_samples  = 0U;
    p_avoid->b_in_black     = false;
    p_avoid->b_entered      = false;

    if (AVOID_PHASE_LEG_PAST == phase)
    {
        /* The beam swung during the turn: judge the rear edge only on
           readings taken while driving past. */
        p_avoid->side_gap_count = 0U;
    }
}

static float
avoid_travelled_cm (avoid_t const * p_avoid, avoid_input_t const * p_input)
{
    return (rmath_absf(p_input->distance_m - p_avoid->phase_start_m) *
            AVOID_CM_PER_M);
}

/*!
 * @brief Heading relative to the one the car had when it decided.
 */
static float
avoid_relative_deg (avoid_t const * p_avoid, avoid_input_t const * p_input)
{
    return (p_input->heading_deg - p_avoid->heading0_deg);
}

static void
avoid_drive (avoid_output_t * p_output, float left_speed, float right_speed)
{
    p_output->type        = AVOID_OUT_DRIVE;
    p_output->left_speed  = left_speed;
    p_output->right_speed = right_speed;
}

static void
avoid_turn (avoid_output_t * p_output, float angle_deg)
{
    p_output->type     = AVOID_OUT_TURN;
    p_output->turn_deg = angle_deg;
}

/*!
 * @brief Dead reckoning since the decision: each bit of distance moves
 *        the car along its current heading.
 */
static void
avoid_track_position (avoid_t * p_avoid, avoid_input_t const * p_input)
{
    float const step_cm = (p_input->distance_m - p_avoid->last_m) *
                          AVOID_CM_PER_M;
    float const rel_rad = avoid_relative_deg(p_avoid, p_input) *
                          RMATH_DEG_TO_RAD;

    p_avoid->last_m      = p_input->distance_m;
    p_avoid->along_cm   += step_cm * rmath_cosf(rel_rad);
    /* sin(rel) is + to the left; the bypass side is left when
       direction = -1. */
    p_avoid->lateral_cm += step_cm * rmath_sinf(rel_rad) *
                           (float) (-p_avoid->direction);
}

/*!
 * @brief Use the latest side reading: where along the original heading
 *        the beam hit the obstacle, and when it stops hitting it.
 *
 * The sensor looks AVOID_SIDE_LOOK_DEG toward the obstacle.  The beam is
 * wider than a ray, so the rear edge is found a little late; that errs
 * toward a thicker obstacle and a wider berth.
 */
static void
avoid_track_side (avoid_t * p_avoid, avoid_input_t const * p_input)
{
    float const look_rad = (avoid_relative_deg(p_avoid, p_input) +
                            ((float) p_avoid->direction *
                             AVOID_SIDE_LOOK_DEG)) * RMATH_DEG_TO_RAD;
    float hit_along;

    if (p_avoid->b_side_fresh && (AVOID_PHASE_LEG_PAST != p_avoid->phase))
    {
        /* Taken before or after driving past: the beam was swinging. */
        p_avoid->b_side_fresh = false;
    }
    else if (p_avoid->b_side_fresh)
    {
        p_avoid->b_side_fresh = false;
        hit_along = p_avoid->along_cm +
                    (p_avoid->side_cm * rmath_cosf(look_rad));

        if ((p_avoid->side_cm <= AVOID_SIDE_SEEN_CM) &&
            (hit_along >= (p_avoid->decision_closest_cm -
                           AVOID_SIDE_BEHIND_CM)))
        {
            p_avoid->b_side_seen    = true;
            p_avoid->side_gap_count = 0U;
            p_avoid->rear_along_cm  = rmath_maxf(p_avoid->rear_along_cm,
                                                 hit_along);
        }
        else if (p_avoid->b_side_seen && (!p_avoid->b_rear_found))
        {
            p_avoid->side_gap_count++;

            if (p_avoid->side_gap_count >= AVOID_SIDE_GAP_READINGS)
            {
                p_avoid->b_rear_found = true;
                p_avoid->thickness_cm =
                    rmath_maxf(0.0f, p_avoid->rear_along_cm -
                                     p_avoid->decision_closest_cm);
            }
        }
        else
        {
            /* Nothing there yet, or the rear is already known. */
        }
    }
}

/*!
 * @brief Turn the decision from a fresh scan into the next phase.
 */
static avoid_status_t
avoid_apply_decision (avoid_t * p_avoid, avoid_input_t const * p_input)
{
    avoid_status_t status = AVOID_RUNNING;

    p_avoid->b_decision = false;

    switch (p_avoid->decision)
    {
        case OBSTACLE_ACTION_TURN_LEFT:
        case OBSTACLE_ACTION_TURN_RIGHT:
            p_avoid->direction    = (OBSTACLE_ACTION_TURN_LEFT ==
                                     p_avoid->decision) ? -1 : 1;
            p_avoid->heading0_deg = p_input->heading_deg;
            p_avoid->last_m       = p_input->distance_m;
            p_avoid->along_cm     = 0.0f;
            p_avoid->lateral_cm   = 0.0f;
            p_avoid->past_cm      = p_avoid->decision_closest_cm +
                                    AVOID_OBSTACLE_DEPTH_CM +
                                    AVOID_CAR_LENGTH_CM;
            avoid_enter(p_avoid, AVOID_PHASE_TURN_OUT, p_input);
            break;

        case OBSTACLE_ACTION_REVERSE_RETRY:
            if (p_avoid->reverse_count >= AVOID_REVERSE_MAX)
            {
                status = AVOID_FAILED_BOXED_IN;
            }
            else
            {
                p_avoid->reverse_count++;
                p_avoid->reverse_end_m = p_input->distance_m;
                avoid_enter(p_avoid, AVOID_PHASE_REVERSE, p_input);
            }
            break;

        case OBSTACLE_ACTION_CONTINUE:
            status = AVOID_DONE;
            break;

        default:
            /* STOP on a complete scan: stay stopped and scan again. */
            avoid_enter(p_avoid, AVOID_PHASE_WAIT_SCAN, p_input);
            break;
    }

    return (status);
}

/*!
 * @brief Watch for a black run the width of the tape crossed at an angle.
 *
 * @return true when the line has been crossed.
 */
static bool
avoid_tape_crossed (avoid_t * p_avoid, avoid_input_t const * p_input)
{
    bool b_crossed = false;

    if (p_input->b_line_detected)
    {
        if (!p_avoid->b_in_black)
        {
            p_avoid->b_in_black    = true;
            p_avoid->black_start_m = p_input->distance_m;
            p_avoid->black_samples = 0U;
        }

        p_avoid->black_samples++;
#if AVOID_SIM_LINE
        b_crossed = (p_avoid->black_samples >= AVOID_BLACK_SAMPLES);
#endif
    }
    else if (p_avoid->b_in_black)
    {
        float const run_cm = rmath_absf(p_input->distance_m -
                                        p_avoid->black_start_m) *
                             AVOID_CM_PER_M;

        p_avoid->b_in_black = false;
        b_crossed = ((p_avoid->black_samples >= AVOID_BLACK_SAMPLES) &&
                     (run_cm >= AVOID_TAPE_MIN_CM) &&
                     (run_cm <= AVOID_TAPE_MAX_CM));
    }
    else
    {
        /* Still on white. */
    }

    return (b_crossed);
}

/*!
 * @brief Count consecutive black samples.
 *
 * @return true once a sensor has been on the line long enough to trust.
 */
static bool
avoid_line_seen (avoid_t * p_avoid, avoid_input_t const * p_input)
{
    p_avoid->black_samples = p_input->b_line_detected
                             ? (p_avoid->black_samples + 1U) : 0U;

    return (p_avoid->black_samples >= AVOID_BLACK_SAMPLES);
}

/*!
 * @brief Begin a spin-on-the-spot search, or give up after too many.
 */
static avoid_status_t
avoid_start_sweep (avoid_t * p_avoid, avoid_input_t const * p_input)
{
    avoid_status_t status = AVOID_RUNNING;

    if (p_avoid->sweep_count >= AVOID_SWEEP_MAX)
    {
        status = AVOID_FAILED_NO_LINE;
    }
    else
    {
        p_avoid->sweep_count++;
        avoid_enter(p_avoid, AVOID_PHASE_SWEEP, p_input);
    }

    return (status);
}

/*!
 * @brief Start a turn to a heading relative to heading0_deg.
 *
 * @return false if the car already points there (no command sent).
 */
static bool
avoid_begin_turn (avoid_t * p_avoid, avoid_input_t const * p_input,
                  avoid_output_t * p_output, float target_deg)
{
    float const turn_deg = target_deg - avoid_relative_deg(p_avoid, p_input);

    p_avoid->b_entered       = true;
    p_avoid->turn_target_deg = target_deg;

    if (rmath_absf(turn_deg) > AVOID_HEADING_TOL_DEG)
    {
        avoid_turn(p_output, turn_deg);
    }

    return (rmath_absf(turn_deg) > AVOID_HEADING_TOL_DEG);
}

/*!
 * @brief True once a turn has ended: the heading got there, subsystem 2
 *        reported it complete, or it ran out of time.  A turn that ends
 *        short is made up by the next one, which aims at a heading.
 */
static bool
avoid_turn_ended (avoid_t const * p_avoid, avoid_input_t const * p_input,
                  uint32_t timeout_ms)
{
    return ((rmath_absf(p_avoid->turn_target_deg -
                        avoid_relative_deg(p_avoid, p_input)) <=
             AVOID_HEADING_TOL_DEG) ||
            p_input->b_turn_done ||
            ((p_input->now_ms - p_avoid->phase_start_ms) >= timeout_ms));
}

/*!
 * @brief Turn to a heading (relative to heading0_deg); then the next
 *        phase.
 */
static void
avoid_step_turn (avoid_t * p_avoid, avoid_input_t const * p_input,
                 avoid_output_t * p_output, float target_deg,
                 avoid_phase_t next_phase)
{
    if (!p_avoid->b_entered)
    {
        if (!avoid_begin_turn(p_avoid, p_input, p_output, target_deg))
        {
            avoid_enter(p_avoid, next_phase, p_input);
        }
    }
    else if (avoid_turn_ended(p_avoid, p_input, AVOID_TURN_TIMEOUT_MS))
    {
        avoid_enter(p_avoid, next_phase, p_input);
    }
    else
    {
        /* Turn still running. */
    }
}

/*!
 * @brief A straight leg: when far enough, go to the next phase.
 */
static avoid_status_t
avoid_step_leg (avoid_t * p_avoid, avoid_input_t const * p_input,
                avoid_output_t * p_output, float speed, float length_cm,
                avoid_phase_t next_phase)
{
    avoid_status_t status = AVOID_RUNNING;

    if (!p_avoid->b_entered)
    {
        p_avoid->b_entered = true;
        avoid_drive(p_output, speed, speed);
    }
    else if (avoid_travelled_cm(p_avoid, p_input) >= length_cm)
    {
        avoid_enter(p_avoid, next_phase, p_input);
    }
    else if ((p_input->now_ms - p_avoid->phase_start_ms) >=
             AVOID_DRIVE_TIMEOUT_MS)
    {
        status = AVOID_FAILED_TIMEOUT;
    }
    else
    {
        /* Leg still running. */
    }

    return (status);
}

/*!
 * @brief Drive past the obstacle while the sensor watches its side.
 *        Done when the car's rear has cleared the obstacle's rear edge;
 *        if the side was never seen, after the assumed depth.
 */
static avoid_status_t
avoid_step_past (avoid_t * p_avoid, avoid_input_t const * p_input,
                 avoid_output_t * p_output)
{
    avoid_status_t status = AVOID_RUNNING;
    float const    max_along = p_avoid->decision_closest_cm +
                               AVOID_DEPTH_MAX_CM + AVOID_CAR_LENGTH_CM;

    if (!p_avoid->b_entered)
    {
        p_avoid->b_entered = true;
        avoid_drive(p_output, AVOID_LEG_SPEED_M_S, AVOID_LEG_SPEED_M_S);
    }
    else if ((p_avoid->b_rear_found &&
              (p_avoid->along_cm >= (p_avoid->rear_along_cm +
                                     AVOID_CAR_LENGTH_CM +
                                     AVOID_PAST_MARGIN_CM))) ||
             ((!p_avoid->b_side_seen) &&
              (p_avoid->along_cm >= p_avoid->past_cm)) ||
             (p_avoid->along_cm >= max_along))
    {
        avoid_enter(p_avoid, AVOID_PHASE_TURN_IN, p_input);
    }
    else if ((p_input->now_ms - p_avoid->phase_start_ms) >=
             AVOID_DRIVE_TIMEOUT_MS)
    {
        status = AVOID_FAILED_TIMEOUT;
    }
    else
    {
        /* Still beside the obstacle. */
    }

    return (status);
}

/*!
 * @brief True in the steps that drive forward, where something close
 *        ahead must stop the car.  Turns are left alone: the beam sweeps
 *        across the obstacle being passed.
 */
static bool
avoid_is_driving_forward (avoid_phase_t phase)
{
    return ((AVOID_PHASE_LEG_OUT == phase) ||
            (AVOID_PHASE_LEG_PAST == phase) ||
            (AVOID_PHASE_SEARCH == phase) ||
            (AVOID_PHASE_ADVANCE == phase) ||
            (AVOID_PHASE_VERIFY == phase));
}

/*!
 * @brief Stop for anything close ahead while driving (phase BLOCKED);
 *        carry on with the same step once the way has been clear for a
 *        few readings.
 */
static void
avoid_check_ahead (avoid_t * p_avoid, avoid_input_t const * p_input,
                   avoid_output_t * p_output, avoid_status_t * p_status)
{
    bool const b_fresh = p_avoid->b_ahead_fresh;

    p_avoid->b_ahead_fresh = false;

    if (AVOID_PHASE_BLOCKED == p_avoid->phase)
    {
        uint32_t const held_ms = p_input->now_ms - p_avoid->blocked_start_ms;

        if (b_fresh)
        {
            p_avoid->clear_count = (p_avoid->ahead_cm > AVOID_AHEAD_STOP_CM)
                                   ? (p_avoid->clear_count + 1U) : 0U;
        }

        if (p_avoid->clear_count >= AVOID_AHEAD_CLEAR_READINGS)
        {
            /* Same step as before, its clocks moved on by the wait. */
            p_avoid->phase           = p_avoid->blocked_phase;
            p_avoid->b_entered       = false;
            p_avoid->phase_start_ms += held_ms;
            p_avoid->white_since_ms += held_ms;
        }
        else if (held_ms >= AVOID_BLOCKED_TIMEOUT_MS)
        {
            *p_status = AVOID_FAILED_BLOCKED;
        }
        else
        {
            /* Still waiting for the way to clear. */
        }
    }
    else if (b_fresh && avoid_is_driving_forward(p_avoid->phase) &&
             (p_avoid->ahead_cm <= AVOID_AHEAD_STOP_CM))
    {
        p_avoid->blocked_phase    = p_avoid->phase;
        p_avoid->blocked_start_ms = p_input->now_ms;
        p_avoid->clear_count      = 0U;
        p_avoid->phase            = AVOID_PHASE_BLOCKED;
        p_output->type            = AVOID_OUT_STOP;
    }
    else
    {
        /* Nothing in the way. */
    }
}

/*!
 * @brief Enter the avoidance sequence: stop and wait for a fresh scan.
 */
void
avoid_start (avoid_t * p_avoid, uint32_t now_ms, float distance_m)
{
    float const    reverse_m = p_avoid->reverse_end_m;
    uint32_t const reverses  =
        ((rmath_absf(distance_m - reverse_m) * AVOID_CM_PER_M) <
         AVOID_RETRY_FORGET_CM) ? p_avoid->reverse_count : 0U;

    *p_avoid = (avoid_t) {0};
    p_avoid->phase          = AVOID_PHASE_WAIT_SCAN;
    p_avoid->phase_start_ms = now_ms;
    p_avoid->phase_start_m  = distance_m;
    p_avoid->reverse_count  = reverses;
    p_avoid->reverse_end_m  = reverse_m;
}

/*!
 * @brief Take a scan result.  Only a complete scan that started after
 *        the car stopped describes where the car is now.
 */
void
avoid_on_scan (avoid_t * p_avoid, obstacle_event_t const * p_obstacle)
{
    if ((AVOID_PHASE_WAIT_SCAN == p_avoid->phase) &&
        p_obstacle->b_scan_complete &&
        ((int32_t) (p_obstacle->scan_start_ms -
                    p_avoid->phase_start_ms) >= 0))
    {
        p_avoid->decision            = p_obstacle->action;
        p_avoid->decision_offset_cm  = p_obstacle->bypass_offset_cm;
        p_avoid->decision_closest_cm = p_obstacle->closest_distance_cm;
        p_avoid->b_decision          = true;
    }
}

/*!
 * @brief Take one reading from the sensor held toward the obstacle's
 *        side (avoid_look()); used at the next avoid_step().
 */
void
avoid_on_side_reading (avoid_t * p_avoid, float distance_cm)
{
    p_avoid->side_cm      = distance_cm;
    p_avoid->b_side_fresh = true;
}

/*!
 * @brief Take one straight-ahead reading; used at the next avoid_step().
 */
void
avoid_on_ahead_reading (avoid_t * p_avoid, float distance_cm)
{
    p_avoid->ahead_cm      = distance_cm;
    p_avoid->b_ahead_fresh = true;
}

/*!
 * @brief True while the controller should steer along the line itself.
 */
bool
avoid_is_following (avoid_t const * p_avoid)
{
    return (AVOID_PHASE_VERIFY == p_avoid->phase);
}

/*!
 * @brief Where the scanner should point: full scans while stopped to
 *        decide, the obstacle's side while driving past it until its
 *        rear edge is found, and straight ahead the rest of the time.
 */
avoid_look_t
avoid_look (avoid_t const * p_avoid)
{
    avoid_look_t look = AVOID_LOOK_AHEAD;

    switch (p_avoid->phase)
    {
        case AVOID_PHASE_IDLE:
        case AVOID_PHASE_WAIT_SCAN:
        case AVOID_PHASE_REVERSE:
            look = AVOID_LOOK_SCAN;
            break;

        case AVOID_PHASE_LEG_PAST:
            if (!p_avoid->b_rear_found)
            {
                /* Passing on the left puts the obstacle on the right. */
                look = (p_avoid->direction < 0) ? AVOID_LOOK_RIGHT
                                                : AVOID_LOOK_LEFT;
            }
            break;

        default:
            look = AVOID_LOOK_AHEAD;
            break;
    }

    return (look);
}

char const *
avoid_phase_name (avoid_phase_t phase)
{
    static char const * const names[] =
    {
        "IDLE", "WAIT_SCAN", "REVERSE", "TURN_OUT", "LEG_OUT",
        "TURN_PARALLEL", "LEG_PAST", "TURN_IN", "SEARCH", "ADVANCE",
        "ALIGN", "SWEEP", "VERIFY", "BLOCKED"
    };

    return (((uint32_t) phase < (sizeof(names) / sizeof(names[0])))
            ? names[phase] : "?");
}

/*!
 * @brief Advance the manoeuvre.  Call every controller cycle.
 *
 * @param[out] p_output  Motion command to send, if not AVOID_OUT_NONE.
 */
avoid_status_t
avoid_step (avoid_t * p_avoid, avoid_input_t const * p_input,
            avoid_output_t * p_output)
{
    float const    side   = (float) p_avoid->direction;
    uint32_t       age_ms;
    avoid_status_t status = AVOID_RUNNING;

    *p_output = (avoid_output_t) {0};

    if (0 != p_avoid->direction)
    {
        avoid_track_position(p_avoid, p_input);
        avoid_track_side(p_avoid, p_input);
    }

    avoid_check_ahead(p_avoid, p_input, p_output, &status);
    age_ms = p_input->now_ms - p_avoid->phase_start_ms;

    switch (p_avoid->phase)
    {
        case AVOID_PHASE_BLOCKED:
            /* Held stopped by avoid_check_ahead(). */
            break;

        case AVOID_PHASE_WAIT_SCAN:
            if (!p_avoid->b_entered)
            {
                p_avoid->b_entered = true;
                p_output->type     = AVOID_OUT_STOP;
            }
            else if (p_avoid->b_decision)
            {
                status = avoid_apply_decision(p_avoid, p_input);
            }
            else if (age_ms >= AVOID_SCAN_TIMEOUT_MS)
            {
                status = AVOID_FAILED_TIMEOUT;
            }
            else
            {
                /* Waiting for the scanner. */
            }
            break;

        case AVOID_PHASE_REVERSE:
            status = avoid_step_leg(p_avoid, p_input, p_output,
                                    -AVOID_LEG_SPEED_M_S, AVOID_REVERSE_CM,
                                    AVOID_PHASE_WAIT_SCAN);
            break;

        case AVOID_PHASE_TURN_OUT:
            avoid_step_turn(p_avoid, p_input, p_output,
                            -side * AVOID_TURN_DEG, AVOID_PHASE_LEG_OUT);
            break;

        case AVOID_PHASE_LEG_OUT:
            /* Out until the car is the planned distance to the side. */
            if (p_avoid->lateral_cm >= p_avoid->decision_offset_cm)
            {
                avoid_enter(p_avoid, AVOID_PHASE_TURN_PARALLEL, p_input);
            }
            else
            {
                status = avoid_step_leg(p_avoid, p_input, p_output,
                                        AVOID_LEG_SPEED_M_S,
                                        AVOID_OUT_SLACK *
                                        (p_avoid->decision_offset_cm /
                                         AVOID_SIN_45),
                                        AVOID_PHASE_TURN_PARALLEL);
            }
            break;

        case AVOID_PHASE_TURN_PARALLEL:
            avoid_step_turn(p_avoid, p_input, p_output, 0.0f,
                            AVOID_PHASE_LEG_PAST);
            break;

        case AVOID_PHASE_LEG_PAST:
            status = avoid_step_past(p_avoid, p_input, p_output);
            break;

        case AVOID_PHASE_TURN_IN:
            avoid_step_turn(p_avoid, p_input, p_output,
                            side * AVOID_TURN_DEG, AVOID_PHASE_SEARCH);
            break;

        case AVOID_PHASE_SEARCH:
            /* Back toward the line: it is where lateral_cm is 0. */
            if (!p_avoid->b_entered)
            {
                p_avoid->b_entered = true;
                avoid_drive(p_output, AVOID_SEARCH_SPEED_M_S,
                            AVOID_SEARCH_SPEED_M_S);
            }
            else if (avoid_tape_crossed(p_avoid, p_input))
            {
                float const black_start_m = p_avoid->black_start_m;

                avoid_enter(p_avoid, AVOID_PHASE_ADVANCE, p_input);
                p_avoid->phase_start_m = black_start_m;
            }
            else if ((!p_avoid->b_in_black) &&
                     ((p_avoid->lateral_cm <= -AVOID_SEARCH_OVERSHOOT_CM) ||
                      (avoid_travelled_cm(p_avoid, p_input) >=
                       AVOID_SEARCH_MAX_CM)))
            {
                status = avoid_start_sweep(p_avoid, p_input);
            }
            else if (age_ms >= AVOID_DRIVE_TIMEOUT_MS)
            {
                status = avoid_start_sweep(p_avoid, p_input);
            }
            else
            {
                /* Still searching. */
            }
            break;

        case AVOID_PHASE_ADVANCE:
            /* Still driving from SEARCH: bring the axle over the line, so
               that turning swings the sensors round onto it. */
            if (!p_avoid->b_entered)
            {
                /* Already moving, unless it was stopped while blocked. */
                p_avoid->b_entered = true;
                avoid_drive(p_output, AVOID_SEARCH_SPEED_M_S,
                            AVOID_SEARCH_SPEED_M_S);
            }

            if (avoid_travelled_cm(p_avoid, p_input) >= AVOID_SENSOR_AHEAD_CM)
            {
                avoid_enter(p_avoid, AVOID_PHASE_ALIGN, p_input);
            }
            else if (age_ms >= AVOID_DRIVE_TIMEOUT_MS)
            {
                status = AVOID_FAILED_TIMEOUT;
            }
            else
            {
                /* Still driving forward. */
            }
            break;

        case AVOID_PHASE_ALIGN:
        case AVOID_PHASE_SWEEP:
            /* ALIGN turns back to the original heading and a little past
               it, so the car joins the line going the way it was going;
               SWEEP turns a full circle.  Seeing the line ends either. */
            if (!p_avoid->b_entered)
            {
                (void) avoid_begin_turn(p_avoid, p_input, p_output,
                    (AVOID_PHASE_ALIGN == p_avoid->phase)
                    ? (-side * AVOID_ALIGN_OVERSHOOT_DEG)
                    : (avoid_relative_deg(p_avoid, p_input) -
                       (side * AVOID_SWEEP_DEG)));
            }
            else if (avoid_line_seen(p_avoid, p_input))
            {
                p_output->type = AVOID_OUT_STOP;    /* cancels the turn */
                avoid_enter(p_avoid, AVOID_PHASE_VERIFY, p_input);
            }
            else if (avoid_turn_ended(p_avoid, p_input,
                                      (AVOID_PHASE_ALIGN == p_avoid->phase)
                                      ? AVOID_TURN_TIMEOUT_MS
                                      : AVOID_SWEEP_TIMEOUT_MS))
            {
                status = avoid_start_sweep(p_avoid, p_input);
            }
            else
            {
                /* Still turning. */
            }
            break;

        case AVOID_PHASE_VERIFY:
            /* The controller is steering along the line. */
            if (p_input->b_line_detected)
            {
                p_avoid->white_since_ms = p_input->now_ms;
            }

            if ((p_input->now_ms - p_avoid->white_since_ms) >=
                AVOID_VERIFY_LOST_MS)
            {
                status = avoid_start_sweep(p_avoid, p_input);
            }
            else if (avoid_travelled_cm(p_avoid, p_input) >= AVOID_VERIFY_CM)
            {
                status = AVOID_DONE;
            }
            else
            {
                /* Not far enough yet. */
            }
            break;

        default:
            status = AVOID_DONE;
            break;
    }

    return (status);
}

/*** end of file ***/
