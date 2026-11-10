/** @file avoidance.c
 *
 * @brief Buddy 5 avoidance and line recovery (see avoidance.h).
 *
 * The distances below are first guesses for the 13 cm car on the tile
 * mat; tune them from floor tests (testing/subsystem5/TEST_LOG.md).
 */

#include "avoidance.h"
#include "../common/robot_config.h"
#include "../common/rmath.h"

/* Speeds: slow enough for the sensor to see something ahead in time. */
#define AVOID_SPEED_M_S         (0.12f)

/* Turns. */
#define AVOID_TURN_OUT_DEG      (60.0f) /* out: steep, so the car gets
                                           sideways in a short distance
                                           and need not back up        */
#define AVOID_TAN_OUT           (1.7321f)   /* tan 60 */
#define AVOID_SIN_OUT           (0.8660f)   /* sin 60 */
#define AVOID_TURN_IN_DEG       (45.0f) /* back in                     */
#define AVOID_ALIGN_PAST_DEG    (45.0f) /* past the original heading   */
#define AVOID_SWEEP_DEG         (360.0f)
#define AVOID_SWEEP_MAX         (2U)
#define AVOID_HEADING_TOL_DEG   (4.0f)  /* a turn this close is done   */

/* Backing up. */
#define AVOID_REVERSE_CM        (10.0f) /* to scan again               */
#define AVOID_ROOM_MAX_CM       (25.0f) /* longest back-up for room    */
#define AVOID_REVERSE_MAX       (3U)
#define AVOID_RETRY_FORGET_CM   (40.0f) /* drove clear of that spot    */

/* The bypass. */
#define AVOID_ROOM_MARGIN_CM    (5.0f)  /* front distance needed = the
                                           forward part of the way out
                                           + this (the nose is 5 cm
                                           ahead of the sensor)        */
#define AVOID_DEPTH_CM          (20.0f) /* assumed obstacle depth      */
#define AVOID_PAST_CM           (17.0f) /* sensor past the rear before
                                           turning in: the car's rear
                                           is ~11 cm behind the sensor,
                                           plus margin                 */
#define AVOID_OUT_SLACK         (1.5f)  /* LEG_OUT length limit factor */
#define AVOID_AHEAD_STOP_CM     (10.0f) /* this close ahead: blocked   */
#define AVOID_NEW_OBSTACLE_CM   (20.0f) /* ahead while confirming the
                                           line: a new obstacle        */

/* Finding the line again. */
#define AVOID_SEARCH_OVERSHOOT_CM (10.0f) /* past the line, not seen  */
#define AVOID_SEARCH_MAX_CM     (60.0f)
#define AVOID_SENSOR_AHEAD_CM   (8.0f)  /* line sensors ahead of axle  */
#define AVOID_TAPE_MIN_CM       (1.0f)  /* 1.8 cm tape                 */
#define AVOID_TAPE_MAX_CM       (5.0f)
#define AVOID_BLACK_SAMPLES     (3U)    /* line events in a row        */
#define AVOID_VERIFY_CM         (10.0f)
#define AVOID_VERIFY_LOST_MS    (600U)

/* Time limits. */
#define AVOID_SCAN_TIMEOUT_MS   (10000U)
#define AVOID_TURN_TIMEOUT_MS   (4000U)
#define AVOID_SWEEP_TIMEOUT_MS  (8000U)
#define AVOID_DRIVE_TIMEOUT_MS  (12000U)

#define AVOID_CM_PER_M          (100.0f)

/* The simulated line never ends, so in simulation a few black samples
 * count as crossing it. */
#ifndef AVOID_SIM_LINE
#define AVOID_SIM_LINE          (ROBOT_SIMULATION)
#endif

/*!
 * @brief Start a phase.
 */
static void
avoid_enter (avoid_t * p_avoid, avoid_phase_t phase,
             avoid_input_t const * p_input)
{
    p_avoid->phase          = phase;
    p_avoid->phase_start_ms = p_input->now_ms;
    p_avoid->phase_start_m  = p_input->distance_m;
    p_avoid->b_entered      = false;
    p_avoid->b_in_black     = false;
    p_avoid->black_samples  = 0U;
    p_avoid->white_since_ms = p_input->now_ms;
    p_avoid->b_ahead_fresh  = false;
}

static float
avoid_travelled_cm (avoid_t const * p_avoid, avoid_input_t const * p_input)
{
    return (rmath_absf(p_input->distance_m - p_avoid->phase_start_m) *
            AVOID_CM_PER_M);
}

static float
avoid_relative_deg (avoid_t const * p_avoid, avoid_input_t const * p_input)
{
    return (p_input->heading_deg - p_avoid->heading0_deg);
}

static void
avoid_drive (avoid_output_t * p_output, float speed)
{
    p_output->type        = AVOID_OUT_DRIVE;
    p_output->left_speed  = speed;
    p_output->right_speed = speed;
}

/*!
 * @brief Dead reckoning: each bit of distance moves the car along its
 *        current heading.
 */
static void
avoid_track (avoid_t * p_avoid, avoid_input_t const * p_input)
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
 * @brief A turn to a heading (relative to where the car stopped), then
 *        the next phase.  It ends when the heading gets there, subsystem
 *        2 reports the turn complete, or it runs out of time.
 */
static void
avoid_step_turn (avoid_t * p_avoid, avoid_input_t const * p_input,
                 avoid_output_t * p_output, float target_deg,
                 avoid_phase_t next_phase)
{
    float const turn_deg = target_deg - avoid_relative_deg(p_avoid, p_input);

    if (!p_avoid->b_entered)
    {
        p_avoid->b_entered       = true;
        p_avoid->turn_target_deg = target_deg;

        if (rmath_absf(turn_deg) > AVOID_HEADING_TOL_DEG)
        {
            p_output->type     = AVOID_OUT_TURN;
            p_output->turn_deg = turn_deg;
        }
        else
        {
            avoid_enter(p_avoid, next_phase, p_input);
        }
    }
    else if ((rmath_absf(turn_deg) <= AVOID_HEADING_TOL_DEG) ||
             p_input->b_turn_done ||
             ((p_input->now_ms - p_avoid->phase_start_ms) >=
              AVOID_TURN_TIMEOUT_MS))
    {
        avoid_enter(p_avoid, next_phase, p_input);
    }
    else
    {
        /* Still turning. */
    }
}

/*!
 * @brief A straight drive of a set length (negative speed: backwards),
 *        then the next phase.
 */
static avoid_status_t
avoid_step_drive (avoid_t * p_avoid, avoid_input_t const * p_input,
                  avoid_output_t * p_output, float speed, float length_cm,
                  avoid_phase_t next_phase)
{
    avoid_status_t status = AVOID_RUNNING;

    if (!p_avoid->b_entered)
    {
        p_avoid->b_entered = true;
        avoid_drive(p_output, speed);
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
        /* Still driving. */
    }

    return (status);
}

/*!
 * @brief Back up, then go on to the given phase.
 */
static void
avoid_begin_reverse (avoid_t * p_avoid, avoid_input_t const * p_input,
                     float length_cm, avoid_phase_t after)
{
    p_avoid->reverse_cm    = length_cm;
    p_avoid->after_reverse = after;
    avoid_enter(p_avoid, AVOID_PHASE_REVERSE, p_input);
}

/*!
 * @brief Act on the scan's decision.
 */
static avoid_status_t
avoid_apply_decision (avoid_t * p_avoid, avoid_input_t const * p_input)
{
    avoid_status_t status = AVOID_RUNNING;
    float          room_cm;

    p_avoid->b_decision = false;

    switch (p_avoid->decision)
    {
        case OBSTACLE_ACTION_CONTINUE:
            status = AVOID_DONE;
            break;

        case OBSTACLE_ACTION_REVERSE_RETRY:
            if (p_avoid->reverse_count >= AVOID_REVERSE_MAX)
            {
                status = AVOID_FAILED_BOXED_IN;
            }
            else
            {
                p_avoid->reverse_count++;
                avoid_begin_reverse(p_avoid, p_input, AVOID_REVERSE_CM,
                                    AVOID_PHASE_WAIT_SCAN);
            }
            break;

        case OBSTACLE_ACTION_TURN_LEFT:
        case OBSTACLE_ACTION_TURN_RIGHT:
            p_avoid->direction    = (OBSTACLE_ACTION_TURN_LEFT ==
                                     p_avoid->decision) ? -1 : 1;
            p_avoid->heading0_deg = p_input->heading_deg;
            p_avoid->last_m       = p_input->distance_m;
            p_avoid->along_cm     = 0.0f;
            p_avoid->lateral_cm   = 0.0f;

            /* Driving out at 60 deg moves the car forward offset / tan 60:
               back up first if the obstacle is too close for that. */
            room_cm = ((p_avoid->decision_offset_cm / AVOID_TAN_OUT) +
                       AVOID_ROOM_MARGIN_CM) - p_avoid->decision_closest_cm;

            if (room_cm > 0.0f)
            {
                avoid_begin_reverse(p_avoid, p_input,
                                    rmath_minf(room_cm, AVOID_ROOM_MAX_CM),
                                    AVOID_PHASE_TURN_OUT);
            }
            else
            {
                avoid_enter(p_avoid, AVOID_PHASE_TURN_OUT, p_input);
            }
            break;

        default:
            /* STOP: stay stopped and wait for the next scan. */
            break;
    }

    return (status);
}

/*!
 * @brief True once the sensors have crossed a tape-width black run (or,
 *        in simulation, seen black a few times).
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
 * @brief True once a sensor has been on the line long enough to trust.
 */
static bool
avoid_line_seen (avoid_t * p_avoid, avoid_input_t const * p_input)
{
    p_avoid->black_samples = p_input->b_line_detected
                             ? (p_avoid->black_samples + 1U) : 0U;

    return (p_avoid->black_samples >= AVOID_BLACK_SAMPLES);
}

/*!
 * @brief Spin on the spot looking for the line, or give up.
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
 * @brief A turn that ends early when the line is seen (ALIGN, SWEEP).
 */
static avoid_status_t
avoid_step_find (avoid_t * p_avoid, avoid_input_t const * p_input,
                 avoid_output_t * p_output, float target_deg,
                 uint32_t timeout_ms)
{
    avoid_status_t status   = AVOID_RUNNING;
    float const    turn_deg = target_deg -
                              avoid_relative_deg(p_avoid, p_input);

    if (!p_avoid->b_entered)
    {
        p_avoid->b_entered       = true;
        p_avoid->turn_target_deg = target_deg;
        p_output->type           = AVOID_OUT_TURN;
        p_output->turn_deg       = turn_deg;
    }
    else if (avoid_line_seen(p_avoid, p_input))
    {
        p_output->type = AVOID_OUT_STOP;    /* cancels the turn */
        avoid_enter(p_avoid, AVOID_PHASE_VERIFY, p_input);
    }
    else if ((rmath_absf(p_avoid->turn_target_deg -
                         avoid_relative_deg(p_avoid, p_input)) <=
              AVOID_HEADING_TOL_DEG) ||
             p_input->b_turn_done ||
             ((p_input->now_ms - p_avoid->phase_start_ms) >= timeout_ms))
    {
        status = avoid_start_sweep(p_avoid, p_input);
    }
    else
    {
        /* Still turning. */
    }

    return (status);
}

/*!
 * @brief True in the phases that drive forward, where something close
 *        ahead must stop the car.
 */
static bool
avoid_is_driving_forward (avoid_phase_t phase)
{
    return ((AVOID_PHASE_LEG_OUT == phase) ||
            (AVOID_PHASE_LEG_PAST == phase) ||
            (AVOID_PHASE_SEARCH == phase) ||
            (AVOID_PHASE_ADVANCE == phase));
}

void
avoid_start (avoid_t * p_avoid, uint32_t now_ms, float distance_m)
{
    float const    reverse_end_m = p_avoid->reverse_end_m;
    uint32_t const reverses      =
        ((rmath_absf(distance_m - reverse_end_m) * AVOID_CM_PER_M) <
         AVOID_RETRY_FORGET_CM) ? p_avoid->reverse_count : 0U;

    *p_avoid = (avoid_t) {0};
    p_avoid->phase          = AVOID_PHASE_WAIT_SCAN;
    p_avoid->phase_start_ms = now_ms;
    p_avoid->phase_start_m  = distance_m;
    p_avoid->reverse_count  = reverses;
    p_avoid->reverse_end_m  = reverse_end_m;
}

/*!
 * @brief Take a scan result: only a complete one, started after the car
 *        stopped (so it was taken from where the car is now).
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
 * @brief Side readings are not used by this version.
 */
void
avoid_on_side_reading (avoid_t * p_avoid, float distance_cm)
{
    (void) p_avoid;
    (void) distance_cm;
}

/*!
 * @brief One straight-ahead reading while moving.
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
 *        decide, straight ahead the rest of the time.
 */
avoid_look_t
avoid_look (avoid_t const * p_avoid)
{
    return (((AVOID_PHASE_IDLE == p_avoid->phase) ||
             (AVOID_PHASE_WAIT_SCAN == p_avoid->phase))
            ? AVOID_LOOK_SCAN : AVOID_LOOK_AHEAD);
}

char const *
avoid_phase_name (avoid_phase_t phase)
{
    static char const * const names[] =
    {
        "IDLE", "WAIT_SCAN", "REVERSE", "TURN_OUT", "LEG_OUT",
        "TURN_PARALLEL", "LEG_PAST", "TURN_IN", "SEARCH", "ADVANCE",
        "ALIGN", "SWEEP", "VERIFY"
    };

    return (((uint32_t) phase < (sizeof(names) / sizeof(names[0])))
            ? names[phase] : "?");
}

/*!
 * @brief One control step.  The controller calls it every cycle while
 *        avoiding and carries out p_output.
 */
avoid_status_t
avoid_step (avoid_t * p_avoid, avoid_input_t const * p_input,
            avoid_output_t * p_output)
{
    avoid_status_t status   = AVOID_RUNNING;
    uint32_t const age_ms   = p_input->now_ms - p_avoid->phase_start_ms;
    float const    side     = (float) p_avoid->direction;

    *p_output = (avoid_output_t) {0};

    if (0 != p_avoid->direction)
    {
        avoid_track(p_avoid, p_input);
    }

    /* Something close ahead while driving forward: stop, give up. */
    if (avoid_is_driving_forward(p_avoid->phase) && p_avoid->b_entered &&
        p_avoid->b_ahead_fresh &&
        (p_avoid->ahead_cm <= AVOID_AHEAD_STOP_CM))
    {
        p_output->type = AVOID_OUT_STOP;
        return (AVOID_FAILED_BLOCKED);
    }

    p_avoid->b_ahead_seen  = p_avoid->b_ahead_fresh;
    p_avoid->b_ahead_fresh = false;

    switch (p_avoid->phase)
    {
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
                /* Waiting for the scan. */
            }
            break;

        case AVOID_PHASE_REVERSE:
            status = avoid_step_drive(p_avoid, p_input, p_output,
                                      -AVOID_SPEED_M_S, p_avoid->reverse_cm,
                                      p_avoid->after_reverse);

            if (p_avoid->phase != AVOID_PHASE_REVERSE)
            {
                p_avoid->reverse_end_m = p_input->distance_m;
            }
            break;

        case AVOID_PHASE_TURN_OUT:
            avoid_step_turn(p_avoid, p_input, p_output,
                            -side * AVOID_TURN_OUT_DEG, AVOID_PHASE_LEG_OUT);
            break;

        case AVOID_PHASE_LEG_OUT:
            if (p_avoid->lateral_cm >= p_avoid->decision_offset_cm)
            {
                avoid_enter(p_avoid, AVOID_PHASE_TURN_PARALLEL, p_input);
            }
            else
            {
                status = avoid_step_drive(p_avoid, p_input, p_output,
                                          AVOID_SPEED_M_S,
                                          AVOID_OUT_SLACK *
                                          (p_avoid->decision_offset_cm /
                                           AVOID_SIN_OUT),
                                          AVOID_PHASE_TURN_PARALLEL);
            }
            break;

        case AVOID_PHASE_TURN_PARALLEL:
            avoid_step_turn(p_avoid, p_input, p_output, 0.0f,
                            AVOID_PHASE_LEG_PAST);
            break;

        case AVOID_PHASE_LEG_PAST:
            if (p_avoid->along_cm >= (p_avoid->decision_closest_cm +
                                      AVOID_DEPTH_CM + AVOID_PAST_CM))
            {
                avoid_enter(p_avoid, AVOID_PHASE_TURN_IN, p_input);
            }
            else if (!p_avoid->b_entered)
            {
                p_avoid->b_entered = true;
                avoid_drive(p_output, AVOID_SPEED_M_S);
            }
            else if (age_ms >= AVOID_DRIVE_TIMEOUT_MS)
            {
                status = AVOID_FAILED_TIMEOUT;
            }
            else
            {
                /* Still driving past. */
            }
            break;

        case AVOID_PHASE_TURN_IN:
            avoid_step_turn(p_avoid, p_input, p_output,
                            side * AVOID_TURN_IN_DEG, AVOID_PHASE_SEARCH);
            break;

        case AVOID_PHASE_SEARCH:
            /* Toward the line: it is where lateral_cm is 0. */
            if (!p_avoid->b_entered)
            {
                p_avoid->b_entered = true;
                avoid_drive(p_output, AVOID_SPEED_M_S);
            }
            else if (avoid_tape_crossed(p_avoid, p_input))
            {
                float const black_start_m = p_avoid->black_start_m;

                avoid_enter(p_avoid, AVOID_PHASE_ADVANCE, p_input);
                p_avoid->b_entered     = true;  /* still driving */
                p_avoid->phase_start_m = black_start_m;
            }
            else if ((!p_avoid->b_in_black) &&
                     ((p_avoid->lateral_cm <= -AVOID_SEARCH_OVERSHOOT_CM) ||
                      (avoid_travelled_cm(p_avoid, p_input) >=
                       AVOID_SEARCH_MAX_CM) ||
                      (age_ms >= AVOID_DRIVE_TIMEOUT_MS)))
            {
                status = avoid_start_sweep(p_avoid, p_input);
            }
            else
            {
                /* Still searching. */
            }
            break;

        case AVOID_PHASE_ADVANCE:
            /* Bring the wheels over the line, so turning swings the
               sensors round onto it. */
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
                /* Still driving. */
            }
            break;

        case AVOID_PHASE_ALIGN:
            /* Back to the original heading and a little past it, so the
               car joins the line going the way it was going. */
            status = avoid_step_find(p_avoid, p_input, p_output,
                                     -side * AVOID_ALIGN_PAST_DEG,
                                     AVOID_TURN_TIMEOUT_MS);
            break;

        case AVOID_PHASE_SWEEP:
            status = avoid_step_find(p_avoid, p_input, p_output,
                                     p_avoid->b_entered
                                     ? p_avoid->turn_target_deg
                                     : (avoid_relative_deg(p_avoid, p_input) -
                                        (side * AVOID_SWEEP_DEG)),
                                     AVOID_SWEEP_TIMEOUT_MS);
            break;

        case AVOID_PHASE_VERIFY:
            /* The controller steers along the line.  Something close
               ahead is a new obstacle: stop and scan it. */
            if (p_avoid->b_ahead_seen &&
                (p_avoid->ahead_cm <= AVOID_NEW_OBSTACLE_CM))
            {
                p_avoid->direction = 0;
                avoid_enter(p_avoid, AVOID_PHASE_WAIT_SCAN, p_input);
                break;
            }

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
