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
#define AVOID_ALIGN_DEG             (135.0f)
#define AVOID_SWEEP_DEG             (360.0f)    /* MOTION_TURN maximum */
#define AVOID_TURN_DEG              (45.0f)
#define AVOID_SIN_45                (0.7071f)
#define AVOID_REVERSE_CM            (12.0f)
#define AVOID_REVERSE_MAX           (3U)
#define AVOID_RETRY_FORGET_CM       (40.0f) /* drove clear of that spot */
#define AVOID_OBSTACLE_DEPTH_CM     (15.0f) /* the scan cannot see depth */
#define AVOID_CAR_LENGTH_CM         (16.0f)
#define AVOID_PAST_MIN_CM           (5.0f)
#define AVOID_SEARCH_MIN_CM         (20.0f)
#define AVOID_SEARCH_MAX_CM         (40.0f) /* stays on the tile mat */
#define AVOID_SENSOR_AHEAD_CM       (8.0f)  /* IR sensors ahead of axle */
#define AVOID_TAPE_MIN_CM           (1.0f)  /* 1.8 cm tape, thin bars out */
#define AVOID_TAPE_MAX_CM           (5.0f)  /* wide black patches out */
#define AVOID_BLACK_SAMPLES         (3U)    /* consecutive line events */
#define AVOID_SWEEP_MAX             (2U)
#define AVOID_VERIFY_CM             (10.0f)
#define AVOID_VERIFY_LOST_MS        (600U)
#define AVOID_SCAN_TIMEOUT_MS       (8000U)
#define AVOID_TURN_TIMEOUT_MS       (4000U)
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
}

static float
avoid_travelled_cm (avoid_t const * p_avoid, avoid_input_t const * p_input)
{
    return (rmath_absf(p_input->distance_m - p_avoid->phase_start_m) *
            AVOID_CM_PER_M);
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
            p_avoid->direction = (OBSTACLE_ACTION_TURN_LEFT ==
                                  p_avoid->decision) ? -1 : 1;
            p_avoid->out_cm    = p_avoid->decision_offset_cm / AVOID_SIN_45;
            p_avoid->past_cm   = rmath_maxf(AVOID_PAST_MIN_CM,
                                            p_avoid->decision_closest_cm +
                                            AVOID_OBSTACLE_DEPTH_CM +
                                            AVOID_CAR_LENGTH_CM -
                                            p_avoid->decision_offset_cm);
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
 * @brief A fixed turn: when done, go to the next phase.
 */
static avoid_status_t
avoid_step_turn (avoid_t * p_avoid, avoid_input_t const * p_input,
                 avoid_output_t * p_output, float angle_deg,
                 avoid_phase_t next_phase)
{
    avoid_status_t status = AVOID_RUNNING;

    if (!p_avoid->b_entered)
    {
        p_avoid->b_entered = true;
        avoid_turn(p_output, angle_deg);
    }
    else if (p_input->b_turn_done)
    {
        avoid_enter(p_avoid, next_phase, p_input);
    }
    else if ((p_input->now_ms - p_avoid->phase_start_ms) >=
             AVOID_TURN_TIMEOUT_MS)
    {
        status = AVOID_FAILED_TIMEOUT;
    }
    else
    {
        /* Turn still running. */
    }

    return (status);
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
 * @brief True while the controller should steer along the line itself.
 */
bool
avoid_is_following (avoid_t const * p_avoid)
{
    return (AVOID_PHASE_VERIFY == p_avoid->phase);
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
    uint32_t const age_ms = p_input->now_ms - p_avoid->phase_start_ms;
    avoid_status_t status = AVOID_RUNNING;

    *p_output = (avoid_output_t) {0};

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
                /* Waiting for the scanner. */
            }
            break;

        case AVOID_PHASE_REVERSE:
            status = avoid_step_leg(p_avoid, p_input, p_output,
                                    -AVOID_LEG_SPEED_M_S, AVOID_REVERSE_CM,
                                    AVOID_PHASE_WAIT_SCAN);
            break;

        case AVOID_PHASE_TURN_OUT:
            status = avoid_step_turn(p_avoid, p_input, p_output,
                                     -side * AVOID_TURN_DEG,
                                     AVOID_PHASE_LEG_OUT);
            break;

        case AVOID_PHASE_LEG_OUT:
            status = avoid_step_leg(p_avoid, p_input, p_output,
                                    AVOID_LEG_SPEED_M_S, p_avoid->out_cm,
                                    AVOID_PHASE_TURN_PARALLEL);
            break;

        case AVOID_PHASE_TURN_PARALLEL:
            status = avoid_step_turn(p_avoid, p_input, p_output,
                                     side * AVOID_TURN_DEG,
                                     AVOID_PHASE_LEG_PAST);
            break;

        case AVOID_PHASE_LEG_PAST:
            status = avoid_step_leg(p_avoid, p_input, p_output,
                                    AVOID_LEG_SPEED_M_S, p_avoid->past_cm,
                                    AVOID_PHASE_TURN_IN);
            break;

        case AVOID_PHASE_TURN_IN:
            status = avoid_step_turn(p_avoid, p_input, p_output,
                                     side * AVOID_TURN_DEG,
                                     AVOID_PHASE_SEARCH);
            break;

        case AVOID_PHASE_SEARCH:
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
            else if (((avoid_travelled_cm(p_avoid, p_input) >=
                       rmath_clampf(p_avoid->out_cm * 2.0f,
                                    AVOID_SEARCH_MIN_CM,
                                    AVOID_SEARCH_MAX_CM)) &&
                      (!p_avoid->b_in_black)) ||
                     (age_ms >= AVOID_DRIVE_TIMEOUT_MS))
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
               that spinning swings the sensors round onto it. */
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
            /* Turn back toward the original heading, so the car joins the
               line going the way it was going.  Subsystem 2's closed-loop
               turn limits how far; seeing the line cancels it early. */
            if (!p_avoid->b_entered)
            {
                p_avoid->b_entered = true;
                avoid_turn(p_output, -side *
                           ((AVOID_PHASE_ALIGN == p_avoid->phase)
                            ? AVOID_ALIGN_DEG : AVOID_SWEEP_DEG));
            }
            else if (avoid_line_seen(p_avoid, p_input))
            {
                p_output->type = AVOID_OUT_STOP;    /* cancels the turn */
                avoid_enter(p_avoid, AVOID_PHASE_VERIFY, p_input);
            }
            else if (p_input->b_turn_done ||
                     (age_ms >= AVOID_TURN_TIMEOUT_MS))
            {
                status = avoid_start_sweep(p_avoid, p_input);
            }
            else
            {
                /* Still spinning. */
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
