/** @file avoidance.h
 *
 * @brief Buddy 5 stage 4 (acting on the decision) and stage 5 (line
 *        recovery): a state machine the controller runs, because the
 *        controller owns the motors.  Pure logic, no kernel calls, so it
 *        is unit-tested on the PC.
 *
 * Sequence (side = the way round, d = the planned sideways offset):
 *
 *   WAIT_SCAN      stopped; wait for a complete scan taken after the stop
 *     CONTINUE       -> done: back to line following
 *     REVERSE_RETRY  -> REVERSE 10 cm, then WAIT_SCAN again (3 at most,
 *                       then give up: boxed in)
 *     TURN_LEFT/RIGHT:
 *   REVERSE        only if the way out would not clear the obstacle:
 *                  back up just enough
 *   TURN_OUT       60 deg toward the side
 *   LEG_OUT        drive until the car is d to the side
 *   TURN_PARALLEL  back to the original heading
 *   LEG_PAST       drive past: the obstacle's front, an assumed depth
 *                  and the car's length
 *   TURN_IN        45 deg back toward the line
 *   SEARCH         drive until the sensors cross the line (tape width)
 *   ADVANCE        bring the wheels over the line
 *   ALIGN          turn back past the original heading until a sensor
 *                  is on the line
 *   VERIFY         the controller follows the line a short way; then done
 *                  (something within 20 cm ahead: a new obstacle, so
 *                  stop and go back to WAIT_SCAN)
 *   SWEEP          if the line is not found: turn a full circle looking
 *                  for it (twice at most, then give up: no line)
 *
 * Heading and position come from the wheel encoders, measured from where
 * the car stopped, so the car knows how far it is from the line it left.
 * Every turn aims at a heading, so a short turn is made up by the next.
 *
 * Safety: while driving forward the sensor looks straight ahead; anything
 * within 10 cm stops the car and ends the manoeuvre (blocked).
 */

#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_types.h"

typedef enum
{
    AVOID_PHASE_IDLE = 0,
    AVOID_PHASE_WAIT_SCAN,
    AVOID_PHASE_REVERSE,
    AVOID_PHASE_TURN_OUT,
    AVOID_PHASE_LEG_OUT,
    AVOID_PHASE_TURN_PARALLEL,
    AVOID_PHASE_LEG_PAST,
    AVOID_PHASE_TURN_IN,
    AVOID_PHASE_SEARCH,
    AVOID_PHASE_ADVANCE,
    AVOID_PHASE_ALIGN,
    AVOID_PHASE_SWEEP,
    AVOID_PHASE_VERIFY
} avoid_phase_t;

/* The controller prints these, in this order. */
typedef enum
{
    AVOID_RUNNING = 0,
    AVOID_DONE,             /* back on the line, or nothing in the way */
    AVOID_FAILED_BOXED_IN,  /* out of reverse attempts                 */
    AVOID_FAILED_NO_LINE,   /* search and sweeps found nothing         */
    AVOID_FAILED_TIMEOUT,   /* a scan or a move never finished         */
    AVOID_FAILED_BLOCKED    /* something close ahead while driving     */
} avoid_status_t;

typedef enum
{
    AVOID_OUT_NONE = 0,     /* keep the last command                   */
    AVOID_OUT_STOP,
    AVOID_OUT_DRIVE,        /* left_speed, right_speed                 */
    AVOID_OUT_TURN          /* turn_deg, + = left, as MOTION_TURN      */
} avoid_output_type_t;

/* Where the scanner should point (obstacle_set_look()). */
typedef enum
{
    AVOID_LOOK_SCAN = 0,    /* stopped: full coarse + fine scans       */
    AVOID_LOOK_LEFT,        /* not used by this version                */
    AVOID_LOOK_RIGHT,       /* not used by this version                */
    AVOID_LOOK_AHEAD        /* moving: straight ahead, one at a time   */
} avoid_look_t;

typedef struct
{
    avoid_output_type_t type;
    float               left_speed;
    float               right_speed;
    float               turn_deg;
} avoid_output_t;

typedef struct
{
    uint32_t now_ms;
    float    distance_m;        /* motion event: signed odometer       */
    float    heading_deg;       /* from the encoders, + = left         */
    bool     b_line_detected;   /* latest line event                   */
    bool     b_turn_done;       /* no MOTION_TURN is still running     */
} avoid_input_t;

typedef struct
{
    avoid_phase_t     phase;
    uint32_t          phase_start_ms;
    float             phase_start_m;
    bool              b_entered;        /* phase's command was issued  */

    /* The scan's decision. */
    bool              b_decision;
    obstacle_action_t decision;
    float             decision_offset_cm;
    float             decision_closest_cm;
    int8_t            direction;        /* -1 left, +1 right           */

    /* Backing up: how far, and what comes after. */
    float             reverse_cm;
    avoid_phase_t     after_reverse;
    uint32_t          reverse_count;
    float             reverse_end_m;    /* odometer after the last one */

    /* Position since the decision, from the encoders: along = the
     * original heading, lateral = + toward the bypass side.  The sensor
     * started at (0, 0); the obstacle's front is at decision_closest_cm. */
    float             heading0_deg;
    float             last_m;
    float             along_cm;
    float             lateral_cm;
    float             turn_target_deg;  /* relative to heading0_deg    */

    /* Line recovery. */
    bool              b_in_black;
    float             black_start_m;
    uint32_t          black_samples;
    uint32_t          white_since_ms;
    uint32_t          sweep_count;

    /* Latest straight-ahead reading while moving. */
    bool              b_ahead_fresh;
    bool              b_ahead_seen;     /* this step has a new reading */
    float             ahead_cm;

    /* Printed by the controller; this version does not measure the
     * obstacle's depth, so they stay false / 0. */
    bool              b_side_seen;
    bool              b_rear_found;
    bool              b_rear_bounded;
    float             thickness_cm;
} avoid_t;

/* Reverse attempts carry over between calls until the car has driven
 * clear of that spot, so a dead end cannot loop forever between backing
 * up and driving back. */
void           avoid_start(avoid_t * p_avoid, uint32_t now_ms,
                           float distance_m);
void           avoid_on_scan(avoid_t * p_avoid,
                             obstacle_event_t const * p_obstacle);
void           avoid_on_side_reading(avoid_t * p_avoid, float distance_cm);
void           avoid_on_ahead_reading(avoid_t * p_avoid, float distance_cm);
avoid_status_t avoid_step(avoid_t * p_avoid, avoid_input_t const * p_input,
                          avoid_output_t * p_output);
bool           avoid_is_following(avoid_t const * p_avoid);
avoid_look_t   avoid_look(avoid_t const * p_avoid);
char const *   avoid_phase_name(avoid_phase_t phase);

#endif /* AVOIDANCE_H */

/*** end of file ***/
