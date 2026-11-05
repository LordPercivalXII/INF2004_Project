/** @file avoidance.h
 *
 * @brief Avoidance manoeuvre and line recovery (stages 4 and 5): a state
 *        machine run by the controller, which owns the motors.  Pure
 *        logic: no kernel calls, so it can be tested without hardware.
 *
 * Sequence (d = bypass side):
 *   stop -> wait for a fresh complete scan -> act on its decision:
 *     CONTINUE       back to line following
 *     REVERSE_RETRY  back up, scan again (limited number of attempts)
 *     TURN_LEFT/RIGHT
 *       turn 45 deg toward d, drive out until the car is the planned
 *       distance to the side, turn back to the original heading, drive
 *       past while the sensor watches the obstacle's side (this measures
 *       its thickness) until the axle is past it, turn 70 deg toward the
 *       line (steep, to rejoin it soon after the obstacle), drive until a
 *       tape-width black run is crossed, drive the wheels over the line,
 *       turn back to the original heading until a sensor finds the line,
 *       then follow it a short way to confirm before handing control back.
 *     If the search fails: turn a full circle on the spot looking for it.
 *
 * Heading and position come from the wheel encoders.  Every turn aims at
 * a heading measured from where the car first stopped, so a short or long
 * turn is corrected by the next one, and the car knows how far it is from
 * the line it left.  The bypass assumes the line stays roughly straight
 * past the obstacle.
 *
 * Motion goes through subsystem 2: straight moves at its tested speed,
 * rotations as its closed-loop MOTION_TURN.  A turn ends when the heading
 * gets there or subsystem 2 reports it complete, whichever comes first.
 *
 * The scanner looks at the obstacle's side only while driving past it,
 * until its rear edge is found; the rest of the time it looks ahead.
 * Anything close ahead while driving stops the car (BLOCKED) until the
 * way is clear, then the interrupted step carries on.  If it is still
 * there after a second while driving past, it is a second obstacle: past
 * the first one already, the car turns in to the line; otherwise it scans
 * and goes round it further out on the same side.
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
    AVOID_PHASE_VERIFY,
    AVOID_PHASE_BLOCKED     /* stopped: something close ahead */
} avoid_phase_t;

/* A side reading at or below this is the obstacle beside the car. */
#define AVOID_SIDE_SEEN_CM  (50.0f)

typedef enum
{
    AVOID_RUNNING = 0,
    AVOID_DONE,             /* back on the line, or the path is clear */
    AVOID_FAILED_BOXED_IN,  /* out of reverse attempts                */
    AVOID_FAILED_NO_LINE,   /* search and sweeps found nothing        */
    AVOID_FAILED_TIMEOUT,   /* a scan or manoeuvre never finished     */
    AVOID_FAILED_BLOCKED    /* something ahead never cleared          */
} avoid_status_t;

typedef enum
{
    AVOID_OUT_NONE = 0,     /* keep the last command                  */
    AVOID_OUT_STOP,
    AVOID_OUT_DRIVE,        /* left_speed, right_speed                */
    AVOID_OUT_TURN          /* turn_deg, + = left, as MOTION_TURN     */
} avoid_output_type_t;

/* Where the scanner should point (see obstacle_set_look()). */
typedef enum
{
    AVOID_LOOK_SCAN = 0,    /* full coarse + fine scans               */
    AVOID_LOOK_LEFT,        /* left limit, with glances ahead         */
    AVOID_LOOK_RIGHT,       /* right limit, with glances ahead        */
    AVOID_LOOK_AHEAD        /* hold straight ahead                    */
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
    uint32_t          white_since_ms;
    float             phase_start_m;
    float             black_start_m;
    float             reverse_end_m;    /* odometer at the last reverse */
    float             turn_target_deg;  /* heading the turn aims at,
                                           relative to heading0_deg */
    uint32_t          black_samples;
    uint32_t          reverse_count;
    uint32_t          sweep_count;
    int8_t            direction;    /* -1 left, +1 right */
    bool              b_in_black;
    bool              b_entered;    /* phase's command was issued */
    bool              b_decision;   /* a fresh scan result is waiting */
    obstacle_action_t decision;
    float             decision_offset_cm;
    float             decision_closest_cm;

    /* Position since the decision: along = the original heading, lateral
     * = + toward the bypass side.  The sensor started at (0, 0) and the
     * obstacle's front face is at along = decision_closest_cm. */
    float             heading0_deg;
    float             last_m;
    float             along_cm;
    float             lateral_cm;

    /* Side readings while passing the obstacle. */
    bool              b_side_fresh;
    float             side_cm;
    bool              b_side_seen;      /* the obstacle's side was seen */
    bool              b_rear_found;     /* ...and then the sensor passed it */
    bool              b_rear_bounded;   /* side never seen: rear_along_cm
                                           and thickness_cm are maximums */
    uint32_t          side_gap_count;   /* far readings since the last hit */
    float             rear_along_cm;    /* furthest hit along the heading */
    float             thickness_cm;     /* 0 until the rear edge is found */

    /* Straight-ahead readings: stop for anything close while driving. */
    bool              b_ahead_fresh;
    float             ahead_cm;
    avoid_phase_t     blocked_phase;    /* step to carry on with */
    uint32_t          blocked_start_ms;
    uint32_t          clear_count;      /* clear readings while blocked */

    /* A second obstacle met while driving past the first. */
    bool              b_second;
    uint32_t          second_reverses;
    float             turn_in_min_cm;   /* along: the turn in also clears
                                           the first obstacle */
} avoid_t;

/* p_avoid must start zeroed.  Reverse attempts carry over between calls
 * until the car has driven clear of that spot, so a dead end cannot loop
 * forever between reversing and driving back. */
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
