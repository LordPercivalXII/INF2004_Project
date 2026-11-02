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
 *       turn 45 deg toward d, drive out, turn back to parallel, drive
 *       past, turn 45 deg back toward the line, creep until a tape-width
 *       black run is crossed, drive the wheels over the line, spin toward
 *       the original heading until a sensor finds the line, then follow
 *       it a short way to confirm before handing control back.
 *     If the search fails: spin on the spot looking for the line.
 *
 * All distances come from the wheel encoders; there is no absolute
 * position or heading, so the bypass assumes the line stays roughly
 * straight past the obstacle.
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

typedef enum
{
    AVOID_RUNNING = 0,
    AVOID_DONE,             /* back on the line, or the path is clear */
    AVOID_FAILED_BOXED_IN,  /* out of reverse attempts                */
    AVOID_FAILED_NO_LINE,   /* search and sweeps found nothing        */
    AVOID_FAILED_TIMEOUT    /* a scan or manoeuvre never finished     */
} avoid_status_t;

typedef enum
{
    AVOID_OUT_NONE = 0,     /* keep the last command                  */
    AVOID_OUT_STOP,
    AVOID_OUT_DRIVE,        /* left_speed, right_speed                */
    AVOID_OUT_TURN          /* turn_deg, + = left, as MOTION_TURN     */
} avoid_output_type_t;

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
    float             out_cm;
    float             past_cm;
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
} avoid_t;

/* p_avoid must start zeroed.  Reverse attempts carry over between calls
 * until the car has driven clear of that spot, so a dead end cannot loop
 * forever between reversing and driving back. */
void           avoid_start(avoid_t * p_avoid, uint32_t now_ms,
                           float distance_m);
void           avoid_on_scan(avoid_t * p_avoid,
                             obstacle_event_t const * p_obstacle);
avoid_status_t avoid_step(avoid_t * p_avoid, avoid_input_t const * p_input,
                          avoid_output_t * p_output);
bool           avoid_is_following(avoid_t const * p_avoid);

#endif /* AVOIDANCE_H */

/*** end of file ***/
