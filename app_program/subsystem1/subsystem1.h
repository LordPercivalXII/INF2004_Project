/** @file subsystem1.h
 *
 * @brief Subsystem 1 -- Buddy 1: telemetry.  Student ID: <student ID>.
 *        Port of src/buddy1_telemetry/.
 *
 * Public interface (TEAM.md section 5):
 *   IF-04  controller -> S1: telemetry snapshots, message buffer
 *          gh_telemetry_mbf, sent with telemetry_publish().
 *   IF-02  S1 -> controller: user commands; telemetry_submit_command()
 *          forwards them to the controller's gh_control_mbf.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "../common/robot_types.h"

#define S1_PRI_TELEMETRY        (20)    /* band 20-21: lowest (TEAM.md) */
#define S1_RECEIVE_TIMEOUT_MS   (100U)
#define S1_FIXED_TEXT_MAX       (16U)   /* "-1000000.000" + NUL fits */

extern ID gh_telemetry_mbf;

bool telemetry_publish(telemetry_message_t const * p_message);
bool telemetry_submit_command(control_command_t const * p_command);

/* Private to subsystem 1 (public only for subsystem1_test.c). */
void telemetry_task(INT stacd, void * p_exinf);
void telemetry_format_fixed(char * p_text, float value, uint32_t decimals);

#endif /* SUBSYSTEM1_H */

/*** end of file ***/
