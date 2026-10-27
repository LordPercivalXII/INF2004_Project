/** @file controller.h
 *
 * @brief Vehicle controller: the sole owner of the mission state.
 *        Port of src/controller/controller.c.  Owned by the whole team.
 *
 * Public interface (TEAM.md section 5):
 *   IF-01  any subsystem -> controller: system events, message buffer
 *          gh_event_mbf, sent with controller_post_event().
 *   IF-02  S1 -> controller: user commands, message buffer
 *          gh_control_mbf (control_command_t).
 *   IF-05  controller -> any: status bits, event flag gh_system_flg
 *          (SYSTEM_BIT_xxx).  Read with tk_ref_flg or wait with
 *          tk_wai_flg; only the controller sets or clears bits.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../app.h"
#include "../common/robot_types.h"

#define CTRL_PRI_MAIN       (12)    /* priority band 12-13 (TEAM.md) */

extern ID gh_event_mbf;
extern ID gh_control_mbf;
extern ID gh_system_flg;

bool controller_post_event(system_event_t const * p_event);

/* Private to the controller. */
void controller_task(INT stacd, void * p_exinf);

#endif /* CONTROLLER_H */

/*** end of file ***/
