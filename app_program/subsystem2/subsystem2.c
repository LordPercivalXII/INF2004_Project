/** @file subsystem2.c
 *
 * @brief Subsystem 2 -- motion: creates its objects, starts its task.
 */

#include "subsystem2.h"
#include "../common/robot_config.h"

ID gh_motion_cmd_mbf = E_ID;

/*!
 * @brief Create the motion command message buffer.
 */
ER
subsystem2_init (void)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.maxmsz = (INT) sizeof(motion_command_t);
    cmbf.bufsz  = (SZ) APP_MBF_BUFSZ(ROBOT_QUEUE_LENGTH,
                                     sizeof(motion_command_t));
    gh_motion_cmd_mbf = tk_cre_mbf(&cmbf);

    return ((gh_motion_cmd_mbf < E_OK) ? gh_motion_cmd_mbf : E_OK);
}

/*!
 * @brief Start the 20 ms motion control task.
 */
ER
subsystem2_start (void)
{
    ID const tid = app_start_periodic_task((FP) motion_task, S2_PRI_MOTION,
                                           APP_CORE_ANY, "motion_task",
                                           ROBOT_MOTION_PERIOD_MS);

    return ((tid < E_OK) ? tid : E_OK);
}

/*** end of file ***/
