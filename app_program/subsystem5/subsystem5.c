/** @file subsystem5.c
 *
 * @brief Subsystem 5 -- obstacle: creates its objects, starts its task.
 */

#include "subsystem5.h"
#include "../common/robot_config.h"

ID gh_echo_mbf = E_ID;

/*!
 * @brief Create the echo message buffer (was a FreeRTOS queue created
 *        inside obstacle_hardware_init()).
 */
ER
subsystem5_init (void)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.maxmsz = (INT) sizeof(uint32_t);
    cmbf.bufsz  = (SZ) APP_MBF_BUFSZ(ULTRASONIC_ECHO_QUEUE_LENGTH,
                                     sizeof(uint32_t));
    gh_echo_mbf = tk_cre_mbf(&cmbf);

    return ((gh_echo_mbf < E_OK) ? gh_echo_mbf : E_OK);
}

ER
subsystem5_start (void)
{
    ID const tid = app_start_task((FP) obstacle_task, S5_PRI_OBSTACLE,
                                  APP_CORE_ANY, "obstacle_task");

    return ((tid < E_OK) ? tid : E_OK);
}

/*** end of file ***/
