/** @file subsystem4.c
 *
 * @brief Subsystem 4 -- IMU: starts its 10 ms task.  No shared objects.
 */

#include "subsystem4.h"

ER
subsystem4_init (void)
{
    return (E_OK);
}

ER
subsystem4_start (void)
{
    ID const tid = app_start_periodic_task((FP) imu_task, S4_PRI_IMU,
                                           APP_CORE_ANY, "imu_task",
                                           ROBOT_IMU_PERIOD_MS);

    return ((tid < E_OK) ? tid : E_OK);
}

/*** end of file ***/
