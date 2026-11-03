/** @file subsystem4.h
 *
 * @brief Buddy 4: LSM303DLHC acceleration/magnetic terrain monitoring.
 *        No gyroscope: turn and height are explicitly approximate.
 *        Height uses imu_height.h with Buddy 2 wheel travel, fused in the
 *        controller. Live rise/peak require level-ground initialization,
 *        straight rolling, valid tilt, no wheel slip and calibrated odometry.
 *
 * Posts SYSTEM_EVENT_IMU to the controller every 10 ms (IF-01, NFR7).
 * Owns: I2C0 through the kernel "iica" driver (GP4 SDA, GP5 SCL).
 * Hold the car still and level for the 200-sample startup calibration.
 * Processing assumptions and host-testable API: imu_processing.h.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "../common/robot_types.h"
#include "../common/robot_config.h"
#include "imu_processing.h"

#define S4_PRI_IMU          (16)    /* band 16-17 (TEAM.md) */

void           imu_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
