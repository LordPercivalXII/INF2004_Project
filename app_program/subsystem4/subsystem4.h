/** @file subsystem4.h
 *
 * @brief Subsystem 4 -- Buddy 4: MPU6050 IMU (FR10 shock, FR11 hump
 *        height, motion classification, tilt).  Student ID: <student ID>.
 *        Port of src/buddy4_imu/.
 *
 * Posts SYSTEM_EVENT_IMU to the controller every 10 ms (IF-01, NFR7).
 * Owns: I2C0 through the kernel "iica" driver (GP4 SDA, GP5 SCL).
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "../common/robot_types.h"
#include "../common/robot_config.h"

#define S4_PRI_IMU          (16)    /* band 16-17 (TEAM.md) */

/* Private to subsystem 4 (public only for subsystem4_test.c). */
typedef struct
{
    float   samples[IMU_FILTER_WINDOW_SIZE];
    uint8_t index;
    uint8_t count;
} imu_filter_t;

void           imu_task(INT stacd, void * p_exinf);
void           imu_filter_init(imu_filter_t * p_filter);
float          imu_filter_update(imu_filter_t * p_filter, float value);
motion_class_t imu_classify_motion(float ax_g, float ay_g, float az_g,
                                   float gz_dps);
void           imu_tilt_angles(float ax_g, float ay_g, float az_g,
                               float * p_roll_deg, float * p_pitch_deg);

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
