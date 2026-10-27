/** @file subsystem4_test.c
 *
 * @brief On-target tests for subsystem 4 (IMU).  make TEST=1.
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

void
subsystem4_test (void)
{
    imu_filter_t filter;
    float        average = 0.0f;
    float        roll;
    float        pitch;
    uint32_t     idx;

    TEST_INIT();

    imu_filter_init(&filter);

    for (idx = 0U; idx < 10U; idx++)
    {
        average = imu_filter_update(&filter, (float) idx);
    }

    TEST_CHECK("S4-U01", ((average > 6.99f) && (average < 7.01f)),
               "5-tap moving average of 5..9 is 7");

    TEST_CHECK("S4-U02",
               ((MOTION_STATIONARY == imu_classify_motion(0.0f, 0.0f, 1.0f,
                                                          0.0f)) &&
                (MOTION_IMPACT == imu_classify_motion(3.5f, 0.0f, 1.0f,
                                                      0.0f)) &&
                (MOTION_TURNING == imu_classify_motion(0.0f, 0.0f, 1.0f,
                                                       30.0f)) &&
                (MOTION_CLIMBING == imu_classify_motion(0.0f, 0.0f, 1.4f,
                                                        0.0f))),
               "motion classification thresholds");

    imu_tilt_angles(0.0f, 0.70711f, 0.70711f, &roll, &pitch);
    TEST_CHECK("S4-U03", ((roll > 44.9f) && (roll < 45.1f) &&
                          (pitch > -0.1f) && (pitch < 0.1f)),
               "tilt: 45 deg roll");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */

/*** end of file ***/
