/** @file subsystem5_test.c
 *
 * @brief On-target tests for subsystem 5 (obstacle).  make TEST=1.
 */

#include "subsystem5.h"
#include "../common/robot_config.h"
#include "../test_report.h"

#if APP_TEST

void
subsystem5_test (void)
{
    obstacle_scan_t scan;
    float           width;

    TEST_INIT();

    TEST_CHECK("S5-U01",
               ((0 == obstacle_choose_bypass(10.0f, 10.0f)) &&
                (1 == obstacle_choose_bypass(10.0f, 80.0f)) &&
                (-1 == obstacle_choose_bypass(80.0f, 10.0f)) &&
                (-1 == obstacle_choose_bypass(90.0f, 60.0f))),
               "bypass side: blocked / right / left / clearer");

    obstacle_scan_reset(&scan);
    obstacle_scan_include(&scan, ULTRASONIC_COARSE_CENTER_DEG, 24.0f);
    TEST_CHECK("S5-U02", (scan.b_detected && scan.b_has_hit),
               "hit at 24 cm straight ahead is detected");

    width = obstacle_estimate_width(&scan);
    TEST_CHECK("S5-U03", ((width > 6.2f) && (width < 6.4f)),
               "single hit width = beam footprint (~6.3 cm at 24 cm)");

    obstacle_scan_include(&scan, 60U, 200.0f);
    TEST_CHECK("S5-U04", (scan.left_clearance > 199.0f),
               "far reading on the left sets left clearance");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */

/*** end of file ***/
