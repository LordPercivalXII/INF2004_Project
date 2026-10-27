/** @file test_report.h
 *
 * @brief Minimal on-target test helpers (built only with make TEST=1).
 *
 * Every check prints ONE line in a fixed format, so a saved console log
 * can be searched for results ("grep '\[TEST\]' log.txt"):
 *
 *     [TEST] S3-U01 PASS  Code 39 round trip "L"
 *     [TEST] S3-U02 FAIL  reverse decode   (code39.c:57)
 *     [TEST] subsystem3 SUMMARY 1 passed, 1 failed
 *
 * Usage, inside a test function:
 *     TEST_INIT();
 *     TEST_CHECK("S3-U01", (value >= 0), "description");
 *     TEST_SUMMARY("subsystem3");
 *
 * These are macros, not functions, only because they need the caller's
 * __FILE__ and __LINE__ (BARR-C 6.3 exception).
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <tm/tmonitor.h>

#define TEST_INIT()  int32_t test_pass = 0; int32_t test_fail = 0

#define TEST_CHECK(id, cond, desc)                                       \
    do                                                                   \
    {                                                                    \
        if (cond)                                                        \
        {                                                                \
            test_pass++;                                                 \
            tm_printf((UB *) "[TEST] %s PASS  %s\n", (id), (desc));      \
        }                                                                \
        else                                                             \
        {                                                                \
            test_fail++;                                                 \
            tm_printf((UB *) "[TEST] %s FAIL  %s   (%s:%d)\n",           \
                      (id), (desc), __FILE__, __LINE__);                 \
        }                                                                \
    } while (0)

#define TEST_SUMMARY(name)                                               \
    tm_printf((UB *) "[TEST] %s SUMMARY %d passed, %d failed\n",         \
              (name), (int) test_pass, (int) test_fail)

#endif /* TEST_REPORT_H */

/*** end of file ***/
