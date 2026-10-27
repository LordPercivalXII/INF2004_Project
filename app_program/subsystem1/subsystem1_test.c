/** @file subsystem1_test.c
 *
 * @brief On-target tests for subsystem 1 (telemetry).  make TEST=1.
 *        Record results in testing/subsystem1/TEST_LOG.md.
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

static bool
s1_text_equal (char const * p_a, char const * p_b)
{
    uint32_t idx = 0U;

    while ((p_a[idx] == p_b[idx]) && ('\0' != p_a[idx]))
    {
        idx++;
    }

    return (p_a[idx] == p_b[idx]);
}

void
subsystem1_test (void)
{
    char    text[S1_FIXED_TEXT_MAX];
    T_RMBF  rmbf;

    TEST_INIT();

    telemetry_format_fixed(text, 0.35f, 3U);
    TEST_CHECK("S1-U01", s1_text_equal(text, "0.350"),
               "fixed-point 0.35 -> \"0.350\"");

    telemetry_format_fixed(text, -1.25f, 3U);
    TEST_CHECK("S1-U02", s1_text_equal(text, "-1.250"),
               "fixed-point -1.25 -> \"-1.250\"");

    telemetry_format_fixed(text, 24.04f, 1U);
    TEST_CHECK("S1-U03", s1_text_equal(text, "24.0"),
               "fixed-point 1 decimal -> \"24.0\"");

    TEST_CHECK("S1-U04", (E_OK == tk_ref_mbf(gh_telemetry_mbf, &rmbf)),
               "telemetry message buffer exists");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

/*** end of file ***/
