/** @file subsystem1_test.c
 *
 * @brief On-target tests for subsystem 1 (telemetry).
 *        make TEST=1.
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

    /* Existing fixed-point formatting tests */

    telemetry_format_fixed(text, 0.35f, 3U);
    TEST_CHECK("S1-U01", s1_text_equal(text, "0.350"),
               "fixed-point 0.35 -> \"0.350\"");

    telemetry_format_fixed(text, -1.25f, 3U);
    TEST_CHECK("S1-U02", s1_text_equal(text, "-1.250"),
               "fixed-point -1.25 -> \"-1.250\"");

    telemetry_format_fixed(text, 24.04f, 1U);
    TEST_CHECK("S1-U03", s1_text_equal(text, "24.0"),
               "fixed-point 1 decimal -> \"24.0\"");

    /* Controller-to-telemetry message buffer */

    TEST_CHECK("S1-U04",
               (E_OK == tk_ref_mbf(gh_telemetry_mbf, &rmbf)),
               "telemetry message buffer exists");

    /* MQTT telemetry message buffer */

    TEST_CHECK("S1-U06",
               (E_OK == tk_ref_mbf(gh_mqtt_telemetry_mbf, &rmbf)),
               "MQTT telemetry message buffer exists");

    /* MQTT payload configuration */

#if USE_BIT_PACKING

    TEST_CHECK("S1-U07",
               (S1_MQTT_MESSAGE_SIZE == 4U),
               "packed MQTT message size is 4 bytes");

    TEST_CHECK("S1-U08",
               (S1_MQTT_QUEUE_LENGTH == ROBOT_QUEUE_LENGTH * 4U),
               "MQTT queue supports four frames per snapshot");

#else

    TEST_CHECK("S1-U07",
               (S1_MQTT_MESSAGE_SIZE == sizeof(telemetry_message_t)),
               "MQTT snapshot message size is correct");

    TEST_CHECK("S1-U08",
               (S1_MQTT_QUEUE_LENGTH == ROBOT_QUEUE_LENGTH),
               "MQTT snapshot queue length is correct");

#endif

    /* Independent message buffers */

    TEST_CHECK("S1-U09",
               (gh_telemetry_mbf > E_OK) &&
               (gh_mqtt_telemetry_mbf > E_OK) &&
               (gh_telemetry_mbf != gh_mqtt_telemetry_mbf),
               "telemetry and MQTT buffers are separate");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

/*** end of file ***/
