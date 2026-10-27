/** @file subsystem3_test.c
 *
 * @brief On-target tests for subsystem 3 (line + barcode).  make TEST=1.
 */

#include "subsystem3.h"
#include "code39.h"
#include "../test_report.h"

#if APP_TEST

#define S3_TEST_NARROW  (1000U)
#define S3_TEST_WIDE    (2500U)

void
subsystem3_test (void)
{
    static uint32_t widths[CODE39_MAX_ELEMENTS];
    static uint32_t reversed[CODE39_MAX_ELEMENTS];
    char            text[BARCODE_TEXT_MAX + 1U];
    uint32_t          count;
    uint32_t          idx;

    TEST_INIT();

    count = code39_encode_widths("L", S3_TEST_NARROW, S3_TEST_WIDE,
                                 widths, CODE39_MAX_ELEMENTS);
    TEST_CHECK("S3-U01", ((29U == count) &&
                          code39_decode(widths, count, text, sizeof(text)) &&
                          ('L' == text[0]) && ('\0' == text[1])),
               "Code 39 round trip \"*L*\"");

    for (idx = 0U; idx < count; idx++)
    {
        reversed[idx] = widths[count - 1U - idx];
    }

    TEST_CHECK("S3-U02", (code39_decode(reversed, count, text,
                                        sizeof(text)) && ('L' == text[0])),
               "Code 39 decodes when crossed backwards");

    count = code39_encode_widths("A1", S3_TEST_NARROW, S3_TEST_WIDE,
                                 widths, CODE39_MAX_ELEMENTS);
    widths[4] = S3_TEST_NARROW + 200U;  /* blur one wide/narrow pair */
    widths[2] = S3_TEST_NARROW + 400U;
    TEST_CHECK("S3-U03", !code39_decode(widths, count, text, sizeof(text))
               || ('A' == text[0]),
               "corrupted symbol is rejected or still correct");

    TEST_CHECK("S3-U04",
               (((uint16_t) BARCODE_CMD_LEFT == barcode_to_command("L")) &&
                ((uint16_t) BARCODE_CMD_RIGHT == barcode_to_command("R")) &&
                ((uint16_t) BARCODE_CMD_UTURN == barcode_to_command("U")) &&
                ((uint16_t) BARCODE_CMD_NONE == barcode_to_command("7"))),
               "payload maps to turn command including U-turn");

    TEST_CHECK("S3-U05",
               ((line_compute_error(1.0f, 0.0f, true, 0.0f) < -0.99f) &&
                (line_compute_error(0.0f, 1.0f, true, 0.0f) > 0.99f) &&
                (line_compute_error(0.0f, 0.0f, false, 0.3f) > 0.99f)),
               "line error sign and lost-line saturation");

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */

/*** end of file ***/
