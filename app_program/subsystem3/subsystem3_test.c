/** @file subsystem3_test.c
 *
 * @brief On-target tests for subsystem 3 (line, barcode, routing).
 *        Built only with make TEST=1.  Record results in
 *        testing/subsystem3/TEST_LOG.md.
 *
 * The route tests use a local table, so they do not disturb the table
 * line_task received over MQTT.
 */

#include "subsystem3.h"
#include "code39.h"
#include "../common/drv_board.h"
#include "../test_report.h"

#if APP_TEST

#define S3_TEST_NARROW      (1000U)
#define S3_TEST_WIDE        (2500U)
#define S3_TEST_ROUTE_WAIT  (500)
#define S3_TEST_CLOCK_MS    (200)

#define S3_GO   ((uint8_t) BARCODE_CMD_NONE)     /* straight */
#define S3_L    ((uint8_t) BARCODE_CMD_LEFT)
#define S3_R    ((uint8_t) BARCODE_CMD_RIGHT)
#define S3_U    ((uint8_t) BARCODE_CMD_UTURN)

/*!
 * @brief Encode text, optionally reverse it, decode it; check text and
 *        reported direction.
 */
static bool
s3_round_trip (char const * p_text, bool b_backward)
{
    static uint32_t widths[CODE39_MAX_ELEMENTS];
    static uint32_t crossed[CODE39_MAX_ELEMENTS];
    char            text[BARCODE_TEXT_MAX + 1U];
    bool            b_reversed = !b_backward;
    bool            b_ok;
    uint32_t        count;
    uint32_t        idx;

    count = code39_encode_widths(p_text, S3_TEST_NARROW, S3_TEST_WIDE,
                                 widths, CODE39_MAX_ELEMENTS);

    for (idx = 0U; idx < count; idx++)
    {
        crossed[idx] = widths[b_backward ? (count - 1U - idx) : idx];
    }

    b_ok = (0U != count) &&
           code39_decode_dir(crossed, count, text, sizeof(text),
                             &b_reversed) &&
           (b_reversed == b_backward);

    for (idx = 0U; b_ok && ('\0' != p_text[idx]); idx++)
    {
        b_ok = (p_text[idx] == text[idx]);
    }

    return (b_ok && ('\0' == text[idx]));
}

static void
s3_test_routes (int32_t * p_pass, int32_t * p_fail)
{
    route_table_t        table;
    route_parse_result_t result;
    char const           plain[] = "A:L,B:R,C:S,D:U";
    char const           json[]  = "{\"7\":\"LEFT\",\n \"x\":\"u-turn\"}";
    char const           bad[]   = "A:JUMP,!:L,LONGTOKENTOOLONGXX:L,Q";
    char const           def[]   = "CLEAR;*=R;B=STRAIGHT";
    int32_t              test_pass = *p_pass;
    int32_t              test_fail = *p_fail;

    route_table_reset(&table);
    TEST_CHECK("S3-U06",
               (S3_GO == route_table_lookup(&table, 'A')),
               "empty table: every character goes straight");

    route_table_parse(&table, plain, sizeof(plain) - 1U, &result);
    TEST_CHECK("S3-U07",
               ((4U == result.applied) && (0U == result.rejected) &&
                (S3_L == route_table_lookup(&table, 'A')) &&
                (S3_R == route_table_lookup(&table, 'B')) &&
                (S3_GO == route_table_lookup(&table, 'C')) &&
                (S3_U == route_table_lookup(&table, 'd'))),
               "MQTT \"A:L,B:R,C:S,D:U\" -> left/right/straight/U-turn");

    route_table_parse(&table, json, sizeof(json) - 1U, &result);
    TEST_CHECK("S3-U08",
               ((2U == result.applied) &&
                (S3_L == route_table_lookup(&table, '7')) &&
                (S3_U == route_table_lookup(&table, 'X')) &&
                (S3_L == route_table_lookup(&table, 'A'))),
               "JSON form accepted; updates are cumulative");

    route_table_parse(&table, bad, sizeof(bad) - 1U, &result);
    TEST_CHECK("S3-U09",
               ((0U == result.applied) && (4U == result.rejected) &&
                (S3_L == route_table_lookup(&table, 'A'))),
               "bad entries rejected without changing the table");

    route_table_parse(&table, def, sizeof(def) - 1U, &result);
    TEST_CHECK("S3-U10",
               (result.b_cleared &&
                (S3_R == route_table_lookup(&table, 'A')) &&
                (S3_GO == route_table_lookup(&table, 'B'))),
               "CLEAR and default action \"*\" for unmapped characters");

    *p_pass = test_pass;
    *p_fail = test_fail;
}

void
subsystem3_test (void)
{
    static uint32_t widths[CODE39_MAX_ELEMENTS];
    char            text[BARCODE_TEXT_MAX + 1U];
    route_status_t  status;
    uint32_t        count;

    TEST_INIT();

    TEST_CHECK("S3-U01", s3_round_trip("L", false),
               "Code 39 \"*L*\" read left-to-right");

    TEST_CHECK("S3-U02", (s3_round_trip("L", true) &&
                          s3_round_trip("A1", true) &&
                          s3_round_trip("Z", true)),
               "Code 39 read right-to-left, direction reported");

    count = code39_encode_widths("A1", S3_TEST_NARROW, S3_TEST_WIDE,
                                 widths, CODE39_MAX_ELEMENTS);
    widths[4] = S3_TEST_NARROW + 200U;  /* blur one wide/narrow pair */
    widths[2] = S3_TEST_NARROW + 400U;
    TEST_CHECK("S3-U03", !code39_decode(widths, count, text, sizeof(text))
               || ('A' == text[0]),
               "corrupted symbol is rejected or still correct");

    TEST_CHECK("S3-U04", (s3_round_trip("-", true) &&
                          s3_round_trip("%", false)),
               "punctuation characters decode both ways");

    /* Real sensor: bars ~40% of a narrow element too short, spaces too
       long (threshold off mid-grey), plus a stray edge pair at the end
       of the swipe.  Must still decode, never as a wrong character. */
    count = code39_encode_widths("A", S3_TEST_NARROW * 3U, S3_TEST_WIDE * 3U,
                                 widths, CODE39_MAX_ELEMENTS);
    {
        uint32_t idx;
        bool     b_rev = true;

        for (idx = 0U; idx < count; idx++)
        {
            widths[idx] = (0U == (idx % 2U)) ? (widths[idx] - 1200U)
                                             : (widths[idx] + 1200U);
        }

        widths[count]      = 5000U;     /* stray space + bar */
        widths[count + 1U] = 2000U;
        TEST_CHECK("S3-U14",
                   (code39_decode_dir(widths, count + 2U, text,
                                      sizeof(text), &b_rev) &&
                    ('A' == text[0]) && ('\0' == text[1]) && !b_rev),
                   "edge-biased scan with stray edges decodes \"A\"");
    }

    TEST_CHECK("S3-U05",
               ((line_compute_error(1.0f, 0.0f, true, 0.0f) < -0.99f) &&
                (line_compute_error(0.0f, 1.0f, true, 0.0f) > 0.99f) &&
                (line_compute_error(0.0f, 0.0f, false, 0.3f) > 0.99f)),
               "line error sign and lost-line saturation");

    s3_test_routes(&test_pass, &test_fail);

    /* Live table: in SIM=1 the simulated MQTT message must have arrived;
       on hardware this only checks the status read works. */
    (void) tk_dly_tsk(S3_TEST_ROUTE_WAIT);
    route_get_status(&status);
    TEST_CHECK("S3-U11", ((0 == ROBOT_SIMULATION) ||
                          ((1U <= status.config_count) &&
                           (4U == status.mapped_count))),
               "route message reached line_task through the MQTT hook");

    /* Barcode widths, the 1 kHz IR sampler and the ultrasonic echo all
       depend on the TIMER really counting microseconds. */
    {
        uint32_t const start_us = board_micros();
        uint32_t       elapsed_us;

        (void) tk_dly_tsk(S3_TEST_CLOCK_MS);
        elapsed_us = board_micros() - start_us;
        TEST_CHECK("S3-U13", ((elapsed_us >= 190000U) &&
                              (elapsed_us <= 215000U)),
                   "microsecond TIMER runs at 1 MHz (200 ms delay)");
    }

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */

/*** end of file ***/
