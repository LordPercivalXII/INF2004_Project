/** @file line.c
 *
 * @brief Line tracking task (FR7, NFR6), every 10 ms.
 *
 * Each cycle averages the ~10 IR samples the 1 kHz interrupt took since
 * the last cycle, normalises them with the boot calibration, estimates
 * the line position, detects junctions and posts SYSTEM_EVENT_LINE.
 *
 * Two sensors, left (L) and middle (M), each sit over one EDGE of the
 * ~18 mm line.  An IR spot only gives an in-between reading while it
 * straddles an edge, so placing the sensors on the edges makes the
 * error continuous around the centre:
 *
 *     state     L      M      error = M - L
 *     centred   ~0.5   ~0.5   0
 *     drifted   up     down   towards -1 (line to the left)
 *     drifted   down   up     towards +1 (line to the right)
 *     lost      ~0     ~0     +-1 towards the side last seen
 *     junction  ~1     ~1     both black: a crossing line
 *
 * Routing (route.c): a decoded barcode is stored as the pending waypoint
 * and reported once; the car keeps following the line.  At the next
 * junction the stored character is looked up in the MQTT route table and
 * the action travels to the controller with that junction event.  Route
 * table messages from MQTT are applied here, once per cycle.
 */

#include "subsystem3.h"
#include "code39.h"
#include "drv_ir_sampler.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../controller/controller.h"

#define LINE_ADC_MAX            (4095U)
#define LINE_LOST_DEADBAND      (0.05f)
#define LINE_SIM_STEP_MS        (20U)
#define LINE_SIM_PHASES         (200U)
#define LINE_SIM_HALF           (100U)
#define LINE_SIM_FIRST_CODE_MS  (5000U)
#define LINE_SIM_CODE_GAP_MS    (10000U)
#define LINE_SIM_NARROW_US      (20000U)    /* 2 mm at ~0.1 m/s */
#define LINE_SIM_WIDE_US        (50000U)    /* 5 mm at ~0.1 m/s */
#define LINE_SIM_JUNCTION_AT_MS (3000U)     /* barcode -> junction   */
#define LINE_SIM_JUNCTION_MS    (300U)      /* all-black duration    */
#define LINE_SIM_CODE_COUNT     (5U)

typedef struct
{
    uint16_t white;     /* ADC counts on floor */
    uint16_t black;     /* ADC counts on line  */
    bool     b_calibrated;
} ir_calibration_t;

/* Calibration learned while driving (S3_LINE_HW).  The boot sweep needs
 * the car swept across the line; when it was not, the fixed defaults made
 * a junction (both sensors >= LINE_JUNCTION_LEVEL) register only when the
 * sensors sat squarely on the tape, i.e. late.  While a mission runs, the
 * extremes each sensor holds for LINE_LEARN_HOLD cycles are tracked and,
 * once they span IR_MIN_CONTRAST_COUNTS, used as white and black. */
#define LINE_LEARN_HOLD         (3U)        /* cycles: rejects spikes      */
/* Black is only learned while the OTHER sensor reads white (below this):
   one sensor over the tape is normal line following, whereas both dark
   at once is a junction, the ramp lifting the front (no floor = very
   "black"), or the barcode - learning from those set black far darker
   than the tape, so junctions stopped registering. */
#define LINE_LEARN_OTHER_WHITE  (0.30f)

/* Junction near-miss report: both sensors at least this dark without a
   junction being confirmed prints the values (diagnostics). */
#define LINE_NEAR_MISS_LEVEL    (0.40f)
#define LINE_LEARN_REPORT_PCT   (10U)       /* print when span grows this  */

typedef struct
{
    uint16_t lo;                            /* steady minimum, ADC counts */
    uint16_t hi;                            /* steady maximum             */
    uint16_t recent[LINE_LEARN_HOLD];
    uint32_t count;
    uint16_t reported_span;
} ir_learn_t;

typedef struct
{
    ir_calibration_t cal_left;
    ir_calibration_t cal_center;
    ir_learn_t       learn_left;
    ir_learn_t       learn_center;
    bool             b_near_active;     /* both sensors darkish now     */
    bool             b_near_junction;   /* ...and a junction confirmed  */
    float            near_peak;         /* peak of min(left, center)    */
    float            near_left;
    float            near_center;
    uint16_t         near_raw_left;
    uint16_t         near_raw_center;
    float            last_error;
    line_junction_t  junction;
    uint16_t         held_left;
    uint16_t         held_center;
    uint32_t         last_debug_ms;
    uint32_t         next_sim_barcode_ms;
    uint32_t         sim_code_index;
    uint32_t         sim_junction_ms;
    bool             b_sim_junction_armed;
    bool             b_junction_retry;      /* last junction not sent  */
    uint16_t         retry_action;
    char             retry_char;
} line_ctx_t;

/* ------------------------------------------------------------------ */
/* Pure helpers                                                        */
/* ------------------------------------------------------------------ */

/*!
 * @brief 0.0 = white floor, 1.0 = black line.
 */
static float
line_normalise (uint16_t raw, ir_calibration_t const * p_cal)
{
    float const span  = (float) p_cal->black - (float) p_cal->white;
    float       value = ((float) raw - (float) p_cal->white) / span;

    if (value < 0.0f)
    {
        value = 0.0f;
    }
    else if (value > 1.0f)
    {
        value = 1.0f;
    }
    else
    {
        /* In range. */
    }

    return (value);
}

/*!
 * @brief Line position from the two edge sensors.
 *
 * @param[in]  left          Left sensor, 0 = white .. 1 = black.
 * @param[in]  center        Middle sensor, same scale.
 * @param[in]  last_error    Error of the last cycle the line was seen.
 * @param[out] p_b_detected  false when both sensors read white.
 *
 * @return -1 (line to the left) .. +1 (line to the right).  When the line
 *         is lost, full scale towards the side it was last seen, so the
 *         car turns back to it.
 */
float
line_compute_error (float left, float center, float last_error,
                    bool * p_b_detected)
{
    float error = 0.0f;

    *p_b_detected = (left >= LINE_LOST_LEVEL) || (center >= LINE_LOST_LEVEL);

    if (*p_b_detected)
    {
        error = LINE_ERROR_SIGN * (center - left);
    }
    else if (last_error > LINE_LOST_DEADBAND)
    {
        error = 1.0f;
    }
    else if (last_error < -LINE_LOST_DEADBAND)
    {
        error = -1.0f;
    }
    else
    {
        error = 0.0f;
    }

    return (error);
}

/*!
 * @brief Report a crossing where both sensors got dark but no junction
 *        was confirmed (values and calibration), so a missed junction is
 *        never silent.  Printed once, when the dark patch ends.
 */
static void
line_near_miss (line_ctx_t * p_ctx, float left, float center,
                uint16_t raw_left, uint16_t raw_center, bool b_junction)
{
    float const both = (left < center) ? left : center;

    if (both >= LINE_NEAR_MISS_LEVEL)
    {
        if ((!p_ctx->b_near_active) || (both > p_ctx->near_peak))
        {
            p_ctx->near_peak       = both;
            p_ctx->near_left       = left;
            p_ctx->near_center     = center;
            p_ctx->near_raw_left   = raw_left;
            p_ctx->near_raw_center = raw_center;
        }
        p_ctx->b_near_active   = true;
        p_ctx->b_near_junction = p_ctx->b_near_junction || b_junction;
    }
    else if (p_ctx->b_near_active)
    {
        if (!p_ctx->b_near_junction)
        {
            tm_printf((UB *) "[S3 junction] near miss: both sensors dark "
                      "but below %u%%: left %u%% (raw %u, black %u), "
                      "middle %u%% (raw %u, black %u)\n",
                      (unsigned int) (LINE_JUNCTION_LEVEL * 100.0f),
                      (unsigned int) (p_ctx->near_left * 100.0f),
                      (unsigned int) p_ctx->near_raw_left,
                      (unsigned int) p_ctx->cal_left.black,
                      (unsigned int) (p_ctx->near_center * 100.0f),
                      (unsigned int) p_ctx->near_raw_center,
                      (unsigned int) p_ctx->cal_center.black);
        }
        p_ctx->b_near_active   = false;
        p_ctx->b_near_junction = false;
        p_ctx->near_peak       = 0.0f;
    }
    else
    {
        /* Not near a crossing. */
    }
}

/*!
 * @brief Junction: both sensors black for LINE_JUNCTION_MIN_CYCLES
 *        consecutive cycles.  On a straight line that cannot happen (the
 *        sensors sit on the two edges); only a crossing line covers both.
 *        Reported once per crossing; re-armed once either sensor is back
 *        below LINE_JUNCTION_RELEASE.
 *
 * @return true on the cycle the junction is first confirmed.
 */
bool
line_junction_update (line_junction_t * p_junction, float left, float center)
{
    bool b_new = false;

    if ((left >= LINE_JUNCTION_LEVEL) && (center >= LINE_JUNCTION_LEVEL))
    {
        if (p_junction->black_cycles < LINE_JUNCTION_MIN_CYCLES)
        {
            p_junction->black_cycles++;
        }

        if ((!p_junction->b_reported) &&
            (p_junction->black_cycles >= LINE_JUNCTION_MIN_CYCLES))
        {
            p_junction->b_reported = true;
            b_new = true;
        }
    }
    else
    {
        p_junction->black_cycles = 0U;

        if ((left < LINE_JUNCTION_RELEASE) || (center < LINE_JUNCTION_RELEASE))
        {
            p_junction->b_reported = false;
        }
    }

    return (b_new);
}

static void
line_calibration_apply (ir_calibration_t * p_cal, uint16_t min_counts,
                        uint16_t max_counts)
{
    if ((uint32_t) (max_counts - min_counts) >= IR_MIN_CONTRAST_COUNTS)
    {
        p_cal->white = IR_ANALOG_BLACK_IS_HIGH ? min_counts : max_counts;
        p_cal->black = IR_ANALOG_BLACK_IS_HIGH ? max_counts : min_counts;
        p_cal->b_calibrated = true;
    }
    else
    {
        p_cal->white = IR_DEFAULT_WHITE_COUNTS;
        p_cal->black = IR_DEFAULT_BLACK_COUNTS;
        p_cal->b_calibrated = false;
    }
}

/* ------------------------------------------------------------------ */
/* Hardware: calibration and per-cycle averaging                       */
/* ------------------------------------------------------------------ */

#if S3_LINE_HW
/*!
 * @brief Sweep the sensors across line and floor for 3 s.  Min/max are
 *        taken over ~10-sample averages so one noisy sample cannot set a
 *        threshold.
 */
static void
line_calibrate (line_ctx_t * p_ctx)
{
    uint16_t       left_min   = LINE_ADC_MAX;
    uint16_t       left_max   = 0U;
    uint16_t       center_min = LINE_ADC_MAX;
    uint16_t       center_max = 0U;
    uint32_t const start_ms   = board_millis();
    ir_window_t    window;

    while ((board_millis() - start_ms) < IR_CALIBRATION_MS)
    {
        (void) tk_dly_tsk((TMO) IR_CALIBRATION_STEP_MS);
        ir_sampler_drain(&window);

        if (window.count > 0U)
        {
            uint16_t const left   = (uint16_t) (window.left_sum /
                                                window.count);
            uint16_t const center = (uint16_t) (window.center_sum /
                                                window.count);

            left_min   = (left < left_min) ? left : left_min;
            left_max   = (left > left_max) ? left : left_max;
            center_min = (center < center_min) ? center : center_min;
            center_max = (center > center_max) ? center : center_max;
        }
    }

    tm_printf((UB *) "S3: line calibration: sweep done\n");
    line_calibration_apply(&p_ctx->cal_left, left_min, left_max);
    line_calibration_apply(&p_ctx->cal_center, center_min, center_max);
}

/*!
 * @brief Start learning from the boot result (if it was valid) so the
 *        learned range can only widen it.
 */
static void
line_learn_init (ir_learn_t * p_learn, ir_calibration_t const * p_cal)
{
    *p_learn = (ir_learn_t) {0};
    p_learn->lo = LINE_ADC_MAX;
    p_learn->hi = 0U;

    if (p_cal->b_calibrated)
    {
        p_learn->lo = (p_cal->white < p_cal->black) ? p_cal->white
                                                    : p_cal->black;
        p_learn->hi = (p_cal->white < p_cal->black) ? p_cal->black
                                                    : p_cal->white;
        p_learn->reported_span = (uint16_t) (p_learn->hi - p_learn->lo);
    }
}

static bool
line_mission_running (void)
{
    T_RFLG status = {0};

    return ((gh_system_flg > 0) &&
            (E_OK == tk_ref_flg(gh_system_flg, &status)) &&
            (0U != (status.flgptn & SYSTEM_BIT_MISSION_RUN)));
}

/*!
 * @brief Learn one sensor's white/black from what it reads while driving.
 *
 * @param[in] raw  This cycle's averaged ADC reading.
 */
static void
line_learn (ir_learn_t * p_learn, ir_calibration_t * p_cal, uint16_t raw,
            float other, char const * p_name)
{
    uint16_t steady_hi = LINE_ADC_MAX;
    uint16_t steady_lo = 0U;
    uint16_t span;
    uint32_t idx;

    p_learn->recent[p_learn->count % LINE_LEARN_HOLD] = raw;
    p_learn->count++;

    if (p_learn->count < LINE_LEARN_HOLD)
    {
        return;
    }

    /* A level counts only if it held for LINE_LEARN_HOLD cycles. */
    for (idx = 0U; idx < LINE_LEARN_HOLD; idx++)
    {
        steady_hi = (p_learn->recent[idx] < steady_hi) ? p_learn->recent[idx]
                                                       : steady_hi;
        steady_lo = (p_learn->recent[idx] > steady_lo) ? p_learn->recent[idx]
                                                       : steady_lo;
    }

    if (other < LINE_LEARN_OTHER_WHITE)
    {
        p_learn->hi = (steady_hi > p_learn->hi) ? steady_hi : p_learn->hi;
    }
    p_learn->lo = (steady_lo < p_learn->lo) ? steady_lo : p_learn->lo;

    if (p_learn->hi <= p_learn->lo)
    {
        return;
    }

    span = (uint16_t) (p_learn->hi - p_learn->lo);

    if ((span >= IR_MIN_CONTRAST_COUNTS) &&
        ((!p_cal->b_calibrated) ||
         (((uint32_t) span * 100U) >=
          ((uint32_t) p_learn->reported_span * (100U + LINE_LEARN_REPORT_PCT)))))
    {
        p_cal->white = IR_ANALOG_BLACK_IS_HIGH ? p_learn->lo : p_learn->hi;
        p_cal->black = IR_ANALOG_BLACK_IS_HIGH ? p_learn->hi : p_learn->lo;
        p_cal->b_calibrated = true;
        p_learn->reported_span = span;
        tm_printf((UB *) "S3: line sensor %s learned while driving: "
                  "white %u, black %u counts\n", p_name,
                  (unsigned int) p_cal->white, (unsigned int) p_cal->black);
    }
}

/*!
 * @brief Report one sensor's calibration on the console.
 */
static void
line_calibration_report (char const * p_name, ir_calibration_t const * p_cal)
{
    tm_printf((UB *) "S3: line sensor %s: white %u, black %u counts%s\n",
              p_name, (unsigned int) p_cal->white,
              (unsigned int) p_cal->black,
              p_cal->b_calibrated ? ""
              : "  ** DEFAULTS: sweep across line and floor at boot **");
}
#endif /* S3_LINE_HW */

#if ROBOT_SIMULATION
/*
 * Test data for SIM=1 builds only.  It is pushed through the same entry
 * point as a real MQTT message (mqtt_utk_route_rx), so the simulation
 * exercises the whole route path; SIM=0 firmware contains no mapping.
 */
static char const g_sim_route[]  = "A:L,B:R,C:S,D:U";
#if (!S3_BARCODE_HW) && (!S3_LINE_HW)
static char const g_sim_codes[LINE_SIM_CODE_COUNT][2] =
{
    "A", "B", "C", "D", "E"     /* "E" is unmapped -> default action */
};
#endif

#if !S3_LINE_HW
/*!
 * @brief Simulation: the line drifts between the sensors; every 10 s a
 *        barcode is injected (alternately read backwards), and 3 s later
 *        a junction (all three sensors black for 300 ms).
 */
static void
line_simulate (line_ctx_t * p_ctx, uint32_t now_ms, uint16_t * p_left,
               uint16_t * p_center)
{
    float const    span  = (float) (IR_DEFAULT_BLACK_COUNTS -
                                    IR_DEFAULT_WHITE_COUNTS);
    uint32_t const phase = (now_ms / LINE_SIM_STEP_MS) % LINE_SIM_PHASES;
    float const    drift = (phase < LINE_SIM_HALF)
                           ? ((float) phase / (float) LINE_SIM_HALF)
                           : ((float) (LINE_SIM_PHASES - phase) /
                              (float) LINE_SIM_HALF);
#if !S3_BARCODE_HW
    bool const        b_backward = (1U == (p_ctx->sim_code_index % 2U));
    uint32_t          widths[CODE39_MAX_ELEMENTS];
    barcode_element_t element;
    uint32_t          count;
    uint32_t          idx;
#endif

    *p_left        = (uint16_t) ((float) IR_DEFAULT_WHITE_COUNTS +
                                 ((1.0f - drift) * span));
    *p_center      = (uint16_t) ((float) IR_DEFAULT_WHITE_COUNTS +
                                 (drift * span));

#if !S3_BARCODE_HW
    if ((int32_t) (now_ms - p_ctx->next_sim_barcode_ms) >= 0)
    {
        count = code39_encode_widths(
                    g_sim_codes[p_ctx->sim_code_index % LINE_SIM_CODE_COUNT],
                    LINE_SIM_NARROW_US, LINE_SIM_WIDE_US, widths,
                    CODE39_MAX_ELEMENTS);

        for (idx = 0U; idx < count; idx++)
        {
            element.width_us = widths[b_backward ? (count - 1U - idx) : idx];
            element.end_us   = board_micros();
            element.is_black = (0U == (idx % 2U)) ? 1U : 0U;
            (void) tk_snd_mbf(gh_barcode_edge_mbf, &element,
                              (INT) sizeof(element), TMO_POL);
        }

        p_ctx->sim_code_index++;
        p_ctx->sim_junction_ms      = now_ms + LINE_SIM_JUNCTION_AT_MS;
        p_ctx->b_sim_junction_armed = true;
        p_ctx->next_sim_barcode_ms  = now_ms + LINE_SIM_CODE_GAP_MS;
    }
#endif /* !S3_BARCODE_HW */

    if (p_ctx->b_sim_junction_armed &&
        ((int32_t) (now_ms - p_ctx->sim_junction_ms) >= 0))
    {
        *p_left        = IR_DEFAULT_BLACK_COUNTS;
        *p_center      = IR_DEFAULT_BLACK_COUNTS;

        if ((now_ms - p_ctx->sim_junction_ms) >= LINE_SIM_JUNCTION_MS)
        {
            p_ctx->b_sim_junction_armed = false;
        }
    }
}
#endif /* !S3_LINE_HW */
#endif /* ROBOT_SIMULATION */

/*!
 * @brief Get this cycle's raw readings (simulated or averaged).
 */
static void
line_read (line_ctx_t * p_ctx, uint32_t now_ms, uint16_t * p_left,
           uint16_t * p_center)
{
#if !S3_LINE_HW
    line_simulate(p_ctx, now_ms, p_left, p_center);
#else
    ir_window_t window;

    (void) now_ms;
    ir_sampler_drain(&window);

    if (window.count > 0U)
    {
        p_ctx->held_left   = (uint16_t) (window.left_sum / window.count);
        p_ctx->held_center = (uint16_t) (window.center_sum / window.count);
    }

    *p_left   = p_ctx->held_left;
    *p_center = p_ctx->held_center;
#endif
}

#if S3_LINE_DEBUG
/*!
 * @brief Live view for the hand check: raw counts, normalised % and the
 *        error, every LINE_DEBUG_PERIOD_MS.
 */
static void
line_debug (line_ctx_t * p_ctx, uint32_t now_ms, uint16_t raw_left,
            uint16_t raw_center, float left, float center, float error,
            bool b_detected)
{
    char const * p_state = "on line";

    if ((now_ms - p_ctx->last_debug_ms) >= LINE_DEBUG_PERIOD_MS)
    {
        p_ctx->last_debug_ms = now_ms;

        if (!b_detected)
        {
            p_state = "LOST";
        }
        else if ((left >= LINE_JUNCTION_LEVEL) &&
                 (center >= LINE_JUNCTION_LEVEL))
        {
            p_state = "JUNCTION";
        }
        else
        {
            /* On the line. */
        }

        tm_printf((UB *) "[S3 line] L=%4u(%3d%%) M=%4u(%3d%%) err=%+4d%% %s\n",
                  (unsigned int) raw_left, (int) (left * 100.0f),
                  (unsigned int) raw_center, (int) (center * 100.0f),
                  (int) (error * 100.0f), p_state);
    }
}
#endif /* S3_LINE_DEBUG */

/*!
 * @brief true while the controller is still acting on the last junction
 *        (IF-05 SYSTEM_BIT_JUNCTION_BUSY), or while no mission runs.
 */
static bool
line_junction_busy (void)
{
    T_RFLG status = {0};

    return ((gh_system_flg > 0) &&
            (E_OK == tk_ref_flg(gh_system_flg, &status)) &&
            ((0U != (status.flgptn & SYSTEM_BIT_JUNCTION_BUSY)) ||
             (0U == (status.flgptn & SYSTEM_BIT_MISSION_RUN))));
}

/*!
 * @brief Post this cycle's SYSTEM_EVENT_LINE.
 *
 * A newly decoded barcode is stored as the pending waypoint and reported
 * once.  On a junction, the action for the stored character is added.
 * If the event buffer is full, a junction is re-sent next cycle so the
 * controller never misses a turn.
 */
static void
line_publish (line_ctx_t * p_ctx, float error, bool b_detected,
              bool b_junction, uint32_t now_ms)
{
    system_event_t   event  = {0};
    barcode_result_t result = {0};
    uint32_t         idx;

    event.type                      = SYSTEM_EVENT_LINE;
    event.timestamp_ms              = now_ms;
    event.data.line.line_error      = error;
    event.data.line.b_line_detected = b_detected;
    event.data.line.b_barcode_scanning = barcode_scan_active();

    if (barcode_result_take(&result))
    {
        route_store_waypoint(&result, now_ms);

#if ROBOT_SIMULATION && S3_BARCODE_HW && !S3_LINE_HW
        /* Real barcode, simulated track: fake the junction 3 s later. */
        p_ctx->sim_junction_ms      = now_ms + LINE_SIM_JUNCTION_AT_MS;
        p_ctx->b_sim_junction_armed = true;
#endif
        event.data.line.b_barcode_valid    = true;
        event.data.line.b_barcode_reversed = result.b_reversed;
        event.data.line.barcode            = (uint16_t) (uint8_t)
                                             result.text[0];

        for (idx = 0U; idx < sizeof(result.text); idx++)
        {
            event.data.line.barcode_text[idx] = result.text[idx];
        }
    }

    if (b_junction && line_junction_busy())
    {
        /* The controller is still acting on the previous junction (its
           turn, or the same tape seen again after the pivot), or no
           mission is running (car carried by hand).  Do not consume the
           stored barcode: it belongs to the next junction driven. */
        tm_printf((UB *) "[S3 route] junction ignored: controller busy "
                  "or not running; waypoint kept\n");
    }
    else if (b_junction)
    {
        /* Stored barcode -> MQTT route table -> action; nothing stored or
           no mapping for the character -> straight. */
        p_ctx->retry_action     = route_take_junction_action(
                                      &p_ctx->retry_char);
        p_ctx->b_junction_retry = true;
    }
    else
    {
        /* No junction this cycle. */
    }

    if (p_ctx->b_junction_retry)
    {
        event.data.line.b_junction      = true;
        event.data.line.junction_action = p_ctx->retry_action;
        event.data.line.junction_char   = p_ctx->retry_char;
    }

    if (controller_post_event(&event))
    {
        p_ctx->b_junction_retry = false;
    }
}

/*!
 * @brief Line tracking task (10 ms, woken by its cyclic handler).
 */
void
line_task (INT stacd, void * p_exinf)
{
    static line_ctx_t ctx;
    uint32_t          now_ms;
    uint16_t          raw_left;
    uint16_t          raw_center;
    float             left;
    float             center;
    float             error;
    bool              b_detected;
    bool              b_junction;

    (void) stacd;
    (void) p_exinf;

    line_calibration_apply(&ctx.cal_left, 0U, 0U);      /* defaults */
    line_calibration_apply(&ctx.cal_center, 0U, 0U);
    ctx.held_left           = IR_DEFAULT_WHITE_COUNTS;
    ctx.held_center         = IR_DEFAULT_WHITE_COUNTS;
    ctx.next_sim_barcode_ms = board_millis() + LINE_SIM_FIRST_CODE_MS;

    route_init();

#if ROBOT_SIMULATION
    mqtt_utk_route_rx(g_sim_route, (uint32_t) (sizeof(g_sim_route) - 1U));
#endif

#if S3_LINE_HW
    app_check(ir_sampler_start(), "ir_sampler_start");
    tm_printf((UB *) "S3: line calibration: sweep the car sideways across"
              " the line for 3 s\n");
    line_calibrate(&ctx);
    line_calibration_report("left", &ctx.cal_left);
    line_calibration_report("middle", &ctx.cal_center);
    line_learn_init(&ctx.learn_left, &ctx.cal_left);
    line_learn_init(&ctx.learn_center, &ctx.cal_center);
#endif

    /* Discard wake-ups queued during calibration. */
    (void) tk_can_wup(TSK_SELF);

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);    /* woken every 10 ms */

        route_poll_config();            /* MQTT route table updates */

        now_ms = board_millis();
        line_read(&ctx, now_ms, &raw_left, &raw_center);

#if S3_LINE_HW
        if (line_mission_running())
        {
            float const other_center = line_normalise(raw_center,
                                                      &ctx.cal_center);
            float const other_left   = line_normalise(raw_left,
                                                      &ctx.cal_left);

            line_learn(&ctx.learn_left, &ctx.cal_left, raw_left,
                       other_center, "left");
            line_learn(&ctx.learn_center, &ctx.cal_center, raw_center,
                       other_left, "middle");
        }
#endif

        left   = line_normalise(raw_left, &ctx.cal_left);
        center = line_normalise(raw_center, &ctx.cal_center);
        error  = line_compute_error(left, center, ctx.last_error,
                                    &b_detected);

        if (b_detected)
        {
            ctx.last_error = error;
        }

#if S3_LINE_DEBUG
        line_debug(&ctx, now_ms, raw_left, raw_center, left, center, error,
                   b_detected);
#endif

        b_junction = line_junction_update(&ctx.junction, left, center);
        line_near_miss(&ctx, left, center, raw_left, raw_center,
                       b_junction);
        line_publish(&ctx, error, b_detected, b_junction, now_ms);
    }
}

/*** end of file ***/
