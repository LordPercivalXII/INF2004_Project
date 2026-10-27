/** @file line.c
 *
 * @brief Line tracking task (FR7, NFR6), every 10 ms.
 *
 * Each cycle averages the ~10 IR samples the 1 kHz interrupt took since
 * the last cycle (~20 at the original 20 ms cadence), normalises them
 * with the boot calibration, computes a continuous line error, detects
 * junctions and posts SYSTEM_EVENT_LINE with any decoded barcode.
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

typedef struct
{
    uint16_t white;     /* ADC counts on floor */
    uint16_t black;     /* ADC counts on line  */
    bool     b_calibrated;
} ir_calibration_t;

typedef struct
{
    ir_calibration_t cal_left;
    ir_calibration_t cal_center;
    float            last_error;
    uint32_t         junction_since_ms;
    bool             b_junction_reported;
    uint16_t         held_left;
    uint16_t         held_center;
    bool             b_held_right_black;
    uint32_t         next_sim_barcode_ms;
    bool             b_sim_left_next;
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
 * @brief Line error: left only = -1, centre only = +1, both = 0,
 *        continuous in between.  When the line is lost, saturate toward
 *        the side it was last seen so the robot turns back.
 */
float
line_compute_error (float left, float center, bool b_detected,
                    float last_error)
{
    float error = 0.0f;

    if (b_detected)
    {
        error = center - left;
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

#if !ROBOT_SIMULATION
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

    line_calibration_apply(&p_ctx->cal_left, left_min, left_max);
    line_calibration_apply(&p_ctx->cal_center, center_min, center_max);
}
#endif /* !ROBOT_SIMULATION */

#if ROBOT_SIMULATION
/*!
 * @brief Simulation: line drifts between the sensors; every 10 s a
 *        "*L*" or "*R*" barcode is injected as if crossed at 0.1 m/s.
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
    uint32_t          widths[CODE39_MAX_ELEMENTS];
    barcode_element_t element;
    uint32_t            count;
    uint32_t            idx;

    *p_left   = (uint16_t) ((float) IR_DEFAULT_WHITE_COUNTS +
                            ((1.0f - drift) * span));
    *p_center = (uint16_t) ((float) IR_DEFAULT_WHITE_COUNTS +
                            (drift * span));

    if ((int32_t) (now_ms - p_ctx->next_sim_barcode_ms) >= 0)
    {
        count = code39_encode_widths(p_ctx->b_sim_left_next ? "L" : "R",
                                     LINE_SIM_NARROW_US, LINE_SIM_WIDE_US,
                                     widths, CODE39_MAX_ELEMENTS);

        for (idx = 0U; idx < count; idx++)
        {
            element.width_us = widths[idx];
            element.end_us   = board_micros();
            element.is_black = (0U == (idx % 2U)) ? 1U : 0U;
            (void) tk_snd_mbf(gh_barcode_edge_mbf, &element,
                              (INT) sizeof(element), TMO_POL);
        }

        p_ctx->b_sim_left_next     = !p_ctx->b_sim_left_next;
        p_ctx->next_sim_barcode_ms = now_ms + LINE_SIM_CODE_GAP_MS;
    }
}
#endif /* ROBOT_SIMULATION */

/*!
 * @brief Get this cycle's raw readings (simulated or averaged).
 */
static void
line_read (line_ctx_t * p_ctx, uint32_t now_ms, uint16_t * p_left,
           uint16_t * p_center, bool * p_right_black)
{
#if ROBOT_SIMULATION
    line_simulate(p_ctx, now_ms, p_left, p_center);
    *p_right_black = false;
#else
    ir_window_t window;

    (void) now_ms;
    ir_sampler_drain(&window);

    if (window.count > 0U)
    {
        p_ctx->held_left   = (uint16_t) (window.left_sum / window.count);
        p_ctx->held_center = (uint16_t) (window.center_sum / window.count);
        p_ctx->b_held_right_black =
            ((2U * window.right_black_count) > window.count);
    }

    *p_left        = p_ctx->held_left;
    *p_center      = p_ctx->held_center;
    *p_right_black = p_ctx->b_held_right_black;
#endif
}

/*!
 * @brief Junction: all three sensors black for JUNCTION_MIN_MS (rejects
 *        single barcode bars).  Reported once per crossing.
 */
static bool
line_junction (line_ctx_t * p_ctx, bool b_all_black, uint32_t now_ms)
{
    bool b_junction = false;

    if (b_all_black)
    {
        if (0U == p_ctx->junction_since_ms)
        {
            p_ctx->junction_since_ms = now_ms;
        }

        if ((!p_ctx->b_junction_reported) &&
            ((now_ms - p_ctx->junction_since_ms) >= JUNCTION_MIN_MS))
        {
            b_junction = true;
            p_ctx->b_junction_reported = true;
        }
    }
    else
    {
        p_ctx->junction_since_ms   = 0U;
        p_ctx->b_junction_reported = false;
    }

    return (b_junction);
}

static void
line_publish (float error, bool b_detected, bool b_junction,
              uint32_t now_ms)
{
    system_event_t   event  = {0};
    barcode_result_t result = {0};
    uint32_t           idx;

    event.type                       = SYSTEM_EVENT_LINE;
    event.timestamp_ms               = now_ms;
    event.data.line.line_error       = error;
    event.data.line.b_line_detected  = b_detected;
    event.data.line.b_junction       = b_junction;

    if (barcode_result_take(&result))
    {
        event.data.line.b_barcode_valid = true;
        event.data.line.barcode         = result.command;

        for (idx = 0U; idx < sizeof(result.text); idx++)
        {
            event.data.line.barcode_text[idx] = result.text[idx];
        }
    }

    if ((!controller_post_event(&event)) && event.data.line.b_barcode_valid)
    {
        barcode_result_put(&result);    /* buffer full: retry next cycle */
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
    bool              b_right_black;
    float             left;
    float             center;
    float             error;
    bool              b_left_on;
    bool              b_center_on;

    (void) stacd;
    (void) p_exinf;

    line_calibration_apply(&ctx.cal_left, 0U, 0U);      /* defaults */
    line_calibration_apply(&ctx.cal_center, 0U, 0U);
    ctx.held_left           = IR_DEFAULT_WHITE_COUNTS;
    ctx.held_center         = IR_DEFAULT_WHITE_COUNTS;
    ctx.next_sim_barcode_ms = board_millis() + LINE_SIM_FIRST_CODE_MS;
    ctx.b_sim_left_next     = true;

#if !ROBOT_SIMULATION
    app_check(ir_sampler_start(), "ir_sampler_start");
    line_calibrate(&ctx);
#endif

    /* Discard wake-ups queued during calibration. */
    (void) tk_can_wup(TSK_SELF);

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);    /* woken every 10 ms */

        now_ms = board_millis();
        line_read(&ctx, now_ms, &raw_left, &raw_center, &b_right_black);

        left        = line_normalise(raw_left, &ctx.cal_left);
        center      = line_normalise(raw_center, &ctx.cal_center);
        b_left_on   = (left >= IR_ON_LINE_THRESHOLD);
        b_center_on = (center >= IR_ON_LINE_THRESHOLD);

        error = line_compute_error(left, center, (b_left_on || b_center_on),
                                   ctx.last_error);

        if (b_left_on || b_center_on)
        {
            ctx.last_error = error;
        }

        line_publish(error, (b_left_on || b_center_on),
                     line_junction(&ctx, (b_left_on && b_center_on &&
                                          b_right_black), now_ms),
                     now_ms);
    }
}

/*** end of file ***/
