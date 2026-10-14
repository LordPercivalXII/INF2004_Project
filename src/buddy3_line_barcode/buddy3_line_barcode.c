/*
 * Buddy 3 (Wicus): IR line tracking + Code 39 barcode decoding.
 *
 * Two RTOS tasks, matching the Design Review task table:
 *   LineTracking  - periodic 20 ms (NFR6). Reads left/middle IR on ADC0/ADC1 (GP26/27),
 *                   calibrates thresholds at boot (FR7), computes a continuous line error,
 *                   detects junctions, and posts SYSTEM_EVENT_LINE to the controller.
 *   Barcode       - event-driven (FR8). The GP28 edge ISR timestamps every transition in
 *                   microseconds and queues it; this task debounces, collects bar/space
 *                   widths, decodes Code 39 and hands the result to LineTracking (FR9).
 *
 * The barcode result rides on the next line event so the controller always receives a
 * real line_error with it (the controller uses line_error on every SYSTEM_EVENT_LINE).
 */
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "config.h"
#include "common_types.h"
#include "gpio_irq_router.h"
#include "code39.h"
#include "interfaces/buddy3_line_barcode.h"

/* ------------------------------------------------------------------------- */
/* Shared objects                                                            */
/* ------------------------------------------------------------------------- */

typedef struct {
    uint32_t width_us;   /* duration of the element that just ended */
    uint32_t end_us;     /* timestamp of the edge that ended it */
    uint8_t is_black;    /* 1 = bar, 0 = space */
} BarcodeElement;

typedef struct {
    uint16_t command;                          /* BarcodeCommand */
    char text[BARCODE_TEXT_MAX + 1U];
} BarcodeResult;

static QueueHandle_t s_edge_queue;     /* ISR -> Barcode task */
static QueueHandle_t s_result_queue;   /* Barcode task -> LineTracking task (length 1, overwrite) */

/* ------------------------------------------------------------------------- */
/* Right IR edge ISR                                                         */
/* ------------------------------------------------------------------------- */

static volatile uint32_t s_isr_last_edge_us;
static volatile uint8_t s_isr_level_black;
static volatile bool s_isr_overflow;

static inline uint8_t barcode_pin_is_black(void)
{
    return (gpio_get(PIN_IR_BARCODE) ? 1U : 0U) == IR_BARCODE_BLACK_LEVEL ? 1U : 0U;
}

#if !ROBOT_SIMULATION
/* Kept short: timestamp, reject edges that did not change the level (bounce that has
 * already settled back), queue the finished element. Width-based glitch filtering is done
 * in the task because merging a short pulse needs the element after it. */
static void barcode_edge_isr(uint gpio, uint32_t events)
{
    (void)gpio;
    (void)events;
    const uint32_t now_us = time_us_32();
    const uint8_t black = barcode_pin_is_black();
    if (black == s_isr_level_black) return;

    const BarcodeElement element = {
        .width_us = now_us - s_isr_last_edge_us,
        .end_us = now_us,
        .is_black = s_isr_level_black,
    };
    s_isr_level_black = black;
    s_isr_last_edge_us = now_us;

    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_edge_queue, &element, &woken) != pdPASS) {
        s_isr_overflow = true;    /* current read is discarded by the task */
    }
    portYIELD_FROM_ISR(woken);
}
#endif

/* ------------------------------------------------------------------------- */
/* Barcode task                                                              */
/* ------------------------------------------------------------------------- */

static uint16_t barcode_to_command(const char *text)
{
    if (text[0] == BARCODE_LEFT_CHAR) return (uint16_t)BARCODE_CMD_LEFT;
    if (text[0] == BARCODE_RIGHT_CHAR) return (uint16_t)BARCODE_CMD_RIGHT;
    return (uint16_t)BARCODE_CMD_NONE;
}

typedef struct {
    uint32_t widths[CODE39_MAX_ELEMENTS];
    size_t count;
    bool active;
    bool overflow;
    BarcodeElement pending;   /* last element, held back so a following glitch can merge into it */
    uint32_t widest_us;
    uint32_t last_edge_us;
} BarcodeSession;

static void session_reset(BarcodeSession *s)
{
    s->count = 0U;
    s->active = false;
    s->overflow = false;
    s->widest_us = 0U;
}

static void session_push(BarcodeSession *s, uint32_t width_us)
{
    if (s->count < CODE39_MAX_ELEMENTS) {
        s->widths[s->count++] = width_us;
    } else {
        s->overflow = true;
    }
    if (width_us > s->widest_us) s->widest_us = width_us;
}

static void session_finish(BarcodeSession *s)
{
    session_push(s, s->pending.width_us);   /* final bar (trailing quiet zone has no closing edge) */

    const bool isr_overflow = s_isr_overflow;
    s_isr_overflow = false;

    BarcodeResult result;
    memset(&result, 0, sizeof(result));
    if (!s->overflow && !isr_overflow &&
        Code39_Decode(s->widths, s->count, result.text, sizeof(result.text))) {
        result.command = barcode_to_command(result.text);
        (void)xQueueOverwrite(s_result_queue, &result);
    }
    session_reset(s);
}

static TickType_t session_wait_ticks(const BarcodeSession *s)
{
    if (!s->active) return portMAX_DELAY;
    uint32_t end_gap_ms = (s->widest_us * BARCODE_END_GAP_FACTOR) / 1000U;
    if (end_gap_ms < BARCODE_END_GAP_MIN_MS) end_gap_ms = BARCODE_END_GAP_MIN_MS;
    const uint32_t elapsed_ms = (time_us_32() - s->last_edge_us) / 1000U;
    return (elapsed_ms >= end_gap_ms) ? 0U : pdMS_TO_TICKS(end_gap_ms - elapsed_ms);
}

static void barcode_task(void *argument)
{
    (void)argument;
    static BarcodeSession session;   /* static: keeps the width buffer off the task stack */
    session_reset(&session);

#if !ROBOT_SIMULATION
    gpio_init(PIN_IR_BARCODE);
    gpio_set_dir(PIN_IR_BARCODE, GPIO_IN);
    gpio_disable_pulls(PIN_IR_BARCODE);   /* module drives the line */
    s_isr_level_black = barcode_pin_is_black();
    s_isr_last_edge_us = time_us_32();
    configASSERT(Robot_GpioIrqRegister(PIN_IR_BARCODE, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       barcode_edge_isr));
#endif

    for (;;) {
        BarcodeElement element;
        if (xQueueReceive(s_edge_queue, &element, session_wait_ticks(&session)) != pdPASS) {
            if (session.active) session_finish(&session);     /* quiet zone reached */
            continue;
        }

        if (!session.active) {
            /* Idle: white before the first bar is the leading quiet zone; a read starts
             * at the first bar that is not a glitch. */
            if (element.is_black && element.width_us >= BARCODE_GLITCH_US) {
                session.active = true;
                session.pending = element;
                session.last_edge_us = element.end_us;
                session.widest_us = element.width_us;
            }
            continue;
        }

        session.last_edge_us = element.end_us;
        if (element.width_us < BARCODE_GLITCH_US || element.is_black == session.pending.is_black) {
            /* Debounce: a too-short pulse and the same-colour remainder after it are merged
             * into the element in progress instead of becoming separate bars/spaces. */
            session.pending.width_us += element.width_us;
            continue;
        }
        session_push(&session, session.pending.width_us);
        session.pending = element;
    }
}

/* ------------------------------------------------------------------------- */
/* Line tracking task                                                        */
/* ------------------------------------------------------------------------- */

typedef struct {
    uint16_t white;   /* ADC counts on floor */
    uint16_t black;   /* ADC counts on line */
    bool calibrated;
} IrCalibration;

static IrCalibration s_cal_left;
static IrCalibration s_cal_center;

#if !ROBOT_SIMULATION
static uint16_t ir_read(uint channel)
{
    adc_select_input(channel);
    return adc_read();
}

static void ir_calibration_apply(IrCalibration *cal, uint16_t min_counts, uint16_t max_counts)
{
    if ((uint32_t)(max_counts - min_counts) >= IR_MIN_CONTRAST_COUNTS) {
        cal->white = IR_ANALOG_BLACK_IS_HIGH ? min_counts : max_counts;
        cal->black = IR_ANALOG_BLACK_IS_HIGH ? max_counts : min_counts;
        cal->calibrated = true;
    } else {
        cal->white = IR_DEFAULT_WHITE_COUNTS;
        cal->black = IR_DEFAULT_BLACK_COUNTS;
        cal->calibrated = false;
    }
}

/* IR_CALIBRATION state: record min/max while the sensors are swept across line and floor. */
static void ir_calibrate(void)
{
    uint16_t left_min = 4095U, left_max = 0U, center_min = 4095U, center_max = 0U;
    const uint32_t start_ms = Robot_Millis();
    while ((Robot_Millis() - start_ms) < IR_CALIBRATION_MS) {
        const uint16_t left = ir_read(ADC_CHANNEL_IR_LEFT);
        const uint16_t center = ir_read(ADC_CHANNEL_IR_CENTER);
        if (left < left_min) left_min = left;
        if (left > left_max) left_max = left;
        if (center < center_min) center_min = center;
        if (center > center_max) center_max = center;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ir_calibration_apply(&s_cal_left, left_min, left_max);
    ir_calibration_apply(&s_cal_center, center_min, center_max);
}
#endif

/* 0.0 = white floor, 1.0 = black line. */
static float ir_normalise(uint16_t raw, const IrCalibration *cal)
{
    const float span = (float)cal->black - (float)cal->white;
    float value = ((float)raw - (float)cal->white) / span;
    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;
    return value;
}

static void line_task(void *argument)
{
    (void)argument;

#if ROBOT_SIMULATION
    s_cal_left = (IrCalibration){ .white = IR_DEFAULT_WHITE_COUNTS, .black = IR_DEFAULT_BLACK_COUNTS, .calibrated = true };
    s_cal_center = s_cal_left;
    uint32_t next_sim_barcode_ms = Robot_Millis() + 5000U;
    bool sim_left_next = true;
#else
    adc_init();
    adc_gpio_init(PIN_IR_LEFT);
    adc_gpio_init(PIN_IR_CENTER);
    ir_calibrate();
#endif

    float last_error = 0.0f;
    uint32_t junction_since_ms = 0U;
    bool junction_reported = false;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        const uint32_t now_ms = Robot_Millis();
        uint16_t raw_left = 0U;
        uint16_t raw_center = 0U;
        bool right_black = false;

#if ROBOT_SIMULATION
        /* Line slowly drifts between the two sensors. */
        const uint32_t phase = (now_ms / 20U) % 200U;
        const float drift = (phase < 100U) ? (float)phase / 100.0f : (float)(200U - phase) / 100.0f;
        raw_left = (uint16_t)(IR_DEFAULT_WHITE_COUNTS + (1.0f - drift) * (IR_DEFAULT_BLACK_COUNTS - IR_DEFAULT_WHITE_COUNTS));
        raw_center = (uint16_t)(IR_DEFAULT_WHITE_COUNTS + drift * (IR_DEFAULT_BLACK_COUNTS - IR_DEFAULT_WHITE_COUNTS));
        if ((int32_t)(now_ms - next_sim_barcode_ms) >= 0) {
            /* Inject "*L*" / "*R*" as if crossed at ~0.1 m/s (2 mm narrow, 5 mm wide). */
            uint32_t widths[CODE39_MAX_ELEMENTS];
            const size_t n = Code39_EncodeWidths(sim_left_next ? "L" : "R", 20000U, 50000U,
                                                 widths, CODE39_MAX_ELEMENTS);
            for (size_t i = 0U; i < n; ++i) {
                const BarcodeElement e = { .width_us = widths[i], .end_us = time_us_32(),
                                           .is_black = (uint8_t)((i % 2U) == 0U ? 1U : 0U) };
                (void)xQueueSend(s_edge_queue, &e, 0);
            }
            sim_left_next = !sim_left_next;
            next_sim_barcode_ms = now_ms + 10000U;
        }
#else
        raw_left = ir_read(ADC_CHANNEL_IR_LEFT);
        raw_center = ir_read(ADC_CHANNEL_IR_CENTER);
        right_black = barcode_pin_is_black() != 0U;
#endif

        const float left = ir_normalise(raw_left, &s_cal_left);
        const float center = ir_normalise(raw_center, &s_cal_center);
        const bool left_on = left >= IR_ON_LINE_THRESHOLD;
        const bool center_on = center >= IR_ON_LINE_THRESHOLD;
        const bool line_detected = left_on || center_on;

        /* Same sign convention as before (left only = -1, middle only = +1, both = 0),
         * but continuous so the controller gets proportional steering. */
        float error;
        if (line_detected) {
            error = center - left;
            last_error = error;
        } else {
            /* Line lost: saturate toward the side it was last seen so the robot turns back. */
            error = (last_error > 0.05f) ? 1.0f : ((last_error < -0.05f) ? -1.0f : 0.0f);
        }

        /* Junction: all three sensors black for JUNCTION_MIN_MS. The duration rejects
         * individual barcode bars passing under the right sensor. Reported once per crossing. */
        bool junction = false;
        if (left_on && center_on && right_black) {
            if (junction_since_ms == 0U) junction_since_ms = now_ms;
            if (!junction_reported && (now_ms - junction_since_ms) >= JUNCTION_MIN_MS) {
                junction = true;
                junction_reported = true;
            }
        } else {
            junction_since_ms = 0U;
            junction_reported = false;
        }

        SystemEvent event;
        memset(&event, 0, sizeof(event));
        event.type = SYSTEM_EVENT_LINE;
        event.timestamp_ms = now_ms;
        event.data.line.line_error = error;
        event.data.line.line_detected = line_detected;
        event.data.line.junction = junction;

        BarcodeResult result;
        if (xQueueReceive(s_result_queue, &result, 0) == pdPASS) {
            event.data.line.barcode_valid = true;
            event.data.line.barcode = result.command;
            memcpy(event.data.line.barcode_text, result.text, sizeof(event.data.line.barcode_text));
        }

        if (!Robot_PostEvent(&event) && event.data.line.barcode_valid) {
            (void)xQueueOverwrite(s_result_queue, &result);   /* queue full: retry next cycle */
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_LINE_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------------- */

void Buddy3LineBarcode_CreateTask(void)
{
    s_edge_queue = xQueueCreate(BARCODE_EDGE_QUEUE_LENGTH, sizeof(BarcodeElement));
    s_result_queue = xQueueCreate(1U, sizeof(BarcodeResult));
    configASSERT(s_edge_queue != NULL && s_result_queue != NULL);

    BaseType_t result = xTaskCreate(line_task, "LineTracking", ROBOT_TASK_STACK_WORDS,
                                    NULL, tskIDLE_PRIORITY + 3U, NULL);
    configASSERT(result == pdPASS);
    result = xTaskCreate(barcode_task, "Barcode", ROBOT_TASK_STACK_WORDS,
                         NULL, tskIDLE_PRIORITY + 3U, NULL);
    configASSERT(result == pdPASS);
}
