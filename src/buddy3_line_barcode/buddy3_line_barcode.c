/*
 * Buddy 3 (Wicus): IR line tracking + Code 39 barcode decoding.
 *
 * Two RTOS tasks, matching the Design Review task table:
 *   IR sample ISR - repeating hardware timer at IR_SAMPLE_RATE_HZ (1 kHz). Reads left/middle
 *                   IR on ADC0/ADC1 (GP26/27) and the right IR level (GP28) into a ring buffer.
 *   LineTracking  - periodic 20 ms (NFR6). Drains the ring buffer and averages the ~20 samples,
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
#include "hardware/sync.h"
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
/* ---- IR sampling ISR -> ring buffer -------------------------------------
 * Single producer (timer ISR) / single consumer (LineTracking task), lock-free:
 * only the ISR writes s_ir_head, only the task writes s_ir_tail. The ISR is the
 * only code that touches the ADC once sampling has started. */

#if (IR_RING_SIZE & (IR_RING_SIZE - 1U)) != 0U
#error "IR_RING_SIZE must be a power of two"
#endif

typedef struct {
    uint16_t left;
    uint16_t center;
    uint8_t right_black;
} IrSample;

typedef struct {
    uint32_t count;
    uint32_t left_sum;
    uint32_t center_sum;
    uint32_t right_black_count;
} IrWindow;

static IrSample s_ir_ring[IR_RING_SIZE];
static volatile uint32_t s_ir_head;      /* written by ISR only */
static volatile uint32_t s_ir_tail;      /* written by task only */
static volatile uint32_t s_ir_overruns;  /* samples dropped because the task fell behind */
static repeating_timer_t s_ir_timer;

/* Runs in the SDK alarm IRQ every 1 ms. No FreeRTOS calls, ~5 us of work. */
static bool ir_sample_isr(repeating_timer_t *timer)
{
    (void)timer;
    const uint32_t head = s_ir_head;
    if ((head - s_ir_tail) >= IR_RING_SIZE) {
        ++s_ir_overruns;                 /* buffer full: drop this sample, keep older ones */
        return true;
    }
    IrSample *sample = &s_ir_ring[head & (IR_RING_SIZE - 1U)];
    adc_select_input(ADC_CHANNEL_IR_LEFT);
    sample->left = adc_read();
    adc_select_input(ADC_CHANNEL_IR_CENTER);
    sample->center = adc_read();
    sample->right_black = barcode_pin_is_black();
    __dmb();                             /* sample fully written before it is published */
    s_ir_head = head + 1U;
    return true;                         /* keep repeating */
}

static void ir_sampling_start(void)
{
    adc_init();
    adc_gpio_init(PIN_IR_LEFT);
    adc_gpio_init(PIN_IR_CENTER);
    s_ir_head = 0U;
    s_ir_tail = 0U;
    /* Negative period = fixed interval between sample starts (no drift). */
    const bool started = add_repeating_timer_us(-(int64_t)(1000000U / IR_SAMPLE_RATE_HZ),
                                                ir_sample_isr, NULL, &s_ir_timer);
    configASSERT(started);
}

/* Move every sample published since the last call into a summary window. */
static void ir_drain(IrWindow *window)
{
    memset(window, 0, sizeof(*window));
    const uint32_t head = s_ir_head;
    __dmb();                             /* read samples only after seeing the new head */
    uint32_t tail = s_ir_tail;
    while (tail != head) {
        const IrSample *sample = &s_ir_ring[tail & (IR_RING_SIZE - 1U)];
        window->left_sum += sample->left;
        window->center_sum += sample->center;
        window->right_black_count += sample->right_black;
        ++window->count;
        ++tail;
    }
    __dmb();
    s_ir_tail = tail;                    /* release the slots back to the ISR */
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

/* IR_CALIBRATION state: sweep the sensors across line and floor. Min/max are taken over
 * ~5-sample averages from the ISR buffer, so a single noisy sample cannot set a threshold. */
static void ir_calibrate(void)
{
    uint16_t left_min = 4095U, left_max = 0U, center_min = 4095U, center_max = 0U;
    const uint32_t start_ms = Robot_Millis();
    while ((Robot_Millis() - start_ms) < IR_CALIBRATION_MS) {
        vTaskDelay(pdMS_TO_TICKS(5));
        IrWindow window;
        ir_drain(&window);
        if (window.count == 0U) continue;
        const uint16_t left = (uint16_t)(window.left_sum / window.count);
        const uint16_t center = (uint16_t)(window.center_sum / window.count);
        if (left < left_min) left_min = left;
        if (left > left_max) left_max = left;
        if (center < center_min) center_min = center;
        if (center > center_max) center_max = center;
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
    ir_sampling_start();
    ir_calibrate();
    uint16_t held_left = IR_DEFAULT_WHITE_COUNTS;     /* reused if a window is ever empty */
    uint16_t held_center = IR_DEFAULT_WHITE_COUNTS;
    bool held_right_black = false;
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
        /* Average everything the ISR sampled since the last cycle (~20 samples at 1 kHz). */
        IrWindow window;
        ir_drain(&window);
        if (window.count > 0U) {
            held_left = (uint16_t)(window.left_sum / window.count);
            held_center = (uint16_t)(window.center_sum / window.count);
            held_right_black = (2U * window.right_black_count) > window.count;   /* majority */
        }
        raw_left = held_left;
        raw_center = held_center;
        right_black = held_right_black;
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