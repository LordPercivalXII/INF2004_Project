#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "common_types.h"
#include "interfaces/buddy3_line_barcode.h"

static void line_hardware_init(void)
{
#if !ROBOT_SIMULATION
    const uint pins[] = { PIN_IR_LEFT, PIN_IR_CENTER, PIN_IR_BARCODE };
    for (unsigned i = 0U; i < 3U; ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }
#endif
}

static void line_task(void *argument)
{
    (void)argument;
    line_hardware_init();
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t previous_barcode_level = 1U;
    uint32_t barcode_bits = 0U;
    uint8_t barcode_bit_count = 0U;
    uint32_t last_edge_ms = Robot_Millis();
    for (;;) {
        bool left = false;
        bool center = false;
        bool barcode_level = true;
#if ROBOT_SIMULATION
        const uint32_t phase = (Robot_Millis() / 700U) % 3U;
        left = phase == 0U;
        center = phase != 2U;
        barcode_level = true;
#else
        left = !gpio_get(PIN_IR_LEFT);
        center = !gpio_get(PIN_IR_CENTER);
        barcode_level = gpio_get(PIN_IR_BARCODE);
#endif
        float error = 0.0f;
        if (left && !center) error = -1.0f;
        else if (!left && center) error = 1.0f;
        else if (!left && !center) error = 0.0f;

        const uint32_t now_ms = Robot_Millis();
        bool decoded = false;
        uint16_t code = 0U;
        if (barcode_level != (previous_barcode_level != 0U)) {
            const uint32_t pulse_ms = now_ms - last_edge_ms;
            last_edge_ms = now_ms;
            if (pulse_ms > 35U) {
                barcode_bits = 0U;
                barcode_bit_count = 0U;
            } else if (pulse_ms >= 4U && pulse_ms <= 30U && barcode_bit_count < 8U) {
                barcode_bits = (barcode_bits << 1U) | (pulse_ms >= 14U ? 1U : 0U);
                ++barcode_bit_count;
                if (barcode_bit_count == 8U) {
                    code = (uint16_t)barcode_bits;
                    decoded = true;
                    barcode_bit_count = 0U;
                }
            }
            previous_barcode_level = barcode_level ? 1U : 0U;
        }
        SystemEvent event = { .type = SYSTEM_EVENT_LINE, .timestamp_ms = now_ms };
        event.data.line.line_error = error;
        event.data.line.line_detected = left || center;
        event.data.line.junction = left && center;
        event.data.line.barcode_valid = decoded;
        event.data.line.barcode = code;
        (void)Robot_PostEvent(&event);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_LINE_PERIOD_MS));
    }
}

void Buddy3LineBarcode_CreateTask(void)
{
    BaseType_t result = xTaskCreate(line_task, "LineBarcode", ROBOT_TASK_STACK_WORDS,
                                    NULL, tskIDLE_PRIORITY + 3U, NULL);
    configASSERT(result == pdPASS);
}