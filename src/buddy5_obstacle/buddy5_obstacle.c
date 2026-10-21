#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "gpio_irq_router.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "config.h"
#include "common_types.h"
#include "interfaces/buddy5_obstacle.h"

#define DEGREES_TO_RADIANS 0.017453292519943295f

#if !ROBOT_SIMULATION
static QueueHandle_t echo_queue;
static volatile uint32_t echo_rise_us;
static volatile bool echo_waiting_for_fall;
static volatile bool echo_measurement_active;
static volatile uint32_t echo_queue_drops;
static TickType_t last_trigger_tick;
static bool trigger_started;
#endif
static unsigned current_servo_angle = ULTRASONIC_COARSE_CENTER_DEG;

#if !ROBOT_SIMULATION
static void echo_irq(uint gpio, uint32_t events)
{
    if (gpio != PIN_ULTRASONIC_ECHO || !echo_measurement_active) return;

    if ((events & GPIO_IRQ_EDGE_RISE) != 0U) {
        echo_rise_us = time_us_32();
        echo_waiting_for_fall = true;
    } else if ((events & GPIO_IRQ_EDGE_FALL) != 0U && echo_waiting_for_fall) {
        const uint32_t echo_width_us = time_us_32() - echo_rise_us;
        BaseType_t higher_priority_task_woken = pdFALSE;
        echo_waiting_for_fall = false;
        if (xQueueSendFromISR(echo_queue, &echo_width_us, &higher_priority_task_woken) != pdPASS) {
            echo_queue_drops++;
        }
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
#endif

static bool obstacle_hardware_init(void)
{
#if !ROBOT_SIMULATION
    echo_queue = xQueueCreate(ULTRASONIC_ECHO_QUEUE_LENGTH, sizeof(uint32_t));
    if (echo_queue == NULL) return false;

    gpio_init(PIN_ULTRASONIC_TRIGGER);
    gpio_set_dir(PIN_ULTRASONIC_TRIGGER, GPIO_OUT);
    gpio_put(PIN_ULTRASONIC_TRIGGER, false);
    gpio_init(PIN_ULTRASONIC_ECHO);
    gpio_set_dir(PIN_ULTRASONIC_ECHO, GPIO_IN);

    if (!Robot_GpioIrqRegister(PIN_ULTRASONIC_ECHO,
                               GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, &echo_irq)) {
        vQueueDelete(echo_queue);
        echo_queue = NULL;
        return false;
    }

    gpio_set_function(PIN_SCAN_SERVO, GPIO_FUNC_PWM);
    const uint slice = pwm_gpio_to_slice_num(PIN_SCAN_SERVO);
    pwm_set_clkdiv(slice, 125.0f);
    pwm_set_wrap(slice, 19999U);
    pwm_set_enabled(slice, true);
#endif
    return true;
}

static void servo_set_angle(unsigned angle)
{
#if !ROBOT_SIMULATION
    const unsigned bounded = angle < ULTRASONIC_COARSE_LEFT_DEG ? ULTRASONIC_COARSE_LEFT_DEG :
                             (angle > ULTRASONIC_COARSE_RIGHT_DEG ?
                              ULTRASONIC_COARSE_RIGHT_DEG : angle);
    const uint16_t pulse_us = (uint16_t)(500U + (bounded * 2000U / 180U));
    pwm_set_gpio_level(PIN_SCAN_SERVO, pulse_us);
#endif
    current_servo_angle = angle;
}

#if !ROBOT_SIMULATION
static void wait_for_ping_interval(void)
{
    if (trigger_started) {
        const TickType_t interval = pdMS_TO_TICKS(ULTRASONIC_MIN_PING_INTERVAL_MS);
        const TickType_t elapsed = xTaskGetTickCount() - last_trigger_tick;
        if (elapsed < interval) vTaskDelay(interval - elapsed);
    }
    last_trigger_tick = xTaskGetTickCount();
    trigger_started = true;
}
#endif

static float measure_distance_cm(uint32_t scan_number)
{
#if ROBOT_SIMULATION
    if (scan_number % 12U == 0U &&
        current_servo_angle == ULTRASONIC_COARSE_CENTER_DEG) {
        return 24.0f;
    }
    return current_servo_angle < ULTRASONIC_COARSE_CENTER_DEG ? 90.0f :
           (current_servo_angle > ULTRASONIC_COARSE_CENTER_DEG ? 60.0f : 100.0f);
#else
    (void)scan_number;
    uint32_t echo_width_us = 0U;
    while (xQueueReceive(echo_queue, &echo_width_us, 0U) == pdPASS) {}
    echo_waiting_for_fall = false;
    wait_for_ping_interval();
    echo_measurement_active = true;
    gpio_put(PIN_ULTRASONIC_TRIGGER, false);
    busy_wait_us_32(2U);
    gpio_put(PIN_ULTRASONIC_TRIGGER, true);
    busy_wait_us_32(10U);
    gpio_put(PIN_ULTRASONIC_TRIGGER, false);
    const BaseType_t received = xQueueReceive(echo_queue, &echo_width_us,
                                               pdMS_TO_TICKS(ULTRASONIC_ECHO_TIMEOUT_MS));
    echo_measurement_active = false;
    if (received != pdPASS || echo_width_us == 0U) return ULTRASONIC_MAX_DISTANCE_CM;

    const float distance = (float)echo_width_us * 0.0343f * 0.5f;
    if (distance < ULTRASONIC_MIN_DISTANCE_CM) return ULTRASONIC_MIN_DISTANCE_CM;
    return distance > ULTRASONIC_MAX_DISTANCE_CM ? ULTRASONIC_MAX_DISTANCE_CM : distance;
#endif
}

static void include_measurement(unsigned angle, float distance, float *left_clearance,
                                float *right_clearance, bool *detected, float *minimum_hit,
                                float *minimum_lateral, float *maximum_lateral,
                                bool *has_hit)
{
    if (angle < ULTRASONIC_COARSE_CENTER_DEG && distance < *left_clearance) {
        *left_clearance = distance;
    } else if (angle > ULTRASONIC_COARSE_CENTER_DEG && distance < *right_clearance) {
        *right_clearance = distance;
    }

    if (distance <= ULTRASONIC_THRESHOLD_CM) {
        const float lateral = distance *
            sinf(((float)angle - (float)ULTRASONIC_COARSE_CENTER_DEG) * DEGREES_TO_RADIANS);
        *detected = true;
        if (!*has_hit) {
            *minimum_lateral = lateral;
            *maximum_lateral = lateral;
            *minimum_hit = distance;
            *has_hit = true;
        } else {
            if (lateral < *minimum_lateral) *minimum_lateral = lateral;
            if (lateral > *maximum_lateral) *maximum_lateral = lateral;
            if (distance < *minimum_hit) *minimum_hit = distance;
        }
    }
}

static int8_t choose_bypass_direction(float left_clearance, float right_clearance)
{
    const bool left_is_clear = left_clearance > ULTRASONIC_THRESHOLD_CM;
    const bool right_is_clear = right_clearance > ULTRASONIC_THRESHOLD_CM;
    if (!left_is_clear && !right_is_clear) return 0;
    if (!left_is_clear) return 1;
    if (!right_is_clear) return -1;
    return left_clearance >= right_clearance ? -1 : 1;
}

static float estimate_obstacle_width(float minimum_lateral, float maximum_lateral,
                                     float minimum_hit, bool has_hit)
{
    if (!has_hit) return 0.0f;
    const float beam_half_angle =
        (float)ULTRASONIC_BEAM_WIDTH_DEG * 0.5f * DEGREES_TO_RADIANS;
    const float beam_footprint = 2.0f * minimum_hit * tanf(beam_half_angle);
    return maximum_lateral - minimum_lateral + beam_footprint;
}

static void publish_obstacle_event(const SystemEvent *event)
{
    static bool delivery_error_reported;
    if (!Robot_PostEvent(event)) {
        if (!delivery_error_reported) {
            printf("Buddy 5: system event queue full; event was not delivered\n");
            delivery_error_reported = true;
        }
    } else {
        delivery_error_reported = false;
    }
}

static void obstacle_task(void *argument)
{
    (void)argument;
    if (!obstacle_hardware_init()) {
        SystemEvent fault = { .type = SYSTEM_EVENT_FAULT, .timestamp_ms = Robot_Millis() };
        printf("Buddy 5: ultrasonic hardware initialization failed\n");
        publish_obstacle_event(&fault);
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000U));
    }

    uint32_t scan_number = 0U;
    const unsigned coarse_angles[] = {
        ULTRASONIC_COARSE_LEFT_DEG,
        ULTRASONIC_COARSE_CENTER_DEG,
        ULTRASONIC_COARSE_RIGHT_DEG
    };

    for (;;) {
        float left_clearance = ULTRASONIC_MAX_DISTANCE_CM;
        float right_clearance = ULTRASONIC_MAX_DISTANCE_CM;
        float minimum_hit = ULTRASONIC_MAX_DISTANCE_CM;
        float minimum_lateral = 0.0f;
        float maximum_lateral = 0.0f;
        bool detected = false;
        bool has_hit = false;
        unsigned first_hit_angle = ULTRASONIC_COARSE_RIGHT_DEG;
        unsigned last_hit_angle = ULTRASONIC_COARSE_LEFT_DEG;
        float center_distance = ULTRASONIC_MAX_DISTANCE_CM;

        for (unsigned index = 0U; index < 3U; ++index) {
            const unsigned angle = coarse_angles[index];
            servo_set_angle(angle);
            vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SERVO_SETTLE_MS));
            const float distance = measure_distance_cm(scan_number);
            include_measurement(angle, distance, &left_clearance, &right_clearance,
                                &detected, &minimum_hit, &minimum_lateral,
                                &maximum_lateral, &has_hit);
            if (angle == ULTRASONIC_COARSE_CENTER_DEG) center_distance = distance;
            if (distance <= ULTRASONIC_THRESHOLD_CM) {
                if (angle < first_hit_angle) first_hit_angle = angle;
                if (angle > last_hit_angle) last_hit_angle = angle;
            }
        }

        if (detected) {
            const unsigned fine_start = first_hit_angle > ULTRASONIC_COARSE_LEFT_DEG +
                                        ULTRASONIC_FINE_MARGIN_DEG ?
                                        first_hit_angle - ULTRASONIC_FINE_MARGIN_DEG :
                                        ULTRASONIC_COARSE_LEFT_DEG;
            const unsigned fine_end = last_hit_angle < ULTRASONIC_COARSE_RIGHT_DEG -
                                      ULTRASONIC_FINE_MARGIN_DEG ?
                                      last_hit_angle + ULTRASONIC_FINE_MARGIN_DEG :
                                      ULTRASONIC_COARSE_RIGHT_DEG;

            for (unsigned angle = fine_start; angle <= fine_end;
                 angle += ULTRASONIC_FINE_STEP_DEG) {
                servo_set_angle(angle);
                vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SERVO_SETTLE_MS));
                const float distance = measure_distance_cm(scan_number);
                include_measurement(angle, distance, &left_clearance, &right_clearance,
                                    &detected, &minimum_hit, &minimum_lateral,
                                    &maximum_lateral, &has_hit);
            }
        }

        SystemEvent event = { .type = SYSTEM_EVENT_OBSTACLE, .timestamp_ms = Robot_Millis() };
        event.data.obstacle.detected = detected;
        event.data.obstacle.scan_complete = true;
        event.data.obstacle.center_distance_cm = center_distance;
        event.data.obstacle.estimated_width_cm =
            estimate_obstacle_width(minimum_lateral, maximum_lateral, minimum_hit, has_hit);
        event.data.obstacle.left_clearance_cm =
            left_clearance <= ULTRASONIC_THRESHOLD_CM ? 0.0f : left_clearance;
        event.data.obstacle.right_clearance_cm =
            right_clearance <= ULTRASONIC_THRESHOLD_CM ? 0.0f : right_clearance;
        event.data.obstacle.bypass_direction =
            detected ? choose_bypass_direction(event.data.obstacle.left_clearance_cm,
                                               event.data.obstacle.right_clearance_cm) : 0;
        publish_obstacle_event(&event);
        servo_set_angle(ULTRASONIC_COARSE_CENTER_DEG);
        scan_number++;

#if !ROBOT_SIMULATION
        if (echo_queue_drops != 0U) {
            printf("Buddy 5: dropped %lu ultrasonic echo capture(s)\n",
                   (unsigned long)echo_queue_drops);
            echo_queue_drops = 0U;
        }
#endif
    }
}

void Buddy5Obstacle_CreateTask(void)
{
    BaseType_t result = xTaskCreate(obstacle_task, "ObstacleScan", ROBOT_TASK_STACK_WORDS,
                                    NULL, tskIDLE_PRIORITY + 3U, NULL);
    configASSERT(result == pdPASS);
}