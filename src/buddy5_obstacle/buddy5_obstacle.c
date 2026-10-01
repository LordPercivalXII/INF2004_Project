#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "gpio_irq_router.h"
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "common_types.h"
#include "interfaces/buddy5_obstacle.h"

static volatile uint32_t echo_rise_us;
static volatile uint32_t echo_width_us;
static TaskHandle_t echo_wait_task;

static void echo_irq(uint gpio, uint32_t events)
{
    if (gpio != PIN_ULTRASONIC_ECHO) return;
    const uint32_t now = time_us_32();
    if ((events & GPIO_IRQ_EDGE_RISE) != 0U) {
        echo_rise_us = now;
    } else if ((events & GPIO_IRQ_EDGE_FALL) != 0U) {
        echo_width_us = now - echo_rise_us;
        if (echo_wait_task != NULL) {
            BaseType_t higher_priority_task_woken = pdFALSE;
            vTaskNotifyGiveFromISR(echo_wait_task, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
}

static void obstacle_hardware_init(void)
{
#if !ROBOT_SIMULATION
    gpio_init(PIN_ULTRASONIC_TRIGGER);
    gpio_set_dir(PIN_ULTRASONIC_TRIGGER, GPIO_OUT);
    gpio_put(PIN_ULTRASONIC_TRIGGER, false);
    gpio_init(PIN_ULTRASONIC_ECHO);
    gpio_set_dir(PIN_ULTRASONIC_ECHO, GPIO_IN);
    (void)Robot_GpioIrqRegister(PIN_ULTRASONIC_ECHO,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, &echo_irq);
    gpio_set_function(PIN_SCAN_SERVO, GPIO_FUNC_PWM);
    const uint slice = pwm_gpio_to_slice_num(PIN_SCAN_SERVO);
    pwm_set_clkdiv(slice, 125.0f);
    pwm_set_wrap(slice, 19999U);
    pwm_set_enabled(slice, true);
#endif
}

static void servo_set_angle(unsigned angle)
{
#if !ROBOT_SIMULATION
    const unsigned bounded = angle < SERVO_MIN_DEG ? SERVO_MIN_DEG :
                             (angle > SERVO_MAX_DEG ? SERVO_MAX_DEG : angle);
    const uint16_t pulse_us = (uint16_t)(500U + (bounded * 2000U / 180U));
    pwm_set_gpio_level(PIN_SCAN_SERVO, pulse_us);
#else
    (void)angle;
#endif
}

static float measure_distance_cm(uint32_t sample)
{
#if ROBOT_SIMULATION
    return sample % 9U == 0U ? 24.0f : 90.0f;
#else
    uint32_t width = 0U;
    echo_wait_task = xTaskGetCurrentTaskHandle();
    (void)ulTaskNotifyTake(pdTRUE, 0U);
    gpio_put(PIN_ULTRASONIC_TRIGGER, false);
    busy_wait_us_32(2U);
    gpio_put(PIN_ULTRASONIC_TRIGGER, true);
    busy_wait_us_32(10U);
    gpio_put(PIN_ULTRASONIC_TRIGGER, false);
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30)) > 0U) width = echo_width_us;
    echo_wait_task = NULL;
    if (width == 0U) return ULTRASONIC_MAX_CM;
    const float distance = (float)width * 0.01715f;
    return distance > ULTRASONIC_MAX_CM ? ULTRASONIC_MAX_CM : distance;
#endif
}

static void obstacle_task(void *argument)
{
    (void)argument;
    obstacle_hardware_init();
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t sample = 0U;
    for (;;) {
        servo_set_angle(SERVO_CENTER_DEG);
        const float center_distance = measure_distance_cm(sample++);
        if (center_distance < OBSTACLE_TRIGGER_CM) {
            float left_min = ULTRASONIC_MAX_CM;
            float right_min = ULTRASONIC_MAX_CM;
            unsigned obstacle_first_angle = SERVO_MAX_DEG;
            unsigned obstacle_last_angle = SERVO_MIN_DEG;
            bool profile_found = false;
            for (unsigned angle = SERVO_MIN_DEG; angle <= SERVO_MAX_DEG; angle += 20U) {
                servo_set_angle(angle);
                vTaskDelay(pdMS_TO_TICKS(60));
                const float distance = measure_distance_cm(sample++);
                if (angle < SERVO_CENTER_DEG && distance < left_min) left_min = distance;
                if (angle > SERVO_CENTER_DEG && distance < right_min) right_min = distance;
                if (distance < OBSTACLE_TRIGGER_CM) {
                    profile_found = true;
                    if (angle < obstacle_first_angle) obstacle_first_angle = angle;
                    if (angle > obstacle_last_angle) obstacle_last_angle = angle;
                }
            }
            if (!profile_found) {
                obstacle_first_angle = SERVO_CENTER_DEG - 15U;
                obstacle_last_angle = SERVO_CENTER_DEG + 15U;
                    }
            const unsigned fine_start = obstacle_first_angle > SERVO_MIN_DEG + 10U
                ? obstacle_first_angle - 10U : SERVO_MIN_DEG;
            const unsigned fine_end = obstacle_last_angle < SERVO_MAX_DEG - 10U
                ? obstacle_last_angle + 10U : SERVO_MAX_DEG;
            for (unsigned angle = fine_start; angle <= fine_end; angle += 5U) {
                servo_set_angle(angle);
                vTaskDelay(pdMS_TO_TICKS(45));
                const float distance = measure_distance_cm(sample++);
                if (angle < SERVO_CENTER_DEG && distance < left_min) left_min = distance;
                if (angle > SERVO_CENTER_DEG && distance < right_min) right_min = distance;
                if (distance < OBSTACLE_TRIGGER_CM) {
                    if (angle < obstacle_first_angle) obstacle_first_angle = angle;
                    if (angle > obstacle_last_angle) obstacle_last_angle = angle;
                }
            }
            const float left_clearance = left_min >= OBSTACLE_TRIGGER_CM ? left_min : 0.0f;
            const float right_clearance = right_min >= OBSTACLE_TRIGGER_CM ? right_min : 0.0f;
            const float angular_width = obstacle_last_angle >= obstacle_first_angle
                ? (float)(obstacle_last_angle - obstacle_first_angle) * 0.017453293f : 0.0f;
            SystemEvent event = { .type = SYSTEM_EVENT_OBSTACLE, .timestamp_ms = Robot_Millis() };
            event.data.obstacle.detected = true;
            event.data.obstacle.scan_complete = true;
            event.data.obstacle.center_distance_cm = center_distance;
            event.data.obstacle.estimated_width_cm = 2.0f * center_distance * tanf(angular_width * 0.5f);
            event.data.obstacle.left_clearance_cm = left_clearance;
            event.data.obstacle.right_clearance_cm = right_clearance;
            event.data.obstacle.bypass_direction = left_clearance >= right_clearance ? -1 : 1;
            (void)Robot_PostEvent(&event);
            servo_set_angle(SERVO_CENTER_DEG);
            while (measure_distance_cm(sample++) < OBSTACLE_TRIGGER_CM) {
                vTaskDelay(pdMS_TO_TICKS(ROBOT_OBSTACLE_PERIOD_MS));
            }
            event.timestamp_ms = Robot_Millis();
            event.data.obstacle.detected = false;
            event.data.obstacle.scan_complete = true;
            (void)Robot_PostEvent(&event);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_OBSTACLE_PERIOD_MS));
    }
}

void Buddy5Obstacle_CreateTask(void)
{
    BaseType_t result = xTaskCreate(obstacle_task, "ObstacleScan", ROBOT_TASK_STACK_WORDS,
                                    NULL, tskIDLE_PRIORITY + 3U, NULL);
    configASSERT(result == pdPASS);
}