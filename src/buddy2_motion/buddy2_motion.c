#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "gpio_irq_router.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "config.h"
#include "common_types.h"
#include "interfaces/buddy2_motion.h"

static volatile int32_t left_encoder_ticks;
static volatile int32_t right_encoder_ticks;

static void encoder_irq(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == PIN_ENCODER_LEFT_A) {
        left_encoder_ticks += gpio_get(PIN_ENCODER_LEFT_B) ? 1 : -1;
    } else if (gpio == PIN_ENCODER_RIGHT_A) {
        right_encoder_ticks += gpio_get(PIN_ENCODER_RIGHT_B) ? -1 : 1;
    }
}

static void set_motor(uint pwm_pin, uint direction_pin, float duty)
{
    duty = fminf(1.0f, fmaxf(-1.0f, duty));
    gpio_put(direction_pin, duty >= 0.0f);
    const uint slice = pwm_gpio_to_slice_num(pwm_pin);
    const uint channel = pwm_gpio_to_channel(pwm_pin);
    const uint16_t level = (uint16_t)(fabsf(duty) * 65535.0f);
    pwm_set_chan_level(slice, channel, level);
}

static void motor_hardware_init(void)
{
#if !ROBOT_SIMULATION
    const uint pwm_pins[] = { PIN_MOTOR_LEFT_PWM, PIN_MOTOR_RIGHT_PWM };
    const uint dir_pins[] = { PIN_MOTOR_LEFT_DIR, PIN_MOTOR_RIGHT_DIR };
    for (unsigned index = 0U; index < 2U; ++index) {
        gpio_set_function(pwm_pins[index], GPIO_FUNC_PWM);
        gpio_init(dir_pins[index]);
        gpio_set_dir(dir_pins[index], GPIO_OUT);
        pwm_set_wrap(pwm_gpio_to_slice_num(pwm_pins[index]), 65535U);
        pwm_set_enabled(pwm_gpio_to_slice_num(pwm_pins[index]), true);
    }
    const uint encoders[] = { PIN_ENCODER_LEFT_A, PIN_ENCODER_LEFT_B,
                              PIN_ENCODER_RIGHT_A, PIN_ENCODER_RIGHT_B };
    for (unsigned index = 0U; index < 4U; ++index) {
        gpio_init(encoders[index]);
        gpio_set_dir(encoders[index], GPIO_IN);
        gpio_pull_up(encoders[index]);
    }
    (void)Robot_GpioIrqRegister(PIN_ENCODER_LEFT_A, GPIO_IRQ_EDGE_RISE, &encoder_irq);
    (void)Robot_GpioIrqRegister(PIN_ENCODER_RIGHT_A, GPIO_IRQ_EDGE_RISE, &encoder_irq);
#endif
}

bool Buddy2Motion_SendCommand(const MotionCommand *command)
{
    if (command == NULL || g_motion_command_queue == NULL) return false;
    return xQueueSend(g_motion_command_queue, command, 0) == pdPASS;
}

static void motion_task(void *argument)
{
    (void)argument;
    motor_hardware_init();
    MotionCommand requested = { .type = MOTION_STOP, .left_speed = 0.0f, .right_speed = 0.0f };
    MotionCommand incoming;
    int32_t previous_left = 0;
    int32_t previous_right = 0;
    float left_integral = 0.0f;
    float right_integral = 0.0f;
    float previous_left_error = 0.0f;
    float previous_right_error = 0.0f;
    float total_distance = 0.0f;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        while (xQueueReceive(g_motion_command_queue, &incoming, 0) == pdPASS) requested = incoming;
        int32_t left_now = 0;
        int32_t right_now = 0;
#if ROBOT_SIMULATION
        if (requested.type == MOTION_DRIVE) {
            left_encoder_ticks += requested.left_speed >= 0.0f ? 2 : -2;
            right_encoder_ticks += requested.right_speed >= 0.0f ? 2 : -2;
        }
#endif
        const uint32_t irq_state = save_and_disable_interrupts();
        left_now = left_encoder_ticks;
        right_now = right_encoder_ticks;
        restore_interrupts(irq_state);

        const int32_t left_delta = left_now - previous_left;
        const int32_t right_delta = right_now - previous_right;
        previous_left = left_now;
        previous_right = right_now;
        const float interval_s = (float)ROBOT_MOTION_PERIOD_MS / 1000.0f;
        const float meters_per_tick = (3.14159265f * MOTOR_WHEEL_DIAMETER_M) / MOTOR_ENCODER_TICKS_PER_REV;
        const float measured_left = ((float)left_delta * meters_per_tick) / interval_s;
        const float measured_right = ((float)right_delta * meters_per_tick) / interval_s;
        total_distance += ((float)(left_delta + right_delta) * 0.5f) * meters_per_tick;

        float left_output = 0.0f;
        float right_output = 0.0f;
        if (requested.type == MOTION_DRIVE) {
            const float left_error = requested.left_speed - measured_left;
            const float right_error = requested.right_speed - measured_right;
            left_integral = fminf(1.0f, fmaxf(-1.0f, left_integral + left_error * interval_s));
            right_integral = fminf(1.0f, fmaxf(-1.0f, right_integral + right_error * interval_s));
            const float left_derivative = (left_error - previous_left_error) / interval_s;
            const float right_derivative = (right_error - previous_right_error) / interval_s;
            left_output = requested.left_speed + MOTOR_PID_KP * left_error +
                          MOTOR_PID_KI * left_integral + MOTOR_PID_KD * left_derivative;
            right_output = requested.right_speed + MOTOR_PID_KP * right_error +
                           MOTOR_PID_KI * right_integral + MOTOR_PID_KD * right_derivative;
            previous_left_error = left_error;
            previous_right_error = right_error;
        } else {
            left_integral = 0.0f;
            right_integral = 0.0f;
            previous_left_error = 0.0f;
            previous_right_error = 0.0f;
        }
#if !ROBOT_SIMULATION
        set_motor(PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_DIR, left_output);
        set_motor(PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_DIR, right_output);
#else
        (void)set_motor;
#endif
        SystemEvent event = { .type = SYSTEM_EVENT_MOTION, .timestamp_ms = Robot_Millis() };
        event.data.motion.left_ticks = left_now;
        event.data.motion.right_ticks = right_now;
        event.data.motion.left_speed_m_s = measured_left;
        event.data.motion.right_speed_m_s = measured_right;
        event.data.motion.distance_m = total_distance;
        (void)Robot_PostEvent(&event);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_MOTION_PERIOD_MS));
    }
}

void Buddy2Motion_CreateTask(void)
{
    BaseType_t result = xTaskCreate(motion_task, "MotionPID", ROBOT_TASK_STACK_WORDS,
                                    NULL, tskIDLE_PRIORITY + 4U, NULL);
    configASSERT(result == pdPASS);
}