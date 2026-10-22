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

#define MOTION_PI 3.14159265f

static volatile int32_t left_encoder_ticks;
static volatile int32_t right_encoder_ticks;

#if !ROBOT_SIMULATION
static void encoder_irq(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == PIN_ENCODER_LEFT_A) {
        left_encoder_ticks += gpio_get(PIN_ENCODER_LEFT_B) ? 1 : -1;
    } else if (gpio == PIN_ENCODER_RIGHT_A) {
        right_encoder_ticks += gpio_get(PIN_ENCODER_RIGHT_B) ? -1 : 1;
    }
}
#endif

static float clamp(float value, float minimum, float maximum)
{
    return fminf(maximum, fmaxf(minimum, value));
}

static void set_motor(uint pwm_pin, uint direction_pin, float duty)
{
    duty = clamp(duty, -1.0f, 1.0f);
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
        gpio_put(dir_pins[index], false);
        pwm_set_wrap(pwm_gpio_to_slice_num(pwm_pins[index]), 65535U);
        pwm_set_chan_level(pwm_gpio_to_slice_num(pwm_pins[index]),
                           pwm_gpio_to_channel(pwm_pins[index]), 0U);
        pwm_set_enabled(pwm_gpio_to_slice_num(pwm_pins[index]), true);
    }

    const uint encoders[] = { PIN_ENCODER_LEFT_A, PIN_ENCODER_LEFT_B,
                              PIN_ENCODER_RIGHT_A, PIN_ENCODER_RIGHT_B };
    for (unsigned index = 0U; index < 4U; ++index) {
        gpio_init(encoders[index]);
        gpio_set_dir(encoders[index], GPIO_IN);
        gpio_pull_up(encoders[index]);
    }
    configASSERT(Robot_GpioIrqRegister(PIN_ENCODER_LEFT_A, GPIO_IRQ_EDGE_RISE, &encoder_irq));
    configASSERT(Robot_GpioIrqRegister(PIN_ENCODER_RIGHT_A, GPIO_IRQ_EDGE_RISE, &encoder_irq));
#endif
}

static bool motion_command_is_valid(const MotionCommand *command)
{
    if (command == NULL) return false;
    switch (command->type) {
    case MOTION_STOP:
    case MOTION_BRAKE:
        return true;
    case MOTION_DRIVE:
        return isfinite(command->left_speed) && isfinite(command->right_speed);
    case MOTION_TURN:
        return isfinite(command->turn_angle_deg) &&
               fabsf(command->turn_angle_deg) > 0.0f &&
               fabsf(command->turn_angle_deg) <= 360.0f &&
               command->turn_id != 0U;
    default:
        return false;
    }
}

static float command_wheel_speed(const MotionCommand *command, bool left_wheel)
{
    if (command->type == MOTION_DRIVE) {
        const float speed = left_wheel ? command->left_speed : command->right_speed;
        return clamp(speed, -ROBOT_MAX_SPEED, ROBOT_MAX_SPEED);
    }
    if (command->type == MOTION_TURN) {
        const float direction = command->turn_angle_deg > 0.0f ? 1.0f : -1.0f;
        return left_wheel ? -direction * MOTOR_TURN_SPEED_M_S :
                            direction * MOTOR_TURN_SPEED_M_S;
    }
    return 0.0f;
}

bool Buddy2Motion_SendCommand(const MotionCommand *command)
{
    if (!motion_command_is_valid(command) || g_motion_command_queue == NULL) return false;
    return xQueueOverwrite(g_motion_command_queue, command) == pdPASS;
}

static float speed_pid(float target_speed, float measured_speed, float interval_s,
                       float *integral, float *previous_error)
{
    const float error = target_speed - measured_speed;
    const float derivative = (error - *previous_error) / interval_s;
    const float old_integral = *integral;
    const float candidate_integral = clamp(old_integral + error * interval_s, -1.0f, 1.0f);
    float output = target_speed + MOTOR_PID_KP * error +
                   MOTOR_PID_KI * candidate_integral + MOTOR_PID_KD * derivative;

    if (fabsf(output) > 1.0f && error * output > 0.0f) {
        output = target_speed + MOTOR_PID_KP * error +
                 MOTOR_PID_KI * old_integral + MOTOR_PID_KD * derivative;
    } else {
        *integral = candidate_integral;
    }
    *previous_error = error;
    return clamp(output, -1.0f, 1.0f);
}

#if ROBOT_SIMULATION
static void simulate_encoder_motion(float speed, float interval_s, float meters_per_tick,
                                    float *fractional_ticks, volatile int32_t *ticks)
{
    const float accumulated = *fractional_ticks + speed * interval_s / meters_per_tick;
    const int32_t whole_ticks = (int32_t)accumulated;
    *fractional_ticks = accumulated - (float)whole_ticks;
    *ticks += whole_ticks;
}
#endif

static void motion_task(void *argument)
{
    (void)argument;
    motor_hardware_init();
    configASSERT(MOTOR_TRACK_WIDTH_M > 0.0f);

    MotionCommand requested = {
        .type = MOTION_STOP,
        .left_speed = 0.0f,
        .right_speed = 0.0f,
        .turn_angle_deg = 0.0f
    };
    MotionCommand incoming;
    int32_t previous_left = 0;
    int32_t previous_right = 0;
    float left_integral = 0.0f;
    float right_integral = 0.0f;
    float previous_left_error = 0.0f;
    float previous_right_error = 0.0f;
    float total_distance = 0.0f;
    float turn_progress_deg = 0.0f;
    uint32_t active_turn_id = 0U;
    uint32_t completed_turn_id = 0U;
    uint32_t left_last_tick_ms = Robot_Millis();
    uint32_t right_last_tick_ms = left_last_tick_ms;
    bool encoder_fault_latched = false;
    bool encoder_fault_pending = false;
#if ROBOT_SIMULATION
    float simulated_left_fraction = 0.0f;
    float simulated_right_fraction = 0.0f;
#endif
    const float interval_s = (float)ROBOT_MOTION_PERIOD_MS / 1000.0f;
    const float meters_per_tick =
        (MOTION_PI * MOTOR_WHEEL_DIAMETER_M) / MOTOR_ENCODER_TICKS_PER_REV;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        if (xQueueReceive(g_motion_command_queue, &incoming, 0U) == pdPASS) {
            const float old_left_target = command_wheel_speed(&requested, true);
            const float old_right_target = command_wheel_speed(&requested, false);
            const bool entering_turn = incoming.type == MOTION_TURN &&
                                       requested.type != MOTION_TURN;
            if (requested.type == MOTION_TURN && incoming.type != MOTION_TURN) {
                active_turn_id = 0U;
            }
            requested = incoming;
            const uint32_t command_time_ms = Robot_Millis();
            if (fabsf(old_left_target) <= 0.001f &&
                fabsf(command_wheel_speed(&requested, true)) > 0.001f) {
                left_last_tick_ms = command_time_ms;
            }
            if (fabsf(old_right_target) <= 0.001f &&
                fabsf(command_wheel_speed(&requested, false)) > 0.001f) {
                right_last_tick_ms = command_time_ms;
            }
            if ((fabsf(old_left_target) <= 0.001f &&
                 fabsf(command_wheel_speed(&requested, true)) > 0.001f) ||
                (fabsf(old_right_target) <= 0.001f &&
                 fabsf(command_wheel_speed(&requested, false)) > 0.001f)) {
                encoder_fault_latched = false;
            }
            if (entering_turn) {
                turn_progress_deg = 0.0f;
                active_turn_id = incoming.turn_id;
                completed_turn_id = 0U;
                left_integral = 0.0f;
                right_integral = 0.0f;
                previous_left_error = 0.0f;
                previous_right_error = 0.0f;
            }
            if (requested.type != MOTION_DRIVE && requested.type != MOTION_TURN) {
                left_integral = 0.0f;
                right_integral = 0.0f;
                previous_left_error = 0.0f;
                previous_right_error = 0.0f;
            }
        }

        float target_left = 0.0f;
        float target_right = 0.0f;
#if ROBOT_SIMULATION
        if (requested.type == MOTION_DRIVE) {
            target_left = clamp(requested.left_speed, -ROBOT_MAX_SPEED, ROBOT_MAX_SPEED);
            target_right = clamp(requested.right_speed, -ROBOT_MAX_SPEED, ROBOT_MAX_SPEED);
        } else if (requested.type == MOTION_TURN) {
            const float direction = requested.turn_angle_deg > 0.0f ? 1.0f : -1.0f;
            target_left = -direction * MOTOR_TURN_SPEED_M_S;
            target_right = direction * MOTOR_TURN_SPEED_M_S;
        }
        simulate_encoder_motion(target_left, interval_s, meters_per_tick,
                                &simulated_left_fraction, &left_encoder_ticks);
        simulate_encoder_motion(target_right, interval_s, meters_per_tick,
                                &simulated_right_fraction, &right_encoder_ticks);
#endif

        const uint32_t irq_state = save_and_disable_interrupts();
        const int32_t left_now = left_encoder_ticks;
        const int32_t right_now = right_encoder_ticks;
        restore_interrupts(irq_state);

        const int32_t left_delta = left_now - previous_left;
        const int32_t right_delta = right_now - previous_right;
        previous_left = left_now;
        previous_right = right_now;
        const uint32_t now_ms = Robot_Millis();
        if (left_delta != 0) left_last_tick_ms = now_ms;
        if (right_delta != 0) right_last_tick_ms = now_ms;
        const float measured_left = ((float)left_delta * meters_per_tick) / interval_s;
        const float measured_right = ((float)right_delta * meters_per_tick) / interval_s;
        total_distance += ((float)(left_delta + right_delta) * 0.5f) * meters_per_tick;

        if (requested.type == MOTION_TURN) {
            const float wheel_delta_m = (float)(right_delta - left_delta) * meters_per_tick;
            turn_progress_deg += (wheel_delta_m / MOTOR_TRACK_WIDTH_M) *
                                 (180.0f / MOTION_PI);
            const float turn_direction = requested.turn_angle_deg > 0.0f ? 1.0f : -1.0f;
            const float remaining_deg = fabsf(requested.turn_angle_deg) -
                                        turn_direction * turn_progress_deg;
            if (remaining_deg <= 0.0f) {
                requested.type = MOTION_STOP;
                completed_turn_id = active_turn_id;
                active_turn_id = 0U;
            } else {
                const float ramp = clamp(remaining_deg / MOTOR_TURN_SLOWDOWN_DEG, 0.0f, 1.0f);
                const float turn_speed = MOTOR_TURN_MIN_SPEED_M_S +
                    (MOTOR_TURN_SPEED_M_S - MOTOR_TURN_MIN_SPEED_M_S) * ramp;
                target_left = -turn_direction * turn_speed;
                target_right = turn_direction * turn_speed;
            }
        } else if (requested.type == MOTION_DRIVE) {
            target_left = clamp(requested.left_speed, -ROBOT_MAX_SPEED, ROBOT_MAX_SPEED);
            target_right = clamp(requested.right_speed, -ROBOT_MAX_SPEED, ROBOT_MAX_SPEED);
        }

        if (!encoder_fault_latched &&
            ((fabsf(target_left) > 0.001f &&
              (uint32_t)(now_ms - left_last_tick_ms) >= MOTOR_ENCODER_TIMEOUT_MS) ||
             (fabsf(target_right) > 0.001f &&
              (uint32_t)(now_ms - right_last_tick_ms) >= MOTOR_ENCODER_TIMEOUT_MS))) {
            requested.type = MOTION_BRAKE;
            active_turn_id = 0U;
            completed_turn_id = 0U;
            encoder_fault_latched = true;
            encoder_fault_pending = true;
            left_integral = 0.0f;
            right_integral = 0.0f;
        }

        if (encoder_fault_pending) {
            SystemEvent fault = {
                .type = SYSTEM_EVENT_FAULT,
                .timestamp_ms = now_ms
            };
            if (Robot_PostEvent(&fault)) encoder_fault_pending = false;
        }

        float left_output = 0.0f;
        float right_output = 0.0f;
        if (requested.type == MOTION_DRIVE || requested.type == MOTION_TURN) {
            left_output = speed_pid(target_left, measured_left, interval_s,
                                    &left_integral, &previous_left_error);
            right_output = speed_pid(target_right, measured_right, interval_s,
                                     &right_integral, &previous_right_error);
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
        (void)left_output;
        (void)right_output;
#endif

        SystemEvent event = { .type = SYSTEM_EVENT_MOTION, .timestamp_ms = Robot_Millis() };
        event.data.motion.left_ticks = left_now;
        event.data.motion.right_ticks = right_now;
        event.data.motion.left_speed_m_s = measured_left;
        event.data.motion.right_speed_m_s = measured_right;
        event.data.motion.distance_m = total_distance;
        event.data.motion.turn_complete = completed_turn_id != 0U;
        event.data.motion.turn_id = completed_turn_id;
        (void)Robot_PostEvent(&event);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_MOTION_PERIOD_MS));
    }
}

void Buddy2Motion_CreateTask(void)
{
    BaseType_t result = xTaskCreate(motion_task, "MotionPID", ROBOT_TASK_STACK_WORDS,
                                    NULL, tskIDLE_PRIORITY + 3U, NULL);
    configASSERT(result == pdPASS);
}
