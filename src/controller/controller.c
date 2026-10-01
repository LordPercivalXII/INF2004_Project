#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "config.h"
#include "common_types.h"
#include "interfaces/controller.h"
#include "interfaces/buddy1_telemetry.h"
#include "interfaces/buddy2_motion.h"

static void set_motion(float left, float right)
{
    MotionCommand command = { .type = MOTION_DRIVE, .left_speed = left, .right_speed = right };
    (void)Buddy2Motion_SendCommand(&command);
}

static void stop_motion(void)
{
    MotionCommand command = { .type = MOTION_STOP, .left_speed = 0.0f, .right_speed = 0.0f };
    (void)Buddy2Motion_SendCommand(&command);
}

static void controller_task(void *argument)
{
    (void)argument;
    MissionState state = MISSION_IDLE;
    float base_speed = ROBOT_BASE_SPEED;
    float current_line_error = 0.0f;
    float distance_m = 0.0f;
    float hump_m = 0.0f;
    float obstacle_cm = 0.0f;
    uint32_t barcode = 0U;
    uint32_t fault_code = 0U;
    SystemEvent event;
    ControlCommand user_command;
    TelemetryMessage telemetry;
    TickType_t last_telemetry = xTaskGetTickCount();

    xEventGroupSetBits(g_system_event_group, SYSTEM_BIT_READY);
    for (;;) {
        if (xQueueReceive(g_control_command_queue, &user_command, 0) == pdPASS) {
            switch (user_command.type) {
            case CONTROL_START:
                if (state != MISSION_FAULT) {
                    state = MISSION_FOLLOWING;
                    xEventGroupSetBits(g_system_event_group, SYSTEM_BIT_MISSION_RUN);
                }
                break;
            case CONTROL_STOP:
                state = MISSION_IDLE;
                xEventGroupClearBits(g_system_event_group, SYSTEM_BIT_MISSION_RUN);
                stop_motion();
                break;
            case CONTROL_SET_SPEED:
                base_speed = fminf(ROBOT_MAX_SPEED, fmaxf(0.10f, user_command.value));
                break;
            case CONTROL_CLEAR_FAULT:
                fault_code = 0U;
                state = MISSION_IDLE;
                xEventGroupClearBits(g_system_event_group, SYSTEM_BIT_FAULT);
                break;
            default:
                break;
            }
        }

        if (xQueueReceive(g_system_event_queue, &event, pdMS_TO_TICKS(5)) == pdPASS) {
            switch (event.type) {
            case SYSTEM_EVENT_LINE:
                current_line_error = event.data.line.line_error;
                if (event.data.line.barcode_valid) {
                    barcode = event.data.line.barcode;
                    /* Waypoint codes 1 and 2 request left/right; other codes continue straight. */
                    if (state == MISSION_FOLLOWING && barcode == 1U) {
                        set_motion(base_speed * 0.55f, base_speed);
                    } else if (state == MISSION_FOLLOWING && barcode == 2U) {
                        set_motion(base_speed, base_speed * 0.55f);
                    }
                }
                if (state == MISSION_FOLLOWING) {
                    const float correction = fminf(0.25f, fmaxf(-0.25f, current_line_error * 0.18f));
                    set_motion(base_speed + correction, base_speed - correction);
                }
                break;
            case SYSTEM_EVENT_MOTION:
                distance_m = event.data.motion.distance_m;
                break;
            case SYSTEM_EVENT_IMU:
                if (event.data.imu.sensor_valid) {
                    hump_m = event.data.imu.estimated_hump_height_m;
                    if (event.data.imu.shock_detected) {
                        base_speed = fminf(base_speed, ROBOT_BASE_SPEED * 0.7f);
                    }
                } else {
                    fault_code = 2U;
                }
                break;
            case SYSTEM_EVENT_OBSTACLE:
                obstacle_cm = event.data.obstacle.center_distance_cm;
                if (event.data.obstacle.detected && event.data.obstacle.scan_complete && state == MISSION_FOLLOWING) {
                    state = MISSION_AVOIDING;
                    xEventGroupSetBits(g_system_event_group, SYSTEM_BIT_OBSTACLE);
                    if (event.data.obstacle.bypass_direction < 0) {
                        set_motion(-base_speed * 0.45f, base_speed * 0.45f);
                    } else {
                        set_motion(base_speed * 0.45f, -base_speed * 0.45f);
                    }
                } else if (!event.data.obstacle.detected && state == MISSION_AVOIDING) {
                    xEventGroupClearBits(g_system_event_group, SYSTEM_BIT_OBSTACLE);
                    state = MISSION_FOLLOWING;
                }
                break;
            case SYSTEM_EVENT_FAULT:
                fault_code = 1U;
                state = MISSION_FAULT;
                xEventGroupSetBits(g_system_event_group, SYSTEM_BIT_FAULT);
                stop_motion();
                break;
            default:
                break;
            }
        }

        if ((xTaskGetTickCount() - last_telemetry) >= pdMS_TO_TICKS(ROBOT_TELEMETRY_PERIOD_MS)) {
            telemetry.timestamp_ms = Robot_Millis();
            telemetry.mission_state = state;
            telemetry.speed_m_s = base_speed;
            telemetry.line_error = current_line_error;
            telemetry.distance_m = distance_m;
            telemetry.hump_height_m = hump_m;
            telemetry.obstacle_distance_cm = obstacle_cm;
            telemetry.last_barcode = barcode;
            telemetry.fault_code = fault_code;
            (void)Buddy1Telemetry_Publish(&telemetry);
            last_telemetry = xTaskGetTickCount();
        }
    }
}

void Controller_CreateTask(void)
{
    BaseType_t result = xTaskCreate(controller_task, "VehicleCtrl", ROBOT_TASK_STACK_WORDS,
                                    NULL, tskIDLE_PRIORITY + 3U, NULL);
    configASSERT(result == pdPASS);
}