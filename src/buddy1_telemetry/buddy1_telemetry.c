#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "config.h"
#include "common_types.h"
#include "interfaces/buddy1_telemetry.h"

bool Buddy1Telemetry_SubmitCommand(const ControlCommand *command)
{
    return command != NULL && g_control_command_queue != NULL &&
           xQueueSend(g_control_command_queue, command, 0) == pdPASS;
}

bool Buddy1Telemetry_Publish(const TelemetryMessage *message)
{
    return message != NULL && g_telemetry_queue != NULL &&
           xQueueSend(g_telemetry_queue, message, 0) == pdPASS;
}

static void telemetry_task(void *argument)
{
    (void)argument;
    TelemetryMessage message;
    TickType_t last_heartbeat = xTaskGetTickCount();
    for (;;) {
        if (xQueueReceive(g_telemetry_queue, &message, pdMS_TO_TICKS(100)) == pdPASS) {
            printf("{\"t_ms\":%lu,\"mission\":%u,\"speed\":%.3f,\"line\":%.3f,"
                   "\"distance\":%.3f,\"hump_m\":%.3f,\"obstacle_cm\":%.1f,"
                   "\"barcode\":%lu,\"fault\":%lu}\n",
                   (unsigned long)message.timestamp_ms, (unsigned)message.mission_state,
                   (double)message.speed_m_s, (double)message.line_error,
                   (double)message.distance_m, (double)message.hump_height_m,
                   (double)message.obstacle_distance_cm, (unsigned long)message.last_barcode,
                   (unsigned long)message.fault_code);
        }
        if ((xTaskGetTickCount() - last_heartbeat) >= pdMS_TO_TICKS(ROBOT_TELEMETRY_PERIOD_MS)) {
            printf("{\"heartbeat_ms\":%lu}\n", (unsigned long)Robot_Millis());
            last_heartbeat = xTaskGetTickCount();
        }
    }
}

void Buddy1Telemetry_CreateTask(void)
{
    BaseType_t result = xTaskCreate(telemetry_task, "Telemetry", ROBOT_TASK_STACK_WORDS,
                                    NULL, tskIDLE_PRIORITY + 1U, NULL);
    configASSERT(result == pdPASS);
}