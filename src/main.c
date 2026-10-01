#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "common_types.h"
#include "config.h"
#include "interfaces/controller.h"
#include "interfaces/buddy1_telemetry.h"
#include "interfaces/buddy2_motion.h"
#include "interfaces/buddy3_line_barcode.h"
#include "interfaces/buddy4_imu.h"
#include "interfaces/buddy5_obstacle.h"

QueueHandle_t g_system_event_queue;
QueueHandle_t g_control_command_queue;
QueueHandle_t g_motion_command_queue;
QueueHandle_t g_telemetry_queue;
EventGroupHandle_t g_system_event_group;

bool Robot_PostEvent(const SystemEvent *event)
{
    return event != NULL && g_system_event_queue != NULL &&
           xQueueSend(g_system_event_queue, event, 0) == pdPASS;
}

uint32_t Robot_Millis(void)
{
    return to_ms_since_boot(get_absolute_time());
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

int main(void)
{
    stdio_init_all();
    sleep_ms(1500);
    printf("INF2004 autonomous car boot (simulation=%d)\n", ROBOT_SIMULATION);

    g_system_event_queue = xQueueCreate(ROBOT_EVENT_QUEUE_LENGTH, sizeof(SystemEvent));
    g_control_command_queue = xQueueCreate(ROBOT_QUEUE_LENGTH, sizeof(ControlCommand));
    g_motion_command_queue = xQueueCreate(ROBOT_QUEUE_LENGTH, sizeof(MotionCommand));
    g_telemetry_queue = xQueueCreate(ROBOT_QUEUE_LENGTH, sizeof(TelemetryMessage));
    g_system_event_group = xEventGroupCreate();

    configASSERT(g_system_event_queue != NULL);
    configASSERT(g_control_command_queue != NULL);
    configASSERT(g_motion_command_queue != NULL);
    configASSERT(g_telemetry_queue != NULL);
    configASSERT(g_system_event_group != NULL);

    Buddy1Telemetry_CreateTask();
    Buddy2Motion_CreateTask();
    Buddy3LineBarcode_CreateTask();
    Buddy4Imu_CreateTask();
    Buddy5Obstacle_CreateTask();
    Controller_CreateTask();

    vTaskStartScheduler();
    printf("FreeRTOS scheduler returned; insufficient heap or port failure.\n");
    for (;;) {}
}