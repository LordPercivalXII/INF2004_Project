#ifndef ROBOT_COMMON_TYPES_H
#define ROBOT_COMMON_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"

typedef enum {
    MISSION_IDLE = 0,
    MISSION_FOLLOWING,
    MISSION_AVOIDING,
    MISSION_FAULT
} MissionState;

typedef enum {
    SYSTEM_EVENT_LINE = 0,
    SYSTEM_EVENT_MOTION,
    SYSTEM_EVENT_IMU,
    SYSTEM_EVENT_OBSTACLE,
    SYSTEM_EVENT_FAULT
} SystemEventType;

typedef enum {
    CONTROL_START = 0,
    CONTROL_STOP,
    CONTROL_SET_SPEED,
    CONTROL_CLEAR_FAULT
} ControlCommandType;

typedef enum {
    MOTION_STOP = 0,
    MOTION_DRIVE,
    MOTION_TURN,
    MOTION_BRAKE
} MotionCommandType;

#define BARCODE_TEXT_MAX 8U

typedef enum {
    BARCODE_CMD_NONE = 0,     /* decoded, no turn: continue straight */
    BARCODE_CMD_LEFT = 1,
    BARCODE_CMD_RIGHT = 2,
    BARCODE_CMD_UTURN = 3
} BarcodeCommand;

typedef struct {
    float line_error;
    bool line_detected;
    bool junction;
    bool barcode_valid;
    uint16_t barcode;                          /* BarcodeCommand when barcode_valid */
    char barcode_text[BARCODE_TEXT_MAX + 1U];  /* decoded Code 39 payload, '\0'-terminated */
} LineEvent;

typedef struct {
    int32_t left_ticks;
    int32_t right_ticks;
    float left_speed_m_s;
    float right_speed_m_s;
    float distance_m;
    bool turn_complete;
    uint32_t turn_id;
} MotionEvent;

typedef enum {
    MOTION_STATIONARY = 0,      /* |a_total| < 0.5g and |gz| < 10 DPS */
    MOTION_ACCELERATING = 1,    /* |a_total| > 1.5g and consistent direction */
    MOTION_TURNING = 2,         /* |gz| > 20 DPS (high yaw rate) */
    MOTION_CLIMBING = 3,        /* az > 1.3g for sustained duration */
    MOTION_DESCENDING = 4,      /* az < 0.7g for sustained duration */
    MOTION_IMPACT = 5           /* |a_total| > 3.0g (sudden shock) */
} MotionClassification;

typedef struct {
    float acceleration_x_g;
    float acceleration_y_g;
    float acceleration_z_g;
    float turn_rate_dps;
    float estimated_hump_height_m;
    bool shock_detected;
    bool sensor_valid;
    MotionClassification motion_state;  /* Current motion classification */
    float tilt_roll_deg;                /* Roll angle in degrees */
    float tilt_pitch_deg;               /* Pitch angle in degrees */
} ImuEvent;

typedef struct {
    bool detected;
    bool scan_complete;
    float center_distance_cm;
    float estimated_width_cm;
    float left_clearance_cm;
    float right_clearance_cm;
    int8_t bypass_direction;
} ObstacleEvent;

typedef struct {
    SystemEventType type;
    uint32_t timestamp_ms;
    union {
        LineEvent line;
        MotionEvent motion;
        ImuEvent imu;
        ObstacleEvent obstacle;
    } data;
} SystemEvent;

typedef struct {
    ControlCommandType type;
    float value;
} ControlCommand;

typedef struct {
    MotionCommandType type;
    float left_speed;
    float right_speed;
    float turn_angle_deg;
    uint32_t turn_id;
} MotionCommand;

typedef struct {
    uint32_t timestamp_ms;
    MissionState mission_state;
    float speed_m_s;
    float line_error;
    float distance_m;
    float hump_height_m;
    float obstacle_distance_cm;
    uint32_t last_barcode;
    uint32_t fault_code;
} TelemetryMessage;

enum {
    SYSTEM_BIT_READY       = (1U << 0),
    SYSTEM_BIT_MISSION_RUN = (1U << 1),
    SYSTEM_BIT_OBSTACLE    = (1U << 2),
    SYSTEM_BIT_FAULT       = (1U << 3),
    SYSTEM_BIT_WIFI_UP     = (1U << 4)
};

extern QueueHandle_t g_system_event_queue;
extern QueueHandle_t g_control_command_queue;
extern QueueHandle_t g_motion_command_queue;
extern QueueHandle_t g_telemetry_queue;
extern EventGroupHandle_t g_system_event_group;

bool Robot_PostEvent(const SystemEvent *event);
uint32_t Robot_Millis(void);

#endif