/** @file robot_types.h
 *
 * @brief Message formats exchanged between modules.  Port of
 *        include/common_types.h from the FreeRTOS project.
 *
 * All of these travel BY VALUE through micro T-Kernel message buffers
 * (tk_snd_mbf copies the bytes), exactly like the FreeRTOS queues did.
 * The kernel object IDs live in the header of the module that owns each
 * object (controller.h, subsystem1.h, subsystem2.h), not here.
 */

#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MISSION_IDLE = 0,
    MISSION_FOLLOWING,
    MISSION_AVOIDING,
    MISSION_FAULT
} mission_state_t;

typedef enum
{
    SYSTEM_EVENT_LINE = 0,
    SYSTEM_EVENT_MOTION,
    SYSTEM_EVENT_IMU,
    SYSTEM_EVENT_OBSTACLE,
    SYSTEM_EVENT_FAULT
} system_event_type_t;

typedef enum
{
    CONTROL_START = 0,
    CONTROL_STOP,
    CONTROL_SET_SPEED,
    CONTROL_CLEAR_FAULT
} control_command_type_t;

typedef enum
{
    MOTION_STOP = 0,
    MOTION_DRIVE,
    MOTION_TURN,
    MOTION_BRAKE
} motion_command_type_t;

#define BARCODE_TEXT_MAX    (8U)

typedef enum
{
    BARCODE_CMD_NONE  = 0,      /* Decoded, no turn: continue straight. */
    BARCODE_CMD_LEFT  = 1,
    BARCODE_CMD_RIGHT = 2,
    BARCODE_CMD_UTURN = 3
} barcode_command_t;

typedef struct
{
    float       line_error;
    bool        b_line_detected;
    bool        b_junction;
    bool        b_barcode_valid;
    uint16_t    barcode;        /* barcode_command_t when b_barcode_valid */
    char        barcode_text[BARCODE_TEXT_MAX + 1U];
} line_event_t;

typedef struct
{
    int32_t     left_ticks;
    int32_t     right_ticks;
    float       left_speed_m_s;
    float       right_speed_m_s;
    float       distance_m;
    bool        turn_complete;
    uint32_t    turn_id;
} motion_event_t;

typedef enum
{
    MOTION_STATIONARY   = 0,    /* |a| < 0.5 g and |gz| < 10 dps      */
    MOTION_ACCELERATING = 1,    /* |a| > 1.5 g                        */
    MOTION_TURNING      = 2,    /* |gz| > 20 dps                      */
    MOTION_CLIMBING     = 3,    /* az > 1.3 g                         */
    MOTION_DESCENDING   = 4,    /* az < 0.7 g                         */
    MOTION_IMPACT       = 5     /* |a| > 3.0 g                        */
} motion_class_t;

typedef struct
{
    float           acceleration_x_g;
    float           acceleration_y_g;
    float           acceleration_z_g;
    float           turn_rate_dps;
    float           estimated_hump_height_m;
    bool            b_shock_detected;
    bool            b_sensor_valid;
    motion_class_t  motion_state;
    float           tilt_roll_deg;
    float           tilt_pitch_deg;
} imu_event_t;

typedef struct
{
    bool        b_detected;
    bool        b_scan_complete;
    float       center_distance_cm;
    float       estimated_width_cm;
    float       left_clearance_cm;
    float       right_clearance_cm;
    int8_t      bypass_direction;   /* -1 left, +1 right, 0 none */
} obstacle_event_t;

typedef struct
{
    system_event_type_t type;
    uint32_t            timestamp_ms;
    union
    {
        line_event_t        line;
        motion_event_t      motion;
        imu_event_t         imu;
        obstacle_event_t    obstacle;
    } data;
} system_event_t;

typedef struct
{
    control_command_type_t  type;
    float                   value;
} control_command_t;

typedef struct
{
    motion_command_type_t   type;
    float                   left_speed;
    float                   right_speed;
    float                   turn_angle_deg;
    uint32_t                turn_id;
} motion_command_t;

typedef struct
{
    uint32_t        timestamp_ms;
    mission_state_t mission_state;
    float           speed_m_s;
    float           line_error;
    float           distance_m;
    float           hump_height_m;
    float           obstacle_distance_cm;
    uint32_t        last_barcode;
    uint32_t        fault_code;
} telemetry_message_t;

/* System status bits (event flag owned by the controller, IF-05). */
#define SYSTEM_BIT_READY        (1UL << 0)
#define SYSTEM_BIT_MISSION_RUN  (1UL << 1)
#define SYSTEM_BIT_OBSTACLE     (1UL << 2)
#define SYSTEM_BIT_FAULT        (1UL << 3)
#define SYSTEM_BIT_WIFI_UP      (1UL << 4)

#endif /* ROBOT_TYPES_H */

/*** end of file ***/
