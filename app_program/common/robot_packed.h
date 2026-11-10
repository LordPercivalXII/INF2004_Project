/** @file robot_packed.h
 *
 * @brief Minimum-bit wire formats for the inter-task message buffers
 *        (Tribunal 1 curveball "Minimum-Bit Data Representation").
 *
 * This is the port's single home for every packed struct, as the
 * tribunal decided for include/common_types.h (robot_types.h here).
 * Each sender packs immediately before tk_snd_mbf() and each receiver
 * unpacks immediately after tk_rcv_mbf(); all CONTROL math (PID, line
 * steering, height fusion, avoidance) keeps its full-precision types.
 *
 * Internal queues use C bit-fields with ROBOT_PACKED and a compile-time
 * size check.  Both ends run on the same RP2040 build, so the compiler's
 * bit order is the same on both sides.  The MQTT payload leaves the
 * board, so it is serialised with explicit shifts and masks instead
 * (subsystem1/telemetry_codec.c, Version C gap 2).
 *
 * BARR-C: bit-fields are unsigned only, and no bitwise operator is used
 * on signed data.  Signed quantities are therefore stored offset-binary
 * (value + bias) in unsigned fields.
 *
 * Bit budget (tribunal size -> size implemented, and why it differs):
 *
 *   queue            tribunal  here  unpacked  note
 *   ---------------  --------  ----  --------  ---------------------------
 *   cmd_q_motion     2 B       4 B   24 B      needs left AND right speed
 *                                              for steering, and 5 types
 *   nav_q_line       2 B       2 B   } 28 B    as decided (error:12, ...)
 *   cmd_q_barcode    1 B       1 B   }         as decided (command:3, ...)
 *   + barcode char   -         1 B   }         ASCII for telemetry
 *   evt_q_terrain    1 B       1 B    8 B      as decided (shock:1, h:7)
 *   nav_q_obstacle   2 B       9 B   56 B      6-bit cm caps at 63 cm but
 *                                              the sensor reads 250 cm;
 *                                              needs signed offset and
 *                                              scan time
 *   MQTT payload     4 B       4 B   ~800 B    as decided (JSON text)
 *
 * USE_BIT_PACKING (make PACK=0|1, default 1) builds the unpacked
 * structs instead, for the Week 13 before/after RAM and size comparison.
 */

#ifndef ROBOT_PACKED_H
#define ROBOT_PACKED_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_types.h"

#ifndef USE_BIT_PACKING
#define USE_BIT_PACKING         (1)
#endif

/* GCC: no padding inside a wire struct. */
#define ROBOT_PACKED            __attribute__((packed))

/* 3-bit vehicle state (mission_state_t fits with room to grow). */
#define ROBOT_STATE_BITS        (3U)
#define ROBOT_STATE_MASK        (0x07U)

/* 12-bit offset-binary fields: stored = value + 2048, 0..4095. */
#define PACK_Q12_BIAS           (2048)
#define PACK_Q12_MAX            (4095)

/* 9-bit offset-binary fields: stored = value + 256, 0..511. */
#define PACK_Q9_BIAS            (256)
#define PACK_Q9_MAX             (511)
#define PACK_U9_MAX             (511U)

/* Scales. */
#define PACK_LINE_SCALE         (2047.0f)   /* line error +-1 -> +-2047   */
#define PACK_SPEED_SCALE        (1000.0f)   /* m/s -> mm/s                */
#define PACK_ANGLE_SCALE        (4.0f)      /* deg -> quarter degrees      */
#define PACK_DISTANCE_SCALE     (1000.0f)   /* m -> mm                     */

/* Turn IDs travel in 5 bits: the controller counts 1..31, then wraps. */
#define MOTION_TURN_ID_MAX      (31U)

/* Terrain height travels in 7 bits: 0..127 mm, saturating. */
#define TERRAIN_HEIGHT_MAX_MM   (127U)

/* Obstacle scan duration travels in 14 bits: 0..16383 ms, saturating. */
#define OBSTACLE_SCAN_MS_MAX    (16383U)

/* ------------------------------------------------------------------ */
/* IF-03 cmd_q_motion: controller -> S2, 4 bytes                       */
/* ------------------------------------------------------------------ */
/*
 *   mode     motion_command_type_t
 *   turn_id  MOTION_TURN only, 1..MOTION_TURN_ID_MAX
 *   arg_a    DRIVE: left mm/s   TURN: angle 1/4 deg   MOVE: speed mm/s
 *   arg_b    DRIVE: right mm/s  TURN: 0               MOVE: distance mm
 *   (speeds and angle offset-binary +2048; distance unsigned)
 */
typedef struct ROBOT_PACKED
{
    unsigned int    mode    : 3;
    unsigned int    turn_id : 5;
    unsigned int    arg_a   : 12;
    unsigned int    arg_b   : 12;
} motion_cmd_packed_t;

/* ------------------------------------------------------------------ */
/* IF-01 nav_q_line + cmd_q_barcode: S3 -> controller, 4 bytes         */
/* ------------------------------------------------------------------ */
typedef struct ROBOT_PACKED
{
    unsigned int    error_q12   : 12;   /* line error, offset-binary   */
    unsigned int    b_junction  : 1;
    unsigned int    b_fault     : 1;    /* line lost (not detected)    */
    unsigned int    b_scanning  : 1;    /* barcode scan in progress     */
    unsigned int    b_barcode   : 1;    /* new character in barcode_char */
} nav_line_packed_t;                    /* 2 bytes */

typedef struct ROBOT_PACKED
{
    unsigned int    command     : 3;    /* barcode_command_t at junction */
    unsigned int    b_char      : 1;    /* junction used a stored char   */
    unsigned int    b_reversed  : 1;    /* barcode read right-to-left    */
    unsigned int    reserved    : 3;
} barcode_cmd_packed_t;                 /* 1 byte */

typedef struct ROBOT_PACKED
{
    nav_line_packed_t       nav;
    barcode_cmd_packed_t    cmd;
    uint8_t                 barcode_char;   /* 7-bit ASCII, 0 = none */
} line_packed_t;                            /* 4 bytes */

/* ------------------------------------------------------------------ */
/* evt_q_terrain: S4 -> controller (polled first), 1 byte              */
/* ------------------------------------------------------------------ */
typedef struct ROBOT_PACKED
{
    unsigned int    b_shock     : 1;
    unsigned int    height_mm   : 7;    /* completed hump, saturating */
} terrain_packed_t;

/* Unpacked form, used when USE_BIT_PACKING is 0. */
typedef struct
{
    bool    b_shock;
    float   height_m;
} terrain_event_t;

/* ------------------------------------------------------------------ */
/* IF-01 nav_q_obstacle: S5 -> controller, 9 bytes                     */
/* ------------------------------------------------------------------ */
/*
 *   bypass_dir  0 none, 1 left, 2 right
 *   angle_idx   0 left, 1 ahead, 2 right (where the obstacle centre is)
 *   distance_cm centre distance, or side distance if b_side
 *   offset_q9   bypass offset cm, offset-binary +256 (+-255 cm)
 */
typedef struct ROBOT_PACKED
{
    unsigned int    action      : 3;    /* obstacle_action_t */
    unsigned int    b_detected  : 1;
    unsigned int    b_scan_done : 1;
    unsigned int    b_side      : 1;
    unsigned int    bypass_dir  : 2;
    unsigned int    angle_idx   : 2;
    unsigned int    distance_cm : 9;
    unsigned int    closest_cm  : 9;
    unsigned int    width_cm    : 9;
    unsigned int    clear_cm    : 9;    /* clearance on the bypass side */
    unsigned int    offset_q9   : 9;
    unsigned int    scan_ms     : 14;   /* event time - scan start      */
    unsigned int    reserved    : 3;
} obstacle_packed_t;

/* Compile-time size checks: a layout change must be deliberate. */
_Static_assert(sizeof(motion_cmd_packed_t) == 4U, "cmd_q_motion size");
_Static_assert(sizeof(nav_line_packed_t) == 2U, "nav_q_line size");
_Static_assert(sizeof(barcode_cmd_packed_t) == 1U, "cmd_q_barcode size");
_Static_assert(sizeof(line_packed_t) == 4U, "line message size");
_Static_assert(sizeof(terrain_packed_t) == 1U, "evt_q_terrain size");
_Static_assert(sizeof(obstacle_packed_t) == 9U, "nav_q_obstacle size");

/* What actually travels in each queue for this build. */
#if USE_BIT_PACKING
typedef motion_cmd_packed_t     motion_cmd_wire_t;
typedef terrain_packed_t        terrain_wire_t;
#else
typedef motion_command_t        motion_cmd_wire_t;
typedef terrain_event_t         terrain_wire_t;
#endif

/* Scalar helpers (saturating, round to nearest). */
uint32_t pack_offset(float value, float scale, int32_t bias, int32_t max);
float    unpack_offset(uint32_t code, float scale, int32_t bias);
uint32_t pack_unsigned(float value, float scale, uint32_t max);

/* Codecs.  pack_* return false if the value cannot be represented. */
bool pack_motion_cmd(motion_command_t const * p_cmd,
                     motion_cmd_packed_t * p_wire);
void unpack_motion_cmd(motion_cmd_packed_t const * p_wire,
                       motion_command_t * p_cmd);

void pack_line(line_event_t const * p_line, line_packed_t * p_wire);
void unpack_line(line_packed_t const * p_wire, line_event_t * p_line);

void pack_obstacle(obstacle_event_t const * p_obs, uint32_t event_ms,
                   obstacle_packed_t * p_wire);
void unpack_obstacle(obstacle_packed_t const * p_wire, uint32_t event_ms,
                     obstacle_event_t * p_obs);

void pack_terrain(bool b_shock, float height_m, terrain_packed_t * p_wire);
void unpack_terrain(terrain_packed_t const * p_wire,
                    terrain_event_t * p_terrain);

#endif /* ROBOT_PACKED_H */

/*** end of file ***/
