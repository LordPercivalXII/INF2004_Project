/** @file telemetry_codec.h
 *
 * @brief 5-byte binary MQTT telemetry packet (Tribunal 1 curveball;
 *        replaces the 256+ byte JSON text payload).  The tribunal's
 *        4 bytes plus one byte for the decoded barcode character.
 *
 * Serialised with explicit shifts and masks, never C bit-fields, so the
 * byte and bit layout is identical on the RP2040 and on the laptop that
 * decodes it (Version C gap 2).  Bit 0 is the least significant bit.
 *
 *   byte bits  field         encoding
 *   ---- ----  ------------  ---------------------------------------------
 *   0    0-2   state         0 idle, 1 following, 2 avoiding, 3 fault,
 *                            4 START countdown running,
 *                            5 START held: IMU not calibrated yet
 *   0    3-7   faults        5-bit fault mask (common/fault_mask.h)
 *   1    0-7   speed         commanded speed, cm/s, 0..255 saturating
 *   2    0-7   line error    offset-binary, 128 = centre: the 12-bit
 *                            nav_q_line code shifted right 4 (Version C
 *                            gap 3); error = (byte - 128) / 128
 *   3    0-6   value         hump: mm 0..127; obstacle: cm / 2, 0..254 cm
 *   3    7     kind          0 = hump height, 1 = obstacle distance
 *   4    0-7   barcode       last decoded character, ASCII 0x21..0x7E;
 *                            0 = none since START
 *
 * The bump/obstacle byte alternates between the two kinds on successive
 * packets; the dashboard keeps the last value of each.
 */

#ifndef TELEMETRY_CODEC_H
#define TELEMETRY_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_types.h"

#define TELEMETRY_PACKED_SIZE       (5U)

/* Byte 0 state values beyond mission_state_t (only on the wire). */
#define TELEMETRY_STATE_STARTING    (4U)
#define TELEMETRY_STATE_WAIT_IMU    (5U)

#define TELEMETRY_ASCII_FIRST       (0x21U)     /* '!' */
#define TELEMETRY_ASCII_LAST        (0x7EU)     /* '~' */

#define TELEMETRY_STATE_MASK        (0x07U)
#define TELEMETRY_FAULT_MASK        (0x1FU)
#define TELEMETRY_FAULT_SHIFT       (3U)
#define TELEMETRY_LINE_SHIFT        (4U)    /* 12-bit -> 8-bit */
#define TELEMETRY_VALUE_MASK        (0x7FU)
#define TELEMETRY_KIND_OBSTACLE     (0x80U)
#define TELEMETRY_OBSTACLE_CM_LSB   (2.0f)

bool telemetry_encode(telemetry_message_t const * p_telemetry,
                      bool b_obstacle_slot,
                      uint8_t * p_payload, uint32_t payload_size);

#endif /* TELEMETRY_CODEC_H */

/*** end of file ***/
