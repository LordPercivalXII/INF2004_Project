/** @file telemetry_codec.c
 *
 * @brief 5-byte binary MQTT telemetry encoder (layout in the header).
 *        Pure C, no kernel calls: also built by test_telemetry_codec.c.
 */

#include <stddef.h>

#include "telemetry_codec.h"
#include "../common/robot_packed.h"

#define TELEMETRY_SPEED_MAX_CM_S    (255U)
#define TELEMETRY_SPEED_SCALE       (100.0f)    /* m/s -> cm/s  */
#define TELEMETRY_HUMP_SCALE        (1000.0f)   /* m -> mm      */

/*!
 * @brief Encode a telemetry snapshot into the 5-byte MQTT payload.
 *
 * @param[in]  p_telemetry      Snapshot from the controller.
 * @param[in]  b_obstacle_slot  true: byte 3 carries the obstacle distance;
 *                              false: the hump height.
 * @param[out] p_payload        TELEMETRY_PACKED_SIZE bytes.
 * @param[in]  payload_size     Capacity of p_payload.
 *
 * @return false if a pointer is NULL or the buffer is too small.
 */
bool
telemetry_encode (telemetry_message_t const * p_telemetry,
                  bool b_obstacle_slot,
                  uint8_t * p_payload, uint32_t payload_size)
{
    uint32_t state;
    uint32_t line_q12;
    uint32_t value;

    if ((NULL == p_telemetry) || (NULL == p_payload) ||
        (payload_size < TELEMETRY_PACKED_SIZE))
    {
        return (false);
    }

    state = (uint32_t) p_telemetry->mission_state;
    if (state > (uint32_t) MISSION_FAULT)
    {
        state = (uint32_t) MISSION_FAULT;
    }
    else if ((MISSION_IDLE == p_telemetry->mission_state) &&
             p_telemetry->b_start_pending)
    {
        /* Tell the dashboard START arrived and why the car is not moving. */
        state = p_telemetry->b_imu_ready ? TELEMETRY_STATE_STARTING
                                         : TELEMETRY_STATE_WAIT_IMU;
    }
    else
    {
        /* A plain mission state. */
    }

    /* Same 12-bit offset-binary code as nav_q_line; the unsigned shift
       is the BARR-C form of the tribunal's (int8_t)(err_12b >> 4). */
    line_q12 = pack_offset(p_telemetry->line_error, PACK_LINE_SCALE,
                           PACK_Q12_BIAS, PACK_Q12_MAX);

    if (b_obstacle_slot)
    {
        value = pack_unsigned(p_telemetry->obstacle_distance_cm,
                              1.0f / TELEMETRY_OBSTACLE_CM_LSB,
                              TELEMETRY_VALUE_MASK) |
                TELEMETRY_KIND_OBSTACLE;
    }
    else
    {
        value = pack_unsigned(p_telemetry->hump_height_m,
                              TELEMETRY_HUMP_SCALE, TELEMETRY_VALUE_MASK);
    }

    p_payload[0] = (uint8_t) ((state & TELEMETRY_STATE_MASK) |
                              (((uint32_t) p_telemetry->fault_mask &
                                TELEMETRY_FAULT_MASK) <<
                               TELEMETRY_FAULT_SHIFT));
    p_payload[1] = (uint8_t) pack_unsigned(p_telemetry->speed_m_s,
                                           TELEMETRY_SPEED_SCALE,
                                           TELEMETRY_SPEED_MAX_CM_S);
    p_payload[2] = (uint8_t) (line_q12 >> TELEMETRY_LINE_SHIFT);
    p_payload[3] = (uint8_t) value;
    p_payload[4] = ((p_telemetry->last_barcode >= TELEMETRY_ASCII_FIRST) &&
                    (p_telemetry->last_barcode <= TELEMETRY_ASCII_LAST))
                   ? (uint8_t) p_telemetry->last_barcode : 0U;

    return (true);
}

/*** end of file ***/
