/** @file telemetry_codec.c
 *
 * @brief Minimum-bit binary telemetry encoder.
 */

#include "telemetry_codec.h"

#include <stddef.h>
#include <string.h>

/*
 * Write a value into a binary payload using exactly bit_count bits.
 *
 * Bits are written most-significant-bit first.
 * bit_position is measured from the start of the payload.
 */
static void
write_bits(uint8_t *p_payload,
           uint32_t bit_position,
           uint32_t value,
           uint8_t bit_count)
{
    uint8_t i;

    for (i = 0U; i < bit_count; i++)
    {
        uint32_t const position = bit_position + i;
        uint32_t const byte_index = position / 8U;
        uint8_t const bit_index =
            (uint8_t)(7U - (position % 8U));

        uint8_t const bit =
            (uint8_t)((value >> (bit_count - 1U - i)) & 1U);

        if (bit != 0U)
        {
            p_payload[byte_index] |=
                (uint8_t)(1U << bit_index);
        }
        else
        {
            p_payload[byte_index] &=
                (uint8_t)~(1U << bit_index);
        }
    }
}

/*
 * Encode a telemetry snapshot into 82 meaningful bits.
 *
 * Layout:
 *  0-31  timestamp
 * 32-33  mission state
 * 34-39  base speed
 * 40-47  line error
 * 48-59  distance
 * 60-67  hump height
 * 68-76  obstacle distance
 * 77-78  last barcode
 * 79-81  fault code
 * 82-87  padding (zero)
 */
bool telemetry_encode(telemetry_message_t const *p_telemetry,
                      uint8_t *p_payload,
                      uint32_t payload_size)
{
    uint32_t speed_code;
    uint32_t line_code;
    uint32_t distance_code;
    uint32_t hump_code;
    uint32_t obstacle_code;

    if ((NULL == p_telemetry) ||
        (NULL == p_payload) ||
        (payload_size < TELEMETRY_PACKED_SIZE))
    {
        return false;
    }

    /*
     * Validate the supported ranges before conversion.
     * These are our proposed protocol limits.
     */
    if ((p_telemetry->mission_state > MISSION_FAULT) ||
        !(p_telemetry->speed_m_s >= 0.10f &&
          p_telemetry->speed_m_s <= 0.70f) ||
        !(p_telemetry->line_error >= -1.0f &&
          p_telemetry->line_error <= 1.0f) ||
        !(p_telemetry->distance_m >= -20.0f &&
          p_telemetry->distance_m <= 20.0f) ||
        !(p_telemetry->hump_height_m >= 0.0f &&
          p_telemetry->hump_height_m <= 0.20f) ||
        !(p_telemetry->obstacle_distance_cm >= 2.0f &&
          p_telemetry->obstacle_distance_cm <= 400.0f) ||
        (p_telemetry->last_barcode > 3U) ||
        (p_telemetry->fault_code > 7U))
    {
        return false;
    }

    /* Convert floating-point values to unsigned integers. */
    speed_code =
        (uint32_t)(p_telemetry->speed_m_s * 100.0f + 0.5f) - 10U;

    line_code =
        (uint32_t)((p_telemetry->line_error + 1.0f) * 100.0f + 0.5f);

    distance_code =
        (uint32_t)((p_telemetry->distance_m + 20.0f) * 100.0f + 0.5f);

    hump_code =
        (uint32_t)(p_telemetry->hump_height_m * 1000.0f + 0.5f);

    obstacle_code =
        (uint32_t)(p_telemetry->obstacle_distance_cm - 2.0f + 0.5f);

    /* Clear the output so unused padding bits are zero. */
    memset(p_payload, 0, TELEMETRY_PACKED_SIZE);

    /* Pack fields using the agreed bit positions. */
    write_bits(p_payload, 0U, p_telemetry->timestamp_ms, 32U);
    write_bits(p_payload, 32U, (uint32_t)p_telemetry->mission_state, 2U);
    write_bits(p_payload, 34U, speed_code, 6U);
    write_bits(p_payload, 40U, line_code, 8U);
    write_bits(p_payload, 48U, distance_code, 12U);
    write_bits(p_payload, 60U, hump_code, 8U);
    write_bits(p_payload, 68U, obstacle_code, 9U);
    write_bits(p_payload, 77U, p_telemetry->last_barcode, 2U);
    write_bits(p_payload, 79U, p_telemetry->fault_code, 3U);

    return true;
}
