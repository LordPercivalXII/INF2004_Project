/** @file test_telemetry_codec.c
 *
 * @brief Unit test for minimum-bit telemetry encoding.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../app_program/subsystem1/telemetry_codec.h"

int main(void)
{
    telemetry_message_t telemetry = {0};

    uint8_t payload[TELEMETRY_PACKED_SIZE] = {0};

    uint8_t const expected[TELEMETRY_PACKED_SIZE] = {
        0x00, 0x00, 0x13, 0x88,
        0x59, 0x64, 0x8C, 0xA3,
        0x23, 0x12, 0x00
    };

    telemetry.timestamp_ms = 5000U;
    telemetry.mission_state = MISSION_FOLLOWING;
    telemetry.speed_m_s = 0.35f;
    telemetry.line_error = 0.0f;
    telemetry.distance_m = 2.50f;
    telemetry.hump_height_m = 0.05f;
    telemetry.obstacle_distance_cm = 100.0f;
    telemetry.last_barcode = 1U;
    telemetry.fault_code = 0U;

    assert(telemetry_encode(
        &telemetry,
        payload,
        sizeof(payload)
    ));

    assert(memcmp(payload, expected, sizeof(expected)) == 0);

    printf("PASS: Telemetry encoded correctly!\n");

    return 0;
}
