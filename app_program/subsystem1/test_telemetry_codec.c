/** @file test_telemetry_codec.c
 *
 * @brief PC test for the 4-byte binary MQTT packet (not in the firmware;
 *        the makefile filters this file out).
 *
 *   gcc -std=c99 -Wall -Wextra test_telemetry_codec.c telemetry_codec.c \
 *       ../common/robot_packed.c -o test_codec && ./test_codec
 */

#include <assert.h>
#include <stdio.h>

#include "telemetry_codec.h"

int
main (void)
{
    telemetry_message_t telemetry = {0};
    uint8_t             payload[TELEMETRY_PACKED_SIZE] = {0};

    telemetry.mission_state        = MISSION_AVOIDING;     /* 2      */
    telemetry.fault_mask           = 0x09U;                 /* IMU+NO_ECHO */
    telemetry.speed_m_s            = 0.35f;                 /* 35 cm/s */
    telemetry.line_error           = -0.5f;
    telemetry.hump_height_m        = 0.042f;                /* 42 mm  */
    telemetry.obstacle_distance_cm = 61.0f;                 /* 31 x 2 cm */

    assert(telemetry_encode(&telemetry, false, payload, sizeof(payload)));
    assert(payload[0] == (uint8_t) (2U | (0x09U << 3)));
    assert(35U == payload[1]);
    /* -0.5 * 2047 = -1024 (rounded) -> 1024 -> >> 4 = 64 */
    assert(64U == payload[2]);
    assert(42U == payload[3]);

    assert(telemetry_encode(&telemetry, true, payload, sizeof(payload)));
    assert((0x80U | 31U) == payload[3]);

    telemetry.line_error = 1.0f;        /* 4095 >> 4 = 255 */
    telemetry.speed_m_s  = 9.0f;        /* saturates      */
    assert(telemetry_encode(&telemetry, true, payload, sizeof(payload)));
    assert((255U == payload[2]) && (255U == payload[1]));
    telemetry.line_error = 0.0f;        /* 2048 >> 4 = 128 */
    assert(telemetry_encode(&telemetry, true, payload, sizeof(payload)));
    assert(128U == payload[2]);

    assert(!telemetry_encode(&telemetry, true, payload, 3U));

    printf("test_telemetry_codec: all tests passed\n");

    return (0);
}

/*** end of file ***/
