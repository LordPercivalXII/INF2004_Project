#ifndef TELEMETRY_CODEC_H
#define TELEMETRY_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#include "../common/robot_types.h"

/* 82 meaningful bits, stored in 11 bytes. */
#define TELEMETRY_PACKED_SIZE  (11U)

/*
 * Encode a telemetry snapshot into a compact binary payload.
 *
 * Returns true on success, false if an input is invalid
 * or cannot be represented by the agreed field ranges.
 */
bool telemetry_encode(
    telemetry_message_t const * p_telemetry,
    uint8_t * p_payload,
    uint32_t payload_size
);

#endif /* TELEMETRY_CODEC_H */
