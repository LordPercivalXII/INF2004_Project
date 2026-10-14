#ifndef CODE39_H
#define CODE39_H

/*
 * Pure Code 39 decoder (no Pico SDK / FreeRTOS dependencies) so it can be
 * unit-tested on a PC. Input is a list of element widths in time units
 * (microseconds here), alternating BAR, SPACE, BAR, ... starting with a bar.
 *
 * Code 39 layout: each character = 9 elements (5 bars + 4 spaces), exactly
 * 3 of them wide, followed by one narrow inter-character gap (a space).
 * A full symbol is  *PAYLOAD*  so N characters give 10*N - 1 elements.
 * Wide/narrow is classified per character using the 3 widest elements, so the
 * decoder tolerates slow speed drift across the barcode.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef CODE39_MAX_PAYLOAD
#define CODE39_MAX_PAYLOAD      8U                                   /* chars between the '*'s */
#endif
#define CODE39_MAX_ELEMENTS     (10U * (CODE39_MAX_PAYLOAD + 2U) - 1U)
#define CODE39_MIN_ELEMENTS     29U                                  /* "*X*" */

#ifndef CODE39_MIN_WIDE_RATIO_X10
#define CODE39_MIN_WIDE_RATIO_X10 15U   /* narrowest wide must be >= 1.5x widest narrow */
#endif

/* Decode element widths into the payload text (start/stop '*' removed).
 * Tries the forward direction first, then the reverse direction, so the
 * barcode can be read whichever way the robot crosses it.
 * out_size must be >= payload length + 1. Returns true on a valid symbol. */
bool Code39_Decode(const uint32_t *widths, size_t count, char *out, size_t out_size);

/* Returns the 9-bit wide/narrow pattern (MSB = first bar, 1 = wide). */
bool Code39_PatternFor(char ch, uint16_t *pattern);

/* Builds element widths for "*payload*" (used by simulation and tests).
 * Returns the number of elements written, or 0 if it does not fit. */
size_t Code39_EncodeWidths(const char *payload, uint32_t narrow, uint32_t wide,
                           uint32_t *out, size_t out_capacity);

#endif
