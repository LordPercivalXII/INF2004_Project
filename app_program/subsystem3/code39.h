/** @file code39.h
 *
 * @brief Pure Code 39 decoder/encoder (no kernel or hardware use, so it
 *        can also be unit-tested on a PC).
 *
 * Input is a list of element widths (microseconds here), alternating
 * BAR, SPACE, BAR, ... starting with a bar.  Each character is 9
 * elements (5 bars + 4 spaces, exactly 3 wide) followed by one narrow
 * inter-character gap.  A full symbol "*PAYLOAD*" of N characters is
 * 10 * N - 1 elements.  Wide/narrow is classified per character, so the
 * decoder tolerates slow speed drift across the barcode.
 */

#ifndef CODE39_H
#define CODE39_H

#include <stdbool.h>
#include <stdint.h>

#ifndef NULL
#define NULL    ((void *) 0)    /* <stddef.h> clashes with the kernel */
#endif

#define CODE39_MAX_PAYLOAD          (8U)    /* chars between the '*'s */
#define CODE39_MAX_ELEMENTS         ((10U * (CODE39_MAX_PAYLOAD + 2U)) - 1U)
#define CODE39_MIN_ELEMENTS         (29U)   /* "*X*" */
#define CODE39_MIN_WIDE_RATIO_X10   (15U)   /* wide >= 1.5 x narrow */

bool   code39_decode(uint32_t const * p_widths, uint32_t count,
                     char * p_out, uint32_t out_size);
bool   code39_decode_dir(uint32_t const * p_widths, uint32_t count,
                         char * p_out, uint32_t out_size,
                         bool * p_b_reversed);
bool   code39_pattern_for(char symbol, uint16_t * p_pattern);
uint32_t code39_encode_widths(char const * p_payload, uint32_t narrow,
                            uint32_t wide, uint32_t * p_out,
                            uint32_t out_capacity);

#endif /* CODE39_H */

/*** end of file ***/
