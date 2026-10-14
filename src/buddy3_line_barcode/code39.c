#include <string.h>
#include "code39.h"

typedef struct {
    char ch;
    uint16_t pattern;   /* b s b s b s b s b, MSB first, 1 = wide */
} Code39Entry;

static const Code39Entry k_code39_table[] = {
    {'0', 0x034U}, {'1', 0x121U}, {'2', 0x061U}, {'3', 0x160U}, {'4', 0x031U},
    {'5', 0x130U}, {'6', 0x070U}, {'7', 0x025U}, {'8', 0x124U}, {'9', 0x064U},
    {'A', 0x109U}, {'B', 0x049U}, {'C', 0x148U}, {'D', 0x019U}, {'E', 0x118U},
    {'F', 0x058U}, {'G', 0x00DU}, {'H', 0x10CU}, {'I', 0x04CU}, {'J', 0x01CU},
    {'K', 0x103U}, {'L', 0x043U}, {'M', 0x142U}, {'N', 0x013U}, {'O', 0x112U},
    {'P', 0x052U}, {'Q', 0x007U}, {'R', 0x106U}, {'S', 0x046U}, {'T', 0x016U},
    {'U', 0x181U}, {'V', 0x0C1U}, {'W', 0x1C0U}, {'X', 0x091U}, {'Y', 0x190U},
    {'Z', 0x0D0U}, {'-', 0x085U}, {'.', 0x184U}, {' ', 0x0C4U}, {'*', 0x094U},
    {'$', 0x0A8U}, {'/', 0x0A2U}, {'+', 0x08AU}, {'%', 0x02AU},
};

#define CODE39_TABLE_SIZE (sizeof(k_code39_table) / sizeof(k_code39_table[0]))

bool Code39_PatternFor(char ch, uint16_t *pattern)
{
    for (size_t i = 0U; i < CODE39_TABLE_SIZE; ++i) {
        if (k_code39_table[i].ch == ch) {
            if (pattern != NULL) *pattern = k_code39_table[i].pattern;
            return true;
        }
    }
    return false;
}

static bool lookup_pattern(uint16_t pattern, char *ch)
{
    for (size_t i = 0U; i < CODE39_TABLE_SIZE; ++i) {
        if (k_code39_table[i].pattern == pattern) {
            *ch = k_code39_table[i].ch;
            return true;
        }
    }
    return false;
}

/* Classify the 9 elements of one character into a wide/narrow bit pattern.
 * The 3 widest elements are wide; reject if wide and narrow are not clearly separated. */
static bool classify_character(const uint32_t *elements, uint16_t *pattern, uint32_t *threshold)
{
    uint32_t sorted[9];
    memcpy(sorted, elements, sizeof(sorted));
    for (size_t i = 1U; i < 9U; ++i) {                 /* insertion sort, 9 items */
        const uint32_t key = sorted[i];
        size_t j = i;
        while (j > 0U && sorted[j - 1U] > key) {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = key;
    }
    const uint32_t narrow_max = sorted[5];
    const uint32_t wide_min = sorted[6];
    if (narrow_max == 0U) return false;
    if ((uint64_t)wide_min * 10U < (uint64_t)narrow_max * CODE39_MIN_WIDE_RATIO_X10) return false;

    const uint32_t split = narrow_max + (wide_min - narrow_max) / 2U;
    uint16_t bits = 0U;
    for (size_t i = 0U; i < 9U; ++i) {
        bits = (uint16_t)((bits << 1U) | (elements[i] > split ? 1U : 0U));
    }
    *pattern = bits;
    *threshold = split;
    return true;
}

static bool decode_one_direction(const uint32_t *widths, size_t count, char *out, size_t out_size)
{
    if (count < CODE39_MIN_ELEMENTS || ((count + 1U) % 10U) != 0U) return false;
    const size_t characters = (count + 1U) / 10U;
    const size_t payload_length = characters - 2U;
    if (payload_length + 1U > out_size) return false;

    for (size_t c = 0U; c < characters; ++c) {
        const uint32_t *element = &widths[c * 10U];
        uint16_t pattern = 0U;
        uint32_t split = 0U;
        char ch = '\0';
        if (!classify_character(element, &pattern, &split)) return false;
        if (!lookup_pattern(pattern, &ch)) return false;

        /* Inter-character gap must be a narrow-ish space, not a quiet zone. */
        if (c + 1U < characters && widths[c * 10U + 9U] > 3U * split) return false;

        const bool is_edge = (c == 0U) || (c == characters - 1U);
        if (is_edge != (ch == '*')) return false;     /* '*' only as start/stop */
        if (!is_edge) out[c - 1U] = ch;
    }
    out[payload_length] = '\0';
    return true;
}

bool Code39_Decode(const uint32_t *widths, size_t count, char *out, size_t out_size)
{
    if (widths == NULL || out == NULL || out_size == 0U || count > CODE39_MAX_ELEMENTS) return false;
    if (decode_one_direction(widths, count, out, out_size)) return true;

    uint32_t reversed[CODE39_MAX_ELEMENTS];
    for (size_t i = 0U; i < count; ++i) reversed[i] = widths[count - 1U - i];
    return decode_one_direction(reversed, count, out, out_size);
}

size_t Code39_EncodeWidths(const char *payload, uint32_t narrow, uint32_t wide,
                           uint32_t *out, size_t out_capacity)
{
    if (payload == NULL || out == NULL) return 0U;
    const size_t payload_length = strlen(payload);
    if (payload_length == 0U || payload_length > CODE39_MAX_PAYLOAD) return 0U;
    const size_t characters = payload_length + 2U;
    const size_t count = characters * 10U - 1U;
    if (count > out_capacity) return 0U;

    size_t n = 0U;
    for (size_t c = 0U; c < characters; ++c) {
        const char ch = (c == 0U || c == characters - 1U) ? '*' : payload[c - 1U];
        uint16_t pattern = 0U;
        if (ch == '*' && c != 0U && c != characters - 1U) return 0U;
        if (!Code39_PatternFor(ch, &pattern)) return 0U;
        for (int bit = 8; bit >= 0; --bit) {
            out[n++] = ((pattern >> (unsigned)bit) & 1U) ? wide : narrow;
        }
        if (c + 1U < characters) out[n++] = narrow;   /* inter-character gap */
    }
    return n;
}
