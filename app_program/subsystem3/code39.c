/** @file code39.c
 *
 * @brief Code 39 decoder/encoder.  Port of buddy3_line_barcode/code39.c.
 */

#include "code39.h"

#define CODE39_CHAR_ELEMENTS    (9U)
#define CODE39_CHAR_STRIDE      (10U)
#define CODE39_WIDE_COUNT       (3U)
#define CODE39_GAP_LIMIT        (3U)
#define CODE39_START_STOP       ('*')

typedef struct
{
    char     symbol;
    uint16_t pattern;   /* b s b s b s b s b, MSB first, 1 = wide */
} code39_entry_t;

static code39_entry_t const g_code39_table[] =
{
    {'0', 0x034U}, {'1', 0x121U}, {'2', 0x061U}, {'3', 0x160U},
    {'4', 0x031U}, {'5', 0x130U}, {'6', 0x070U}, {'7', 0x025U},
    {'8', 0x124U}, {'9', 0x064U}, {'A', 0x109U}, {'B', 0x049U},
    {'C', 0x148U}, {'D', 0x019U}, {'E', 0x118U}, {'F', 0x058U},
    {'G', 0x00DU}, {'H', 0x10CU}, {'I', 0x04CU}, {'J', 0x01CU},
    {'K', 0x103U}, {'L', 0x043U}, {'M', 0x142U}, {'N', 0x013U},
    {'O', 0x112U}, {'P', 0x052U}, {'Q', 0x007U}, {'R', 0x106U},
    {'S', 0x046U}, {'T', 0x016U}, {'U', 0x181U}, {'V', 0x0C1U},
    {'W', 0x1C0U}, {'X', 0x091U}, {'Y', 0x190U}, {'Z', 0x0D0U},
    {'-', 0x085U}, {'.', 0x184U}, {' ', 0x0C4U}, {'*', 0x094U},
    {'$', 0x0A8U}, {'/', 0x0A2U}, {'+', 0x08AU}, {'%', 0x02AU},
};

#define CODE39_TABLE_SIZE \
    (sizeof(g_code39_table) / sizeof(g_code39_table[0]))

/*!
 * @brief Look up the 9-bit wide/narrow pattern of a character.
 *
 * @return true if the character exists in Code 39.
 */
bool
code39_pattern_for (char symbol, uint16_t * p_pattern)
{
    bool   b_found = false;
    uint32_t idx;

    for (idx = 0U; (idx < CODE39_TABLE_SIZE) && !b_found; idx++)
    {
        if (symbol == g_code39_table[idx].symbol)
        {
            if (NULL != p_pattern)
            {
                *p_pattern = g_code39_table[idx].pattern;
            }

            b_found = true;
        }
    }

    return (b_found);
}

static bool
code39_lookup_pattern (uint16_t pattern, char * p_symbol)
{
    bool   b_found = false;
    uint32_t idx;

    for (idx = 0U; (idx < CODE39_TABLE_SIZE) && !b_found; idx++)
    {
        if (pattern == g_code39_table[idx].pattern)
        {
            *p_symbol = g_code39_table[idx].symbol;
            b_found = true;
        }
    }

    return (b_found);
}

/*!
 * @brief Estimate the edge bias d of one character.
 *
 * An IR sensor whose threshold is not exactly at mid-grey switches late
 * on one colour and early on the other, so every bar comes out shorter
 * by some time d and every space longer by d (or the reverse).  Narrow
 * bars and narrow spaces must be equal in Code 39, and every character
 * has at least 3 narrow bars and 1 narrow space, so
 * d = (narrowest space - narrowest bar) / 2.
 */
static int32_t
code39_estimate_bias (uint32_t const * p_elements)
{
    uint32_t min_bar   = UINT32_MAX;
    uint32_t min_space = UINT32_MAX;
    uint32_t idx;

    for (idx = 0U; idx < CODE39_CHAR_ELEMENTS; idx++)
    {
        if (0U == (idx % 2U))
        {
            min_bar = (p_elements[idx] < min_bar) ? p_elements[idx] : min_bar;
        }
        else
        {
            min_space = (p_elements[idx] < min_space) ? p_elements[idx]
                                                      : min_space;
        }
    }

    return (((int32_t) min_space - (int32_t) min_bar) / 2);
}

/*!
 * @brief Classify one character's 9 elements after adding bias to bars
 *        and subtracting it from spaces: the 3 widest are wide.
 *        Rejected if wide and narrow are not clearly separated.
 *
 * @param[in]  p_elements  9 widths, starting with a bar.
 * @param[in]  bias        Edge bias d (0 = take widths as measured).
 * @param[out] p_pattern   9-bit wide/narrow pattern (MSB first).
 * @param[out] p_split     Wide/narrow threshold (corrected units).
 */
static bool
code39_classify (uint32_t const * p_elements, int32_t bias,
                 uint16_t * p_pattern, uint32_t * p_split)
{
    uint32_t corrected[CODE39_CHAR_ELEMENTS];
    uint32_t sorted[CODE39_CHAR_ELEMENTS];
    int32_t  value;
    uint32_t narrow_max;
    uint32_t wide_min;
    uint32_t split;
    uint16_t bits = 0U;
    bool     b_ok = false;
    uint32_t idx;
    uint32_t pos;

    for (idx = 0U; idx < CODE39_CHAR_ELEMENTS; idx++)
    {
        value = (int32_t) p_elements[idx] +
                ((0U == (idx % 2U)) ? bias : -bias);
        corrected[idx] = (value > 0) ? (uint32_t) value : 1U;
    }

    for (idx = 0U; idx < CODE39_CHAR_ELEMENTS; idx++)
    {
        uint32_t const key = corrected[idx];

        pos = idx;

        while ((pos > 0U) && (sorted[pos - 1U] > key))
        {
            sorted[pos] = sorted[pos - 1U];
            pos--;
        }

        sorted[pos] = key;
    }

    narrow_max = sorted[CODE39_CHAR_ELEMENTS - CODE39_WIDE_COUNT - 1U];
    wide_min   = sorted[CODE39_CHAR_ELEMENTS - CODE39_WIDE_COUNT];

    if ((0U != narrow_max) &&
        (((uint64_t) wide_min * 10U) >=
         ((uint64_t) narrow_max * CODE39_MIN_WIDE_RATIO_X10)))
    {
        split = narrow_max + ((wide_min - narrow_max) / 2U);

        for (idx = 0U; idx < CODE39_CHAR_ELEMENTS; idx++)
        {
            bits = (uint16_t) ((uint32_t) (bits << 1U) |
                               ((corrected[idx] > split) ? 1U : 0U));
        }

        *p_pattern = bits;
        *p_split   = split;
        b_ok       = true;
    }

    return (b_ok);
}

/*!
 * @brief Decode one character: as measured first, and only if that does
 *        not give a valid character, again with edge-bias correction.
 */
static bool
code39_decode_char (uint32_t const * p_elements, char * p_symbol,
                    uint32_t * p_split, int32_t * p_bias)
{
    uint16_t pattern = 0U;
    bool     b_ok;

    *p_bias = 0;
    b_ok = code39_classify(p_elements, 0, &pattern, p_split) &&
           code39_lookup_pattern(pattern, p_symbol);

    if (!b_ok)
    {
        *p_bias = code39_estimate_bias(p_elements);
        b_ok = code39_classify(p_elements, *p_bias, &pattern, p_split) &&
               code39_lookup_pattern(pattern, p_symbol);
    }

    return (b_ok);
}

static bool
code39_decode_forward (uint32_t const * p_widths, uint32_t count,
                       char * p_out, uint32_t out_size)
{
    uint32_t   characters;
    uint32_t   chr;
    bool     b_ok = false;

    if ((count >= CODE39_MIN_ELEMENTS) &&
        (0U == ((count + 1U) % CODE39_CHAR_STRIDE)))
    {
        characters = (count + 1U) / CODE39_CHAR_STRIDE;
        b_ok = ((characters - 2U) + 1U) <= out_size;

        for (chr = 0U; (chr < characters) && b_ok; chr++)
        {
            uint32_t const * p_elem  = &p_widths[chr * CODE39_CHAR_STRIDE];
            bool const       b_edge  = (0U == chr) ||
                                       ((characters - 1U) == chr);
            uint32_t         split   = 0U;
            int32_t          bias    = 0;
            char             symbol  = '\0';

            b_ok = code39_decode_char(p_elem, &symbol, &split, &bias);

            /* Inter-character gap (a space, so minus the bias) must be
               narrow-ish, not a quiet zone. */
            if (b_ok && ((chr + 1U) < characters) &&
                (((int32_t) p_elem[CODE39_CHAR_ELEMENTS] - bias) >
                 (int32_t) (CODE39_GAP_LIMIT * split)))
            {
                b_ok = false;
            }

            /* '*' only as start/stop character. */
            if (b_ok && (b_edge != (CODE39_START_STOP == symbol)))
            {
                b_ok = false;
            }

            if (b_ok && !b_edge)
            {
                p_out[chr - 1U] = symbol;
            }
        }

        if (b_ok)
        {
            p_out[characters - 2U] = '\0';
        }
    }

    return (b_ok);
}

/*!
 * @brief Decode element widths into the payload text ('*' removed) and
 *        report which way the symbol was crossed (FR8: both directions).
 *
 * Code 39 is not palindromic element-by-element, so a symbol read from
 * right to left only decodes after its element list is reversed.  The
 * forward order is tried first, then the reversed order.
 *
 * @param[in]  p_widths      Element widths, starting with a bar.
 * @param[in]  count         Number of elements.
 * @param[out] p_out         Payload text, NUL-terminated.
 * @param[in]  out_size      Size of p_out (>= payload length + 1).
 * @param[out] p_b_reversed  true if it decoded right-to-left; may be NULL.
 *
 * @return true on a valid symbol.
 */
bool
code39_decode_dir (uint32_t const * p_widths, uint32_t count, char * p_out,
                   uint32_t out_size, bool * p_b_reversed)
{
    uint32_t reversed[CODE39_MAX_ELEMENTS];
    bool     b_ok       = false;
    bool     b_backward = false;
    uint32_t trim;
    uint32_t lead;
    uint32_t length;
    uint32_t idx;

    if ((NULL != p_widths) && (NULL != p_out) && (0U != out_size) &&
        (count <= CODE39_MAX_ELEMENTS))
    {
        for (idx = 0U; idx < count; idx++)
        {
            reversed[idx] = p_widths[count - 1U - idx];
        }

        /* Try the whole scan first, then windows with an even number of
           stray elements (an extra wobble when the swipe starts or stops)
           removed from the ends, fewest removed first.  Windows keep a
           bar at both ends; '*' start/stop and the gap checks reject
           anything that is not a complete symbol. */
        for (trim = 0U; ((trim + CODE39_MIN_ELEMENTS) <= count) && !b_ok;
             trim += 2U)
        {
            length = count - trim;

            for (lead = 0U; (lead <= trim) && !b_ok &&
                 (0U == ((length + 1U) % CODE39_CHAR_STRIDE)); lead += 2U)
            {
                b_ok = code39_decode_forward(&p_widths[lead], length, p_out,
                                             out_size);

                if (!b_ok)
                {
                    b_ok = code39_decode_forward(&reversed[lead], length,
                                                 p_out, out_size);
                    b_backward = b_ok;
                }
            }
        }
    }

    if (NULL != p_b_reversed)
    {
        *p_b_reversed = b_backward;
    }

    return (b_ok);
}

/*!
 * @brief Decode in either direction, without reporting which.
 *
 * @return true on a valid symbol.
 */
bool
code39_decode (uint32_t const * p_widths, uint32_t count, char * p_out,
               uint32_t out_size)
{
    return (code39_decode_dir(p_widths, count, p_out, out_size, NULL));
}

/*!
 * @brief Build element widths for "*payload*" (simulation and tests).
 *
 * @return Number of elements written, or 0 if it does not fit or the
 *         payload holds a character Code 39 cannot encode.
 */
uint32_t
code39_encode_widths (char const * p_payload, uint32_t narrow,
                      uint32_t wide, uint32_t * p_out, uint32_t out_capacity)
{
    uint32_t length     = 0U;
    uint32_t written    = 0U;
    uint32_t characters = 0U;
    uint32_t chr;
    int32_t bit;
    bool   b_ok       = (NULL != p_payload) && (NULL != p_out);

    while (b_ok && ('\0' != p_payload[length]) &&
           (length <= CODE39_MAX_PAYLOAD))
    {
        length++;
    }

    characters = length + 2U;
    b_ok = b_ok && (0U != length) && (length <= CODE39_MAX_PAYLOAD) &&
           (((characters * CODE39_CHAR_STRIDE) - 1U) <= out_capacity);

    for (chr = 0U; (chr < characters) && b_ok; chr++)
    {
        bool const b_edge  = (0U == chr) || ((characters - 1U) == chr);
        char const symbol  = b_edge ? CODE39_START_STOP
                                    : p_payload[chr - 1U];
        uint16_t   pattern = 0U;

        b_ok = (b_edge || (CODE39_START_STOP != symbol)) &&
               code39_pattern_for(symbol, &pattern);

        for (bit = 8; (bit >= 0) && b_ok; bit--)
        {
            p_out[written] = (0U != ((pattern >> (uint32_t) bit) & 1U))
                             ? wide : narrow;
            written++;
        }

        if (b_ok && ((chr + 1U) < characters))
        {
            p_out[written] = narrow;    /* inter-character gap */
            written++;
        }
    }

    return (b_ok ? written : 0U);
}

/*** end of file ***/
