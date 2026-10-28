/** @file route.c
 *
 * @brief Junction routing for subsystem 3.
 *
 *   - Route table: which barcode character means LEFT / RIGHT / STRAIGHT /
 *     U-TURN.  Empty at boot; filled at run time from MQTT_ROUTE_TOPIC.
 *     Nothing in the firmware decides what a character means.
 *   - Waypoint memory: the last decoded character is stored in RAM and
 *     held until the next junction; the last ROUTE_HISTORY_LENGTH reads
 *     are kept for telemetry and debugging.
 *   - Junction decision: at a junction the stored character is looked up
 *     and the waypoint is consumed.
 *
 * MQTT payload (one message may hold many entries, applied in order):
 *
 *     A:L,B:R,C:S,D:U         character ':' action, comma separated
 *     A=LEFT;B=RIGHT          '=' and ';' work too; actions L/LEFT,
 *                             R/RIGHT, S/STRAIGHT, U/UTURN/U-TURN
 *     {"A":"L","B":"R"}       JSON object form ({ } and " are ignored)
 *     *:S                     action for characters that are not mapped
 *     CLEAR                   forget every mapping and the default first
 *                             (e.g. CLEAR,A:L)
 *
 * Characters are any Code 39 payload character (0-9, A-Z, - . $ / + %),
 * case-insensitive.  Whitespace is ignored, so the Code 39 space
 * character cannot be mapped.  Updates are cumulative unless CLEAR is
 * sent; publish with the MQTT "retain" flag so the car receives the
 * table again after every reconnect.
 *
 * Concurrency: the WiFi service task only copies the payload into
 * gh_route_cfg_mbf (TMO_POL, never blocks).  line_task parses it and is
 * the only writer of the table and the waypoint; gh_route_mtx protects
 * them against readers in other tasks (route_get_status).
 */

#include "subsystem3.h"
#include "code39.h"

#define ROUTE_TOKEN_MAX         (16U)
#define ROUTE_WILDCARD          ('*')
#define ROUTE_ASCII_MASK        (0x7FU)

ID gh_route_cfg_mbf = E_ID;
ID gh_route_mtx     = E_ID;

static route_table_t    g_table;
static route_status_t   g_status;
static volatile uint32_t g_config_drops = 0U;

/* ------------------------------------------------------------------ */
/* Pure helpers (unit-tested in subsystem3_test.c)                     */
/* ------------------------------------------------------------------ */

static char
route_upper (char symbol)
{
    char result = symbol;

    if ((symbol >= 'a') && (symbol <= 'z'))
    {
        result = (char) (symbol - ('a' - 'A'));
    }

    return (result);
}

/*!
 * @brief Case-insensitive comparison of a token with a keyword.
 */
static bool
route_word_equal (char const * p_token, char const * p_word)
{
    uint32_t idx = 0U;

    while (('\0' != p_word[idx]) &&
           (route_upper(p_token[idx]) == p_word[idx]))
    {
        idx++;
    }

    return (('\0' == p_word[idx]) && ('\0' == p_token[idx]));
}

/*!
 * @brief Parse an action word.
 *
 * @return barcode_command_t value, or ROUTE_UNMAPPED if not understood.
 */
static uint8_t
route_parse_action (char const * p_word)
{
    uint8_t action = ROUTE_UNMAPPED;

    if (route_word_equal(p_word, "L") || route_word_equal(p_word, "LEFT"))
    {
        action = (uint8_t) BARCODE_CMD_LEFT;
    }
    else if (route_word_equal(p_word, "R") ||
             route_word_equal(p_word, "RIGHT"))
    {
        action = (uint8_t) BARCODE_CMD_RIGHT;
    }
    else if (route_word_equal(p_word, "S") ||
             route_word_equal(p_word, "STRAIGHT"))
    {
        action = (uint8_t) BARCODE_CMD_NONE;
    }
    else if (route_word_equal(p_word, "U") ||
             route_word_equal(p_word, "UTURN") ||
             route_word_equal(p_word, "U-TURN"))
    {
        action = (uint8_t) BARCODE_CMD_UTURN;
    }
    else
    {
        /* Not an action. */
    }

    return (action);
}

/*!
 * @brief Apply one token ("CLEAR", "A:L", "*:S") to the table.
 */
static void
route_apply_token (route_table_t * p_table, char const * p_token,
                   route_parse_result_t * p_result)
{
    char const symbol = route_upper(p_token[0]);
    uint8_t    action = ROUTE_UNMAPPED;
    bool       b_ok   = false;

    if (route_word_equal(p_token, "CLEAR"))
    {
        route_table_reset(p_table);
        p_result->b_cleared = true;
        b_ok = true;
    }
    else if (('\0' != symbol) &&
             ((':' == p_token[1]) || ('=' == p_token[1])))
    {
        action = route_parse_action(&p_token[2]);

        if ((ROUTE_UNMAPPED != action) && (ROUTE_WILDCARD == symbol))
        {
            p_table->default_action = action;
            b_ok = true;
        }
        else if ((ROUTE_UNMAPPED != action) &&
                 code39_pattern_for(symbol, NULL))
        {
            p_table->action[(uint8_t) symbol & ROUTE_ASCII_MASK] = action;
            b_ok = true;
        }
        else
        {
            /* Unknown action, or not a Code 39 character. */
        }
    }
    else
    {
        /* Not "X:action". */
    }

    if (b_ok)
    {
        p_result->applied++;
    }
    else
    {
        p_result->rejected++;
    }
}

/*!
 * @brief Forget every mapping; unmapped characters go straight.
 */
void
route_table_reset (route_table_t * p_table)
{
    uint32_t idx;

    for (idx = 0U; idx < ROUTE_TABLE_SIZE; idx++)
    {
        p_table->action[idx] = ROUTE_UNMAPPED;
    }

    p_table->default_action = (uint8_t) BARCODE_CMD_NONE;
}

/*!
 * @brief Parse a route message (format at the top of this file) into the
 *        table.  Entries are applied in order; bad ones are skipped.
 *
 * @param[in,out] p_table   Table to update.
 * @param[in]     p_text    Payload (need not be NUL-terminated).
 * @param[in]     length    Payload length in bytes.
 * @param[out]    p_result  Counts of applied and rejected entries.
 */
void
route_table_parse (route_table_t * p_table, char const * p_text,
                   uint32_t length, route_parse_result_t * p_result)
{
    char     token[ROUTE_TOKEN_MAX + 1U];
    uint32_t token_len  = 0U;
    bool     b_too_long = false;
    uint32_t idx;

    p_result->applied   = 0U;
    p_result->rejected  = 0U;
    p_result->b_cleared = false;

    for (idx = 0U; idx <= length; idx++)
    {
        char const chr = (idx < length) ? p_text[idx] : ',';

        if ((',' == chr) || (';' == chr) || ('\0' == chr))
        {
            token[token_len] = '\0';

            if (b_too_long)
            {
                p_result->rejected++;
            }
            else if (token_len > 0U)
            {
                route_apply_token(p_table, token, p_result);
            }
            else
            {
                /* Empty entry, e.g. a trailing comma. */
            }

            token_len  = 0U;
            b_too_long = false;
        }
        else if ((' ' == chr) || ('\t' == chr) || ('\r' == chr) ||
                 ('\n' == chr) || ('"' == chr) || ('{' == chr) ||
                 ('}' == chr))
        {
            /* Layout and JSON punctuation are ignored. */
        }
        else if (token_len < ROUTE_TOKEN_MAX)
        {
            token[token_len] = chr;
            token_len++;
        }
        else
        {
            b_too_long = true;
        }
    }
}

/*!
 * @brief Action for a character: its mapping, else the default action.
 */
uint8_t
route_table_lookup (route_table_t const * p_table, char symbol)
{
    uint8_t const index  = (uint8_t) route_upper(symbol) & ROUTE_ASCII_MASK;
    uint8_t       action = p_table->action[index];

    if (ROUTE_UNMAPPED == action)
    {
        action = p_table->default_action;
    }

    return (action);
}

char const *
route_action_name (uint8_t action)
{
    char const * p_name = "?";

    switch (action)
    {
        case BARCODE_CMD_NONE:
            p_name = "STRAIGHT";
            break;

        case BARCODE_CMD_LEFT:
            p_name = "LEFT";
            break;

        case BARCODE_CMD_RIGHT:
            p_name = "RIGHT";
            break;

        case BARCODE_CMD_UTURN:
            p_name = "U-TURN";
            break;

        default:
            p_name = "UNMAPPED";
            break;
    }

    return (p_name);
}

/* ------------------------------------------------------------------ */
/* MQTT entry point (WiFi service task context)                        */
/* ------------------------------------------------------------------ */

/*!
 * @brief Hand a route message from the MQTT client to line_task.
 *
 * Runs in the WiFi service task, so it only copies and never waits.
 * The overriding weak definition is in mqtt_utk.c.
 */
void
mqtt_utk_route_rx (char const * p_payload, uint32_t length)
{
    if ((NULL == p_payload) || (0U == length) ||
        (length > ROUTE_CONFIG_MAX_LEN) || (gh_route_cfg_mbf <= 0) ||
        (E_OK != tk_snd_mbf(gh_route_cfg_mbf, (void *) p_payload,
                            (INT) length, TMO_POL)))
    {
        g_config_drops++;
    }
}

/* ------------------------------------------------------------------ */
/* Owned by line_task                                                  */
/* ------------------------------------------------------------------ */

/*!
 * @brief Empty table, no waypoint.  Called by line_task before its loop.
 */
void
route_init (void)
{
    if (E_OK == tk_loc_mtx(gh_route_mtx, TMO_FEVR))
    {
        route_table_reset(&g_table);
        g_status = (route_status_t) {0};
        g_status.default_action = g_table.default_action;
        (void) tk_unl_mtx(gh_route_mtx);
    }

    tm_printf((UB *) "S3: route table empty; waiting for MQTT route\n");
}

static uint32_t
route_count_mapped (route_table_t const * p_table)
{
    uint32_t count = 0U;
    uint32_t idx;

    for (idx = 0U; idx < ROUTE_TABLE_SIZE; idx++)
    {
        if (ROUTE_UNMAPPED != p_table->action[idx])
        {
            count++;
        }
    }

    return (count);
}

static void
route_print_table (route_table_t const * p_table)
{
    static char line[(ROUTE_TABLE_SIZE * 3U) + 1U];  /* line_task only */
    uint32_t    used = 0U;
    uint32_t    idx;

    for (idx = 0U; idx < ROUTE_TABLE_SIZE; idx++)
    {
        if (ROUTE_UNMAPPED != p_table->action[idx])
        {
            line[used]      = (char) idx;
            line[used + 1U] = route_action_name(p_table->action[idx])[0];
            line[used + 2U] = ' ';
            used += 3U;
        }
    }

    line[used] = '\0';
    tm_printf((UB *) "S3: route map: %s(others %s)\n", line,
              route_action_name(p_table->default_action));
}

/*!
 * @brief Apply every route message received since the last call.
 *        Called by line_task once per cycle.
 */
void
route_poll_config (void)
{
    char                 text[ROUTE_CONFIG_MAX_LEN + 1U];
    route_parse_result_t result;
    INT                  length;

    length = tk_rcv_mbf(gh_route_cfg_mbf, text, TMO_POL);

    while (length > 0)
    {
        text[length] = '\0';

        if (E_OK == tk_loc_mtx(gh_route_mtx, TMO_FEVR))
        {
            route_table_parse(&g_table, text, (uint32_t) length, &result);
            g_status.mapped_count   = route_count_mapped(&g_table);
            g_status.default_action = g_table.default_action;
            g_status.config_count++;
            (void) tk_unl_mtx(gh_route_mtx);
        }

        tm_printf((UB *) "S3: route msg \"%s\": %u applied, %u rejected\n",
                  text, (unsigned int) result.applied,
                  (unsigned int) result.rejected);
        route_print_table(&g_table);

        length = tk_rcv_mbf(gh_route_cfg_mbf, text, TMO_POL);
    }
}

/*!
 * @brief Store a decoded barcode as the pending waypoint (requirement 2)
 *        and in the history.  A newer barcode replaces an unused one.
 */
void
route_store_waypoint (barcode_result_t const * p_result, uint32_t now_ms)
{
    route_waypoint_t waypoint;
    uint32_t         idx;

    waypoint.symbol     = route_upper(p_result->text[0]);
    waypoint.b_reversed = p_result->b_reversed;
    waypoint.time_ms    = now_ms;

    for (idx = 0U; idx < sizeof(waypoint.text); idx++)
    {
        waypoint.text[idx] = p_result->text[idx];
    }

    if (E_OK == tk_loc_mtx(gh_route_mtx, TMO_FEVR))
    {
        for (idx = ROUTE_HISTORY_LENGTH - 1U; idx > 0U; idx--)
        {
            g_status.history[idx] = g_status.history[idx - 1U];
        }

        g_status.history[0] = waypoint;

        if (g_status.history_count < ROUTE_HISTORY_LENGTH)
        {
            g_status.history_count++;
        }

        if (g_status.b_pending)
        {
            tm_printf((UB *) "S3: waypoint '%c' replaced before junction\n",
                      g_status.pending.symbol);
        }

        g_status.pending   = waypoint;
        g_status.b_pending = true;
        (void) tk_unl_mtx(gh_route_mtx);
    }

    tm_printf((UB *) "S3: barcode \"%s\" (%s) stored, '%c' -> %s "
              "at next junction\n", waypoint.text,
              waypoint.b_reversed ? "right-to-left" : "left-to-right",
              waypoint.symbol,
              route_action_name(route_table_lookup(&g_table,
                                                   waypoint.symbol)));
}

/*!
 * @brief At a junction: look the stored character up and consume it
 *        (requirement 4).  With no stored character, go straight.
 *
 * @param[out] p_symbol  Character used, or '\0' if none was stored.
 *
 * @return barcode_command_t action for the controller.
 */
uint8_t
route_take_junction_action (char * p_symbol)
{
    uint8_t action = (uint8_t) BARCODE_CMD_NONE;

    *p_symbol = '\0';

    if (E_OK == tk_loc_mtx(gh_route_mtx, TMO_FEVR))
    {
        if (g_status.b_pending)
        {
            *p_symbol          = g_status.pending.symbol;
            action             = route_table_lookup(&g_table, *p_symbol);
            g_status.b_pending = false;
        }

        (void) tk_unl_mtx(gh_route_mtx);
    }

    if ('\0' != *p_symbol)
    {
        tm_printf((UB *) "S3: junction: '%c' -> %s\n", *p_symbol,
                  route_action_name(action));
    }
    else
    {
        tm_printf((UB *) "S3: junction: no barcode stored -> STRAIGHT\n");
    }

    return (action);
}

/*!
 * @brief Copy of the waypoint memory and table summary (any task).
 */
void
route_get_status (route_status_t * p_status)
{
    if (E_OK == tk_loc_mtx(gh_route_mtx, TMO_FEVR))
    {
        *p_status = g_status;
        (void) tk_unl_mtx(gh_route_mtx);
    }

    p_status->config_drops = g_config_drops;
}

/*** end of file ***/
