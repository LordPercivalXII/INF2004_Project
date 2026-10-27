/** @file barcode.c
 *
 * @brief Barcode task (FR8, FR9): debounce edges, collect element widths,
 *        decode Code 39 and leave the result for line_task.
 *
 *   FreeRTOS                              micro T-Kernel
 *   ------------------------------------  -------------------------------
 *   xQueueReceive(edge_q, .., wait)       tk_rcv_mbf(.., wait_ms)
 *     returns pdFAIL on timeout             returns E_TMOUT on timeout
 *   xQueueOverwrite(result_q)             barcode_result_put() (mutex)
 */

#include "subsystem3.h"
#include "code39.h"
#include "drv_barcode_sensor.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"

typedef struct
{
    uint32_t          widths[CODE39_MAX_ELEMENTS];
    uint32_t            count;
    bool              b_active;
    bool              b_overflow;
    barcode_element_t pending;  /* held back so a glitch can merge in */
    uint32_t          widest_us;
    uint32_t          last_edge_us;
} barcode_session_t;

static barcode_result_t g_result;
static bool             gb_result_ready = false;

/* ------------------------------------------------------------------ */
/* One-slot result shared with line_task                               */
/* ------------------------------------------------------------------ */

/*!
 * @brief Store a result, replacing any not yet taken (was
 *        xQueueOverwrite on a length-1 queue).
 */
void
barcode_result_put (barcode_result_t const * p_result)
{
    if (E_OK == tk_loc_mtx(gh_barcode_result_mtx, TMO_FEVR))
    {
        g_result        = *p_result;
        gb_result_ready = true;
        (void) tk_unl_mtx(gh_barcode_result_mtx);
    }
}

/*!
 * @brief Take the stored result, if any, without waiting.
 *
 * @return true if a result was taken.
 */
bool
barcode_result_take (barcode_result_t * p_result)
{
    bool b_taken = false;

    if (E_OK == tk_loc_mtx(gh_barcode_result_mtx, TMO_FEVR))
    {
        if (gb_result_ready)
        {
            *p_result       = g_result;
            gb_result_ready = false;
            b_taken         = true;
        }

        (void) tk_unl_mtx(gh_barcode_result_mtx);
    }

    return (b_taken);
}

/*!
 * @brief Map decoded text to a turn command.
 */
uint16_t
barcode_to_command (char const * p_text)
{
    uint16_t command = (uint16_t) BARCODE_CMD_NONE;

    if (BARCODE_LEFT_CHAR == p_text[0])
    {
        command = (uint16_t) BARCODE_CMD_LEFT;
    }
    else if (BARCODE_RIGHT_CHAR == p_text[0])
    {
        command = (uint16_t) BARCODE_CMD_RIGHT;
    }
    else if (BARCODE_UTURN_CHAR == p_text[0])
    {
        command = (uint16_t) BARCODE_CMD_UTURN;
    }
    else
    {
        /* Any other payload: continue straight. */
    }

    return (command);
}

/* ------------------------------------------------------------------ */
/* Session                                                             */
/* ------------------------------------------------------------------ */

static void
barcode_session_reset (barcode_session_t * p_session)
{
    p_session->count      = 0U;
    p_session->b_active   = false;
    p_session->b_overflow = false;
    p_session->widest_us  = 0U;
}

static void
barcode_session_push (barcode_session_t * p_session, uint32_t width_us)
{
    if (p_session->count < CODE39_MAX_ELEMENTS)
    {
        p_session->widths[p_session->count] = width_us;
        p_session->count++;
    }
    else
    {
        p_session->b_overflow = true;
    }

    if (width_us > p_session->widest_us)
    {
        p_session->widest_us = width_us;
    }
}

static void
barcode_session_finish (barcode_session_t * p_session)
{
    barcode_result_t result = {0};
    bool const       b_isr_overflow = barcode_sensor_take_overflow();

    /* Final bar: the trailing quiet zone has no closing edge. */
    barcode_session_push(p_session, p_session->pending.width_us);

    if ((!p_session->b_overflow) && (!b_isr_overflow) &&
        code39_decode(p_session->widths, p_session->count, result.text,
                      sizeof(result.text)))
    {
        result.command = barcode_to_command(result.text);
        barcode_result_put(&result);
    }

    barcode_session_reset(p_session);
}

/*!
 * @brief How long to wait for the next edge before the read is over.
 */
static TMO
barcode_session_wait (barcode_session_t const * p_session)
{
    TMO      wait       = TMO_FEVR;
    uint32_t end_gap_ms;
    uint32_t elapsed_ms;

    if (p_session->b_active)
    {
        end_gap_ms = (p_session->widest_us * BARCODE_END_GAP_FACTOR) /
                     1000U;

        if (end_gap_ms < BARCODE_END_GAP_MIN_MS)
        {
            end_gap_ms = BARCODE_END_GAP_MIN_MS;
        }

        elapsed_ms = (board_micros() - p_session->last_edge_us) / 1000U;
        wait = (elapsed_ms >= end_gap_ms) ? TMO_POL
                                          : (TMO) (end_gap_ms - elapsed_ms);
    }

    return (wait);
}

/*!
 * @brief Feed one element into the session.
 */
static void
barcode_session_add (barcode_session_t * p_session,
                     barcode_element_t const * p_element)
{
    if (!p_session->b_active)
    {
        /* Idle: a read starts at the first bar that is not a glitch. */
        if ((0U != p_element->is_black) &&
            (p_element->width_us >= BARCODE_GLITCH_US))
        {
            p_session->b_active     = true;
            p_session->pending      = *p_element;
            p_session->last_edge_us = p_element->end_us;
            p_session->widest_us    = p_element->width_us;
        }
    }
    else
    {
        p_session->last_edge_us = p_element->end_us;

        if ((p_element->width_us < BARCODE_GLITCH_US) ||
            (p_element->is_black == p_session->pending.is_black))
        {
            /* Debounce: merge a short pulse (and the same-colour rest
               after it) into the element in progress. */
            p_session->pending.width_us += p_element->width_us;
        }
        else
        {
            barcode_session_push(p_session, p_session->pending.width_us);
            p_session->pending = *p_element;
        }
    }
}

/*!
 * @brief Barcode task: event-driven by GP28 edges.
 */
void
barcode_task (INT stacd, void * p_exinf)
{
    static barcode_session_t session;   /* keeps 400 B off the stack */
    barcode_element_t        element;
    INT                      result;

    (void) stacd;
    (void) p_exinf;

    barcode_session_reset(&session);

#if !ROBOT_SIMULATION
    if (!barcode_sensor_init())
    {
        tm_printf((UB *) "S3: barcode interrupt registration failed\n");
    }
#endif

    for (;;)
    {
        result = tk_rcv_mbf(gh_barcode_edge_mbf, &element,
                            barcode_session_wait(&session));

        if (result > 0)
        {
            barcode_session_add(&session, &element);
        }
        else if (session.b_active)
        {
            barcode_session_finish(&session);   /* quiet zone reached */
        }
        else
        {
            /* Spurious wake while idle. */
        }
    }
}

/*** end of file ***/
