/** @file barcode.c
 *
 * @brief Barcode task (FR8): debounce edges, collect element widths,
 *        decode Code 39 in either direction and leave the decoded text
 *        for line_task, which stores it as the next waypoint.
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
    uint32_t          received;     /* elements received this scan   */
    uint32_t          merged;       /* of those, merged as glitches  */
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
    p_session->received   = 0U;
    p_session->merged     = 0U;
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

#if S3_BARCODE_DEBUG
/*!
 * @brief Print one scan: element widths (us), bars marked B, spaces s.
 *        A clean Code 39 read shows two width groups ~2-3x apart, with
 *        bars and spaces of the same kind about equally wide.
 */
static void
barcode_debug_dump (barcode_session_t const * p_session, bool b_decoded,
                    bool b_overflow, barcode_result_t const * p_result)
{
    uint32_t idx;
    uint32_t irqs;
    uint32_t changes;

    barcode_sensor_take_counts(&irqs, &changes);
    tm_printf((UB *) "S3 dbg: GP28 IRQs %u, level changes %u, received %u,"
              " merged %u, now %s\n", (unsigned int) irqs,
              (unsigned int) changes, (unsigned int) p_session->received,
              (unsigned int) p_session->merged,
              (0U != barcode_sensor_is_black()) ? "BLACK" : "WHITE");
    tm_printf((UB *) "S3 dbg: %u elements, ", (unsigned int) p_session->count);

    if (b_decoded)
    {
        tm_printf((UB *) "decoded \"%s\" (%s)\n", p_result->text,
                  p_result->b_reversed ? "right-to-left" : "left-to-right");
    }
    else
    {
        tm_printf((UB *) "NOT decoded%s\n",
                  b_overflow ? " (overflow)" : "");
    }

    for (idx = 0U; idx < p_session->count; idx++)
    {
        tm_printf((UB *) "%c%u%s", (0U == (idx % 2U)) ? 'B' : 's',
                  (unsigned int) p_session->widths[idx],
                  (9U == (idx % 10U)) ? "\n" : " ");
    }

    tm_printf((UB *) "\n");
}
#endif /* S3_BARCODE_DEBUG */

static void
barcode_session_finish (barcode_session_t * p_session)
{
    barcode_result_t result = {0};
    bool const       b_isr_overflow = barcode_sensor_take_overflow();
    bool             b_decoded      = false;

    /* Final bar: the trailing quiet zone has no closing edge. */
    barcode_session_push(p_session, p_session->pending.width_us);

    if ((!p_session->b_overflow) && (!b_isr_overflow) &&
        code39_decode_dir(p_session->widths, p_session->count,
                          result.text, sizeof(result.text),
                          &result.b_reversed))
    {
        /* FR8: either crossing direction; mapping to an action happens
           later, at the junction (route.c), not here. */
        b_decoded = true;
        barcode_result_put(&result);
    }

#if S3_BARCODE_DEBUG
    barcode_debug_dump(p_session, b_decoded,
                       (p_session->b_overflow || b_isr_overflow), &result);
#else
    (void) b_decoded;
#endif

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
            p_session->received     = 1U;
        }
    }
    else
    {
        p_session->last_edge_us = p_element->end_us;
        p_session->received++;

        if ((p_element->width_us < BARCODE_GLITCH_US) ||
            (p_element->is_black == p_session->pending.is_black))
        {
            /* Debounce: merge a short pulse (and the same-colour rest
               after it) into the element in progress. */
            p_session->pending.width_us += p_element->width_us;
            p_session->merged++;
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

#if S3_BARCODE_HW
    if (!barcode_sensor_init())
    {
        tm_printf((UB *) "S3: barcode interrupt registration failed\n");
    }

#if S3_BARCODE_DEBUG
    tm_printf((UB *) "S3 dbg: barcode sensor GP28 reads %s at start\n",
              (0U != barcode_sensor_is_black()) ? "BLACK" : "WHITE");
#endif
#endif

    for (;;)
    {
        result = tk_rcv_mbf(gh_barcode_edge_mbf, &element,
                            barcode_session_wait(&session));

        if (result > 0)
        {
#if S3_BARCODE_DEBUG
            /* Live view: one line per level change on GP28. */
            tm_printf((UB *) "S3 dbg: edge, %s lasted %u us\n",
                      (0U != element.is_black) ? "BLACK" : "white",
                      (unsigned int) element.width_us);
#endif
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
