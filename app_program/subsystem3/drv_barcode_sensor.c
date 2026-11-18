/** @file drv_barcode_sensor.c
 *
 * @brief Barcode edge capture.  The edge callback timestamps every
 *        transition and queues the finished element for barcode_task.
 *
 * xQueueSendFromISR() becomes tk_snd_mbf(..., TMO_POL), which micro
 * T-Kernel allows from interrupt handlers.  Width-based glitch filtering
 * is done in the task, because merging a short pulse needs the element
 * after it.
 *
 * Counters shared with the ISR are single-writer (Tribunal 1, RMW): the
 * ISR only increments, the task keeps the last value it saw and reports
 * the difference, so no read-then-clear can lose an increment made by
 * the ISR between the two (the ISR may run on the other core).
 */

#include "subsystem3.h"
#include "drv_barcode_sensor.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../common/drv_gpio_irq.h"
#include "../common/fault_mask.h"

static volatile uint32_t g_last_edge_us = 0U;
static volatile uint8_t  g_level_black  = 0U;
static volatile uint32_t g_overflow_count = 0U; /* edge buffer full  */
static volatile uint32_t g_irq_count    = 0U;  /* every GP28 interrupt  */
static volatile uint32_t g_change_count = 0U;  /* real level changes    */

/*!
 * @brief true (1) while the sensor sees black.  Callable from interrupts.
 */
uint8_t
barcode_sensor_is_black (void)
{
    uint8_t const level = board_gpio_read(PIN_IR_BARCODE) ? 1U : 0U;

    return ((IR_BARCODE_BLACK_LEVEL == level) ? 1U : 0U);
}

/*!
 * @brief GP28 edge callback (interrupt context).  Kept short.
 */
static void
barcode_edge_isr (uint32_t pin, uint32_t events)
{
    uint32_t const    now_us  = board_micros();
    uint8_t const     black   = barcode_sensor_is_black();
    barcode_element_t element;

    (void) pin;
    (void) events;

    g_irq_count++;

    /* Ignore edges that did not change the level (settled bounce). */
    if (black != g_level_black)
    {
        g_change_count++;
        element.width_us = now_us - g_last_edge_us;
        element.end_us   = now_us;
        element.is_black = g_level_black;
        g_level_black    = black;
        g_last_edge_us   = now_us;

        if (E_OK != tk_snd_mbf(gh_barcode_edge_mbf, &element,
                               (INT) sizeof(element), TMO_POL))
        {
            g_overflow_count++;     /* this read will be discarded */
            fault_mask_set(FAULT_BIT_LINE);
        }
    }
}

/*!
 * @brief Configure GP28 and start capturing both edges.
 *
 * @return true if the interrupt was registered.
 */
bool
barcode_sensor_init (void)
{
    board_gpio_input(PIN_IR_BARCODE, BOARD_PULL_UP);
    g_level_black  = barcode_sensor_is_black();
    g_last_edge_us = board_micros();

    return (gpio_irq_register(PIN_IR_BARCODE,
                              (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL),
                              barcode_edge_isr));
}

/*!
 * @brief Read and clear the "edge buffer overflowed" flag.
 */
bool
barcode_sensor_take_overflow (void)
{
    static uint32_t seen = 0U;  /* task-side copy; the ISR never writes it */
    uint32_t const  now  = g_overflow_count;
    bool const      b_overflow = (now != seen);

    seen = now;

    return (b_overflow);
}

/*!
 * @brief Read and clear the edge counters (diagnostics).
 *
 * @param[out] p_irqs     GP28 interrupts since the last call.
 * @param[out] p_changes  Of those, the ones that changed the level.
 */
void
barcode_sensor_take_counts (uint32_t * p_irqs, uint32_t * p_changes)
{
    static uint32_t seen_irqs    = 0U;
    static uint32_t seen_changes = 0U;
    uint32_t const  irqs    = g_irq_count;
    uint32_t const  changes = g_change_count;

    /* Unsigned subtraction is correct across the 32-bit wrap. */
    *p_irqs      = irqs - seen_irqs;
    *p_changes   = changes - seen_changes;
    seen_irqs    = irqs;
    seen_changes = changes;
}

/*** end of file ***/
