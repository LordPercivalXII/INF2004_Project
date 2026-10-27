/** @file drv_encoder.c
 *
 * @brief Wheel encoder driver.  Counts are written only by the GPIO
 *        interrupt callback (or, in simulation, only by motion_task).
 *
 * FreeRTOS read both counters inside save_and_disable_interrupts().
 * On the dual-core kernel that would only mask the local core, so
 * instead each counter is one aligned 32-bit word with a single writer:
 * each read is atomic on Cortex-M0+.  The two wheels may be read one
 * edge apart, which the 20 ms speed calculation does not notice.
 */

#include "drv_encoder.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../common/drv_gpio_irq.h"

static volatile int32_t g_left_ticks  = 0;
static volatile int32_t g_right_ticks = 0;

/*!
 * @brief GPIO interrupt callback for both encoder A channels.
 */
static void
encoder_isr (uint32_t pin, uint32_t events)
{
    (void) events;

    if (PIN_ENCODER_LEFT_A == pin)
    {
        g_left_ticks += board_gpio_read(PIN_ENCODER_LEFT_B) ? 1 : -1;
    }
    else if (PIN_ENCODER_RIGHT_A == pin)
    {
        g_right_ticks += board_gpio_read(PIN_ENCODER_RIGHT_B) ? -1 : 1;
    }
    else
    {
        /* Not an encoder pin. */
    }
}

/*!
 * @brief Configure encoder inputs (pulled up) and their interrupts.
 *
 * @return true if both interrupts were registered.
 */
bool
encoder_init (void)
{
    bool b_ok = true;

    board_gpio_input(PIN_ENCODER_LEFT_A, BOARD_PULL_UP);
    board_gpio_input(PIN_ENCODER_LEFT_B, BOARD_PULL_UP);
    board_gpio_input(PIN_ENCODER_RIGHT_A, BOARD_PULL_UP);
    board_gpio_input(PIN_ENCODER_RIGHT_B, BOARD_PULL_UP);

    b_ok = gpio_irq_register(PIN_ENCODER_LEFT_A, GPIO_IRQ_EDGE_RISE,
                             encoder_isr) && b_ok;
    b_ok = gpio_irq_register(PIN_ENCODER_RIGHT_A, GPIO_IRQ_EDGE_RISE,
                             encoder_isr) && b_ok;

    return (b_ok);
}

/*!
 * @brief Read both cumulative tick counts.
 */
void
encoder_read (int32_t * p_left, int32_t * p_right)
{
    *p_left  = g_left_ticks;
    *p_right = g_right_ticks;
}

/*!
 * @brief Simulation only: advance the counters from the task.
 */
void
encoder_sim_add (int32_t left_delta, int32_t right_delta)
{
    g_left_ticks  += left_delta;
    g_right_ticks += right_delta;
}

/*** end of file ***/
