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
 * Rising edges closer than MOTOR_ENCODER_MIN_EDGE_US to the last accepted
 * edge on that wheel are counted as rejected duplicates, not odometry.
 *
 * Direction glitches: the direction of each tick is channel B sampled on
 * an A rising edge.  A glitch on A's FALLING edge (noise, a slow edge)
 * also looks like a rising edge, but B is then in its opposite state, so
 * it counts one tick BACKWARDS.  These glitches come about half a period
 * after a real edge, so the short-edge filter cannot catch them.  Each
 * one cancels a real tick: the wheel reads slow, its speed PID drives it
 * harder (the car drifts to that wheel's opposite side) and, if every
 * falling edge glitches, the net count stops moving and motion_task
 * reports an encoder stall while the wheel is turning.
 *
 * A wheel cannot reverse within a few ticks, so a tick against the
 * current direction is held back.  ENCODER_REVERSE_CONFIRM opposite
 * ticks in a row are a real reversal: they are then all counted.  A
 * same-direction tick first means they were glitches: they are dropped
 * and counted in the "reversed glitches" diagnostic.
 */

#include "drv_encoder.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../common/drv_gpio_irq.h"

static volatile int32_t g_left_ticks  = 0;
static volatile int32_t g_right_ticks = 0;
static uint32_t g_last_left_us;
static uint32_t g_last_right_us;
static bool g_have_left_edge;
static bool g_have_right_edge;
static volatile uint32_t g_left_rejected;
static volatile uint32_t g_right_rejected;

/* Opposite-direction ticks needed to accept a reversal (~0.3 mm each). */
#define ENCODER_REVERSE_CONFIRM (3U)

/* Per-wheel direction filter state.  Written only by encoder_isr. */
typedef struct
{
    int32_t     direction;      /* +1, -1, or 0 before the first tick   */
    uint32_t    reverse_run;    /* opposite ticks held back             */
} encoder_dir_t;

static encoder_dir_t     g_left_dir;
static encoder_dir_t     g_right_dir;
static volatile uint32_t g_left_reversed;   /* glitch ticks dropped     */
static volatile uint32_t g_right_reversed;

/*!
 * @brief Apply one tick through the direction filter (ISR only).
 *
 * @param[in,out] p_ticks     Wheel's cumulative count.
 * @param[in,out] p_dir       Wheel's direction filter.
 * @param[in,out] p_reversed  Wheel's dropped-glitch counter.
 * @param[in]     step        +1 or -1 from channel B.
 */
static void
encoder_count (volatile int32_t * p_ticks, encoder_dir_t * p_dir,
               volatile uint32_t * p_reversed, int32_t step)
{
    if ((0 == p_dir->direction) || (step == p_dir->direction))
    {
        /* Ticks held back before this one were glitches. */
        *p_reversed        += p_dir->reverse_run;
        p_dir->reverse_run  = 0U;
        p_dir->direction    = step;
        *p_ticks           += step;
    }
    else
    {
        p_dir->reverse_run++;

        if (p_dir->reverse_run >= ENCODER_REVERSE_CONFIRM)
        {
            /* A real reversal: count every tick that was held back. */
            *p_ticks           += step * (int32_t) p_dir->reverse_run;
            p_dir->direction    = step;
            p_dir->reverse_run  = 0U;
        }
    }
}

/*!
 * @brief GPIO interrupt callback for both encoder A channels.
 */
static void
encoder_isr (uint32_t pin, uint32_t events)
{
    (void) events;

    if (PIN_ENCODER_LEFT_A == pin)
    {
        uint32_t const now_us = board_micros();
        if (g_have_left_edge &&
            ((now_us - g_last_left_us) < MOTOR_ENCODER_MIN_EDGE_US))
        {
            g_left_rejected++;
            return;
        }
        g_have_left_edge = true;
        g_last_left_us = now_us;
        encoder_count(&g_left_ticks, &g_left_dir, &g_left_reversed,
                      board_gpio_read(PIN_ENCODER_LEFT_B) ? 1 : -1);
    }
    else if (PIN_ENCODER_RIGHT_A == pin)
    {
        uint32_t const now_us = board_micros();
        if (g_have_right_edge &&
            ((now_us - g_last_right_us) < MOTOR_ENCODER_MIN_EDGE_US))
        {
            g_right_rejected++;
            return;
        }
        g_have_right_edge = true;
        g_last_right_us = now_us;
        encoder_count(&g_right_ticks, &g_right_dir, &g_right_reversed,
                      board_gpio_read(PIN_ENCODER_RIGHT_B) ? -1 : 1);
    }
    else
    {
        /* Not an encoder pin. */
    }
}

void
encoder_read_rejected (uint32_t * p_left, uint32_t * p_right)
{
    *p_left = g_left_rejected;
    *p_right = g_right_rejected;
}

/*!
 * @brief Cumulative reversed-direction glitch ticks dropped per wheel.
 */
void
encoder_read_reversed (uint32_t * p_left, uint32_t * p_right)
{
    *p_left  = g_left_reversed;
    *p_right = g_right_reversed;
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
