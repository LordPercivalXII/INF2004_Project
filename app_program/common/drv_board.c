/** @file drv_board.c
 *
 * @brief Board helpers: reset release, microsecond timer, GPIO pads, PWM.
 */

#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "drv_board.h"

/* RP2040 atomic register aliases (datasheet 2.1.2). */
#define REG_ALIAS_SET           (0x2000UL)
#define REG_ALIAS_CLR           (0x3000UL)

/* Watchdog tick generator: drives the 1 MHz TIMER from clk_ref. */
#define WATCHDOG_TICK           (0x4005802CUL)
#define WATCHDOG_TICK_ENABLE    (1UL << 9)
#define CLK_REF_MHZ             (12UL)      /* clk_ref = 12 MHz XOSC */

/* PWM slice of a GPIO, and the slice's fractional clock divider. */
#define PWM_SLICE_OF(pin)       (((pin) >> 1) & 0x07UL)
#define PWM_SLICE_DIV(slice)    (PWM_BASE + ((slice) * 0x14UL) + PWM_CHx_DIV)
#define PWM_DIV_INT_SHIFT       (4U)

/*!
 * @brief Bring up the shared hardware the kernel leaves in reset.
 *
 * The TIMER (microsecond clock) and the PWM block are not released by
 * the kernel unless USE_PTMR is set, so it is done once here, before any
 * subsystem starts.
 *
 * @return E_OK.
 */
ER
board_init (void)
{
    board_release_reset(RESETS_RESET_TIMER | RESETS_RESET_PWM);

    if (0UL == (in_w(WATCHDOG_TICK) & WATCHDOG_TICK_ENABLE))
    {
        out_w(WATCHDOG_TICK, (CLK_REF_MHZ | WATCHDOG_TICK_ENABLE));
    }

    return (E_OK);
}

/*!
 * @brief Release peripherals from reset if they are still held.
 *
 * Never asserts a reset, so it cannot disturb a peripheral another task
 * is already using.
 *
 * @param[in] mask  RESETS_RESET_xxx bits.
 */
void
board_release_reset (uint32_t mask)
{
    if (0UL != (in_w(RESETS_RESET) & mask))
    {
        out_w(RESETS_RESET + REG_ALIAS_CLR, mask);

        while (mask != (in_w(RESETS_RESET_DONE) & mask))
        {
            /* Wait for the peripheral to come out of reset. */
        }
    }
}

/*!
 * @brief Milliseconds since the kernel started (monotonic).
 */
uint32_t
board_millis (void)
{
    SYSTIM  now;

    (void) tk_get_otm(&now);

    return ((uint32_t) now.lo);
}

/*!
 * @brief Microseconds from the RP2040 TIMER.  Callable from interrupts.
 */
uint32_t
board_micros (void)
{
    return ((uint32_t) in_w(TIMER_TIMERAWL));
}

/*!
 * @brief Busy-wait for a short time.  Use only for a few microseconds.
 *
 * @param[in] delay_us  Time to wait, microseconds.
 */
void
board_delay_us (uint32_t delay_us)
{
    uint32_t const start_us = board_micros();

    while ((board_micros() - start_us) < delay_us)
    {
        /* Spin. */
    }
}

/*!
 * @brief Make a pin a digital input with the given pull resistor.
 */
void
board_gpio_input (uint32_t pin, board_pull_t pull)
{
    (void) gpio_set_pin(pin, GPIO_MODE_IN);
    out_w(GPIO(pin) + REG_ALIAS_CLR, (GPIO_PUE | GPIO_PDE));

    if (BOARD_PULL_UP == pull)
    {
        out_w(GPIO(pin) + REG_ALIAS_SET, GPIO_PUE);
    }
    else if (BOARD_PULL_DOWN == pull)
    {
        out_w(GPIO(pin) + REG_ALIAS_SET, GPIO_PDE);
    }
    else
    {
        /* BOARD_PULL_NONE: the external module drives the line. */
    }
}

/*!
 * @brief Make a pin a digital output, setting its level first.
 */
void
board_gpio_output (uint32_t pin, bool b_level)
{
    board_gpio_write(pin, b_level);
    (void) gpio_set_pin(pin, GPIO_MODE_OUT);
}

/*!
 * @brief Read a digital input.  Callable from interrupts.
 */
bool
board_gpio_read (uint32_t pin)
{
    return (0U != gpio_get_val(pin));
}

/*!
 * @brief Drive a digital output.
 */
void
board_gpio_write (uint32_t pin, bool b_level)
{
    (void) gpio_set_val(pin, (b_level ? 1U : 0U));
}

/*!
 * @brief Prepare an ADC pin: no digital function, input buffer and pulls
 *        off (what the SDK's adc_gpio_init() does).
 */
void
board_gpio_analog (uint32_t pin)
{
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
    out_w(GPIO(pin) + REG_ALIAS_CLR, (GPIO_IE | GPIO_PUE | GPIO_PDE));
}

/*!
 * @brief Route a pin to its PWM slice and start the slice.
 *
 * @param[in] pin     GPIO number.
 * @param[in] clkdiv  Integer clock divider from 125 MHz (1..255).
 * @param[in] wrap    Counter TOP value: period = (wrap + 1) counts.
 */
void
board_pwm_init (uint32_t pin, uint32_t clkdiv, uint32_t wrap)
{
    uint32_t const slice = PWM_SLICE_OF(pin);

    (void) pwm_set_pin(pin);
    out_w(PWM_SLICE_DIV(slice), (clkdiv << PWM_DIV_INT_SHIFT));
    (void) pwm_set_wrap(pin, wrap);
    (void) pwm_set_cc(pin, 0U);
    (void) pwm_set_enabled(pin, TRUE);
}

/*!
 * @brief Set a PWM output's compare level (0..wrap+1).
 */
void
board_pwm_set_level (uint32_t pin, uint32_t level)
{
    (void) pwm_set_cc(pin, level);
}

/*!
 * @brief Data memory barrier: orders memory accesses between an
 *        interrupt handler and a task, also across the two cores.
 */
void
board_memory_barrier (void)
{
    __asm volatile ("dmb" : : : "memory");
}

/*** end of file ***/
