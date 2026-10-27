/** @file drv_board.h
 *
 * @brief Board helpers shared by every subsystem: peripheral reset
 *        release, microsecond time, GPIO pad set-up and PWM slices.
 *
 * Replaces the Pico SDK calls the FreeRTOS project used (gpio_init,
 * gpio_pull_up, time_us_32, busy_wait_us_32, pwm_set_clkdiv, ...).  The
 * micro T-Kernel template does not link the SDK hardware libraries, so
 * these are thin register-level wrappers around libbsp.
 *
 * Ownership rule (TEAM.md section 3): each pin is configured only by the
 * task that owns it.  These functions keep no state and touch only the
 * registers of the pin they are given, using the RP2040 atomic SET/CLR
 * register aliases, so they are safe to call from tasks on either core.
 */

#ifndef DRV_BOARD_H
#define DRV_BOARD_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    BOARD_PULL_NONE = 0,
    BOARD_PULL_UP,
    BOARD_PULL_DOWN
} board_pull_t;

ER       board_init(void);
void     board_release_reset(uint32_t mask);

uint32_t board_millis(void);
uint32_t board_micros(void);
void     board_delay_us(uint32_t delay_us);

void     board_gpio_input(uint32_t pin, board_pull_t pull);
void     board_gpio_output(uint32_t pin, bool b_level);
bool     board_gpio_read(uint32_t pin);
void     board_gpio_write(uint32_t pin, bool b_level);
void     board_gpio_analog(uint32_t pin);

void     board_pwm_init(uint32_t pin, uint32_t clkdiv, uint32_t wrap);
void     board_pwm_set_level(uint32_t pin, uint32_t level);

void     board_memory_barrier(void);

#endif /* DRV_BOARD_H */

/*** end of file ***/
