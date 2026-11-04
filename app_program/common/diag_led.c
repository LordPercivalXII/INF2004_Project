/** @file diag_led.c
 *
 * @brief LED diagnostic: shows I2C/IMU health without serial console.
 *        GPIO 25 (Pico onboard LED).
 *
 *        BOOT:       1 Hz steady blink (500ms on, 500ms off)
 *        I2C_FAIL:   pattern of 3 fast blinks (100ms on, 100ms off) + 600ms pause
 *        IMU_HEALTHY: LED on steady (100% duty)
 */

#include "diag_led.h"
#include "drv_board.h"

#define DIAG_LED_PIN        25

static diag_led_state_t g_state = DIAG_LED_BOOT;
static uint32_t g_last_toggle_ms = 0;

void diag_led_init(void)
{
    board_gpio_output(DIAG_LED_PIN, false);
}

void diag_led_set_state(diag_led_state_t state)
{
    g_state = state;
    g_last_toggle_ms = 0;
    
    /* Set initial LED state */
    switch (state) {
        case DIAG_LED_BOOT:
        case DIAG_LED_I2C_FAIL:
            board_gpio_write(DIAG_LED_PIN, false);
            break;
        case DIAG_LED_IMU_HEALTHY:
            board_gpio_write(DIAG_LED_PIN, true);
            break;
    }
}

void diag_led_poll(uint32_t now_ms)
{
    static uint32_t blink_phase = 0;
    uint32_t elapsed;
    bool led_state;
    
    if (g_last_toggle_ms == 0) {
        g_last_toggle_ms = now_ms;
    }
    
    elapsed = now_ms - g_last_toggle_ms;
    
    switch (g_state) {
        case DIAG_LED_BOOT:
            /* 1 Hz blink: 500ms on, 500ms off */
            if (elapsed >= 500) {
                led_state = board_gpio_read(DIAG_LED_PIN);
                board_gpio_write(DIAG_LED_PIN, !led_state);
                g_last_toggle_ms = now_ms;
            }
            break;
            
        case DIAG_LED_I2C_FAIL:
            /* 3 fast blinks (100ms) + 600ms pause = 1 second cycle
               Pattern: ON 100ms, OFF 100ms, ON 100ms, OFF 100ms, ON 100ms, OFF 600ms */
            if (elapsed >= 100 && blink_phase < 5) {
                /* First 5 phases: 100ms each */
                led_state = (blink_phase % 2) == 0;
                board_gpio_write(DIAG_LED_PIN, led_state);
                blink_phase++;
                g_last_toggle_ms = now_ms;
            } else if (elapsed >= 600 && blink_phase == 5) {
                /* Final pause: 600ms off */
                board_gpio_write(DIAG_LED_PIN, false);
                blink_phase = 0;
                g_last_toggle_ms = now_ms;
            }
            break;
            
        case DIAG_LED_IMU_HEALTHY:
            /* LED stays on steady */
            board_gpio_write(DIAG_LED_PIN, true);
            break;
    }
}
