/** @file diag_led.h
 *
 * @brief LED diagnostic indicator for IMU and I2C health.
 *        Blinks to indicate boot/health status when serial console unavailable.
 */

#ifndef DIAG_LED_H
#define DIAG_LED_H

#include <stdint.h>

typedef enum {
    DIAG_LED_BOOT,          /* Steady 1 Hz: booting, initializing I2C/IMU */
    DIAG_LED_I2C_FAIL,      /* 3 fast blinks + pause: I2C/IMU failed */
    DIAG_LED_IMU_HEALTHY,   /* On steady: IMU calibrated and ready */
} diag_led_state_t;

void diag_led_init(void);
void diag_led_set_state(diag_led_state_t state);
void diag_led_poll(uint32_t now_ms);

#endif /* DIAG_LED_H */
