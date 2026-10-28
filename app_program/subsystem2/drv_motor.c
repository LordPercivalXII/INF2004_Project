/** @file drv_motor.c
 *
 * @brief Motor driver.  Called only from motion_task (owning task).
 *
 * PWM: 125 MHz / 1 / 65536 = ~1.9 kHz, same as the FreeRTOS version.
 */

#include "drv_motor.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../common/rmath.h"

#define MOTOR_PWM_CLKDIV    (1U)
#define MOTOR_PWM_WRAP      (65535U)
#define MOTOR_PWM_FULL      (65535.0f)

/*!
 * @brief Configure both H-bridge inputs of both motors; motors stopped.
 *
 * Driver truth table: A high/B low = forward, A low/B high = backward,
 * both low = brake.  Both inputs are PWM-capable (GP8/GP9, GP10/GP11).
 */
void
motor_init (void)
{
    board_pwm_init(PIN_MOTOR_LEFT_PWM, MOTOR_PWM_CLKDIV, MOTOR_PWM_WRAP);
    board_pwm_init(PIN_MOTOR_LEFT_DIR, MOTOR_PWM_CLKDIV, MOTOR_PWM_WRAP);
    board_pwm_init(PIN_MOTOR_RIGHT_PWM, MOTOR_PWM_CLKDIV, MOTOR_PWM_WRAP);
    board_pwm_init(PIN_MOTOR_RIGHT_DIR, MOTOR_PWM_CLKDIV, MOTOR_PWM_WRAP);
}

/*!
 * @brief Drive one motor.
 *
 * @param[in] pin_a  Input A of the motor (forward PWM).
 * @param[in] pin_b  Input B of the motor (reverse PWM).
 * @param[in] duty   -1.0 (full reverse) .. +1.0 (full forward).
 */
void
motor_set (uint32_t pin_a, uint32_t pin_b, float duty)
{
    float const sign = (PIN_MOTOR_RIGHT_DIR == pin_b) ?
                       MOTOR_RIGHT_DIR_SIGN : MOTOR_LEFT_DIR_SIGN;
    float const scaled = duty * sign;
    float const lifted = (rmath_absf(scaled) < 0.01f) ? 0.0f :
        ((scaled > 0.0f) ? (MOTOR_MIN_DUTY + (scaled * (1.0f - MOTOR_MIN_DUTY)))
                         : (-MOTOR_MIN_DUTY + (scaled * (1.0f - MOTOR_MIN_DUTY))));
    float const bounded = rmath_clampf(lifted, -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);
    uint32_t const level = (uint32_t) (rmath_absf(bounded) * MOTOR_PWM_FULL);

    board_pwm_set_level(pin_a, (bounded >= 0.0f) ? level : 0U);
    board_pwm_set_level(pin_b, (bounded >= 0.0f) ? 0U : level);
}

/*** end of file ***/
