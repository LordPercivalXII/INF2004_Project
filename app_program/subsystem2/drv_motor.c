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
 * @brief Configure both PWM outputs and direction pins; motors stopped.
 */
void
motor_init (void)
{
    board_pwm_init(PIN_MOTOR_LEFT_PWM, MOTOR_PWM_CLKDIV, MOTOR_PWM_WRAP);
    board_pwm_init(PIN_MOTOR_RIGHT_PWM, MOTOR_PWM_CLKDIV, MOTOR_PWM_WRAP);
    board_gpio_output(PIN_MOTOR_LEFT_DIR, false);
    board_gpio_output(PIN_MOTOR_RIGHT_DIR, false);
}

/*!
 * @brief Drive one motor.
 *
 * @param[in] pwm_pin  PWM pin of the motor.
 * @param[in] dir_pin  Direction pin of the motor.
 * @param[in] duty     -1.0 (full reverse) .. +1.0 (full forward).
 */
void
motor_set (uint32_t pwm_pin, uint32_t dir_pin, float duty)
{
    float const bounded = rmath_clampf(duty, -1.0f, 1.0f);

    board_gpio_write(dir_pin, (bounded >= 0.0f));
    board_pwm_set_level(pwm_pin,
                        (uint32_t) (rmath_absf(bounded) * MOTOR_PWM_FULL));
}

/*** end of file ***/
