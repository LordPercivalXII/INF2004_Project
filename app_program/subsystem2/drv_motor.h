/** @file drv_motor.h
 *
 * @brief Two DC motors on the Robo Pico driver: PWM speed + direction pin.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>

void motor_init(void);
void motor_set(uint32_t pwm_pin, uint32_t dir_pin, float duty);

#endif /* DRV_MOTOR_H */

/*** end of file ***/
