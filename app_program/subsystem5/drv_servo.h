/** @file drv_servo.h
 *
 * @brief Scan servo on GP15 (50 Hz PWM, 0.5-2.5 ms pulse).
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>

void     servo_init(void);
void     servo_set_angle(uint32_t angle_deg);
uint32_t servo_get_angle(void);

#endif /* DRV_SERVO_H */

/*** end of file ***/
