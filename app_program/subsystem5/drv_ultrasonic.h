/** @file drv_ultrasonic.h
 *
 * @brief HC-SR04 style ultrasonic sensor: trigger GP17, echo GP16.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

bool     ultrasonic_init(void);
float    ultrasonic_measure_cm(uint32_t scan_number);
uint32_t ultrasonic_take_drops(void);

#endif /* DRV_ULTRASONIC_H */

/*** end of file ***/
