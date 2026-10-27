/** @file drv_barcode_sensor.h
 *
 * @brief Right IR sensor digital output (GP28): level and edge capture.
 */

#ifndef DRV_BARCODE_SENSOR_H
#define DRV_BARCODE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

bool    barcode_sensor_init(void);
uint8_t barcode_sensor_is_black(void);
bool    barcode_sensor_take_overflow(void);

#endif /* DRV_BARCODE_SENSOR_H */

/*** end of file ***/
