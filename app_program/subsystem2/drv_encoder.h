/** @file drv_encoder.h
 *
 * @brief Quadrature wheel encoders, counted on channel A rising edges.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

bool encoder_init(void);
void encoder_read(int32_t * p_left, int32_t * p_right);
void encoder_sim_add(int32_t left_delta, int32_t right_delta);

#endif /* DRV_ENCODER_H */

/*** end of file ***/
