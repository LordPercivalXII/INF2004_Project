/** @file drv_ir_sampler.h
 *
 * @brief 1 kHz IR sampler: TIMER alarm interrupt -> lock-free ring.
 */

#ifndef DRV_IR_SAMPLER_H
#define DRV_IR_SAMPLER_H

#include <stdint.h>
#include <tk/tkernel.h>

typedef struct
{
    uint32_t count;
    uint32_t left_sum;
    uint32_t center_sum;
    uint32_t right_black_count;
} ir_window_t;

ER       ir_sampler_start(void);
void     ir_sampler_drain(ir_window_t * p_window);
uint32_t ir_sampler_overruns(void);

#endif /* DRV_IR_SAMPLER_H */

/*** end of file ***/
