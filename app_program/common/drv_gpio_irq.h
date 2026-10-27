/** @file drv_gpio_irq.h
 *
 * @brief Shared GPIO interrupt router (port of gpio_irq_router.c).
 *
 * The RP2040 has ONE interrupt line (IO_IRQ_BANK0) for all 30 GPIOs, but
 * three subsystems need GPIO edges: wheel encoders (S2), barcode sensor
 * (S3) and ultrasonic echo (S5).  This router owns that interrupt and
 * calls each pin's registered callback.  Callbacks run in interrupt
 * context: they must be short and must not wait (only TMO_POL kernel
 * calls such as tk_snd_mbf(..., TMO_POL)).
 */

#ifndef DRV_GPIO_IRQ_H
#define DRV_GPIO_IRQ_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#define GPIO_IRQ_LEVEL_LOW      (0x1UL)
#define GPIO_IRQ_LEVEL_HIGH     (0x2UL)
#define GPIO_IRQ_EDGE_FALL      (0x4UL)
#define GPIO_IRQ_EDGE_RISE      (0x8UL)

typedef void (*gpio_irq_callback_t)(uint32_t pin, uint32_t events);

ER   gpio_irq_init(void);
bool gpio_irq_register(uint32_t pin, uint32_t events,
                       gpio_irq_callback_t callback);

#endif /* DRV_GPIO_IRQ_H */

/*** end of file ***/
