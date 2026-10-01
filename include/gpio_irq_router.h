#ifndef GPIO_IRQ_ROUTER_H
#define GPIO_IRQ_ROUTER_H

#include "hardware/gpio.h"

bool Robot_GpioIrqRegister(uint gpio, uint32_t events, gpio_irq_callback_t callback);

#endif