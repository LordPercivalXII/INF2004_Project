#include <stdbool.h>
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "FreeRTOS.h"
#include "gpio_irq_router.h"

typedef struct {
    gpio_irq_callback_t callback;
    uint32_t events;
} GpioIrqRoute;

static GpioIrqRoute routes[NUM_BANK0_GPIOS];
static bool router_installed;

static void gpio_irq_dispatch(uint gpio, uint32_t events)
{
    if (gpio < NUM_BANK0_GPIOS && routes[gpio].callback != NULL &&
        (routes[gpio].events & events) != 0U) {
        routes[gpio].callback(gpio, events);
    }
}

bool Robot_GpioIrqRegister(uint gpio, uint32_t events, gpio_irq_callback_t callback)
{
    if (gpio >= NUM_BANK0_GPIOS || callback == NULL || events == 0U) return false;
    const uint32_t irq_state = save_and_disable_interrupts();
    if (routes[gpio].callback != NULL && routes[gpio].callback != callback) {
        restore_interrupts(irq_state);
        return false;
    }
    routes[gpio].callback = callback;
    routes[gpio].events |= events;
    if (!router_installed) {
        gpio_set_irq_callback(gpio_irq_dispatch);
        irq_set_priority(IO_IRQ_BANK0, (uint8_t)configMAX_SYSCALL_INTERRUPT_PRIORITY);
        irq_set_enabled(IO_IRQ_BANK0, true);
        router_installed = true;
    }
    gpio_set_irq_enabled(gpio, events, true);
    restore_interrupts(irq_state);
    return true;
}