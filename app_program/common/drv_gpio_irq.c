/** @file drv_gpio_irq.c
 *
 * @brief Shared GPIO interrupt router on IO_IRQ_BANK0.
 *
 * gpio_irq_init() is called once from usermain() before any subsystem
 * starts.  It installs the handler with tk_def_int() and enables the
 * interrupt in the NVIC of the core it runs on; that core is remembered
 * so every later registration enables the pin in the SAME core's
 * PROCx_INTE register, whichever core the registering task runs on.
 */

#include <tk/tkernel.h>

#include "drv_gpio_irq.h"

#define IO_IRQ_BANK0            (13U)
#define GPIO_IRQ_PRIORITY       (2)

#define IO_INTR0                (IO_BANK0_BASE + 0x0F0UL)
#define IO_PROC0_INTE0          (IO_BANK0_BASE + 0x100UL)
#define IO_PROC0_INTS0          (IO_BANK0_BASE + 0x120UL)
#define IO_PROC1_INTE0          (IO_BANK0_BASE + 0x130UL)
#define IO_PROC1_INTS0          (IO_BANK0_BASE + 0x150UL)
#define IO_REG_ALIAS_SET        (0x2000UL)

#define GPIO_IRQ_REG_COUNT      (4U)        /* 8 GPIOs per register */
#define GPIO_PER_REG            (8U)
#define GPIO_IRQ_EVENT_MASK     (0xFUL)
#define GPIO_IRQ_EDGE_MASK      (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)

#define SIO_CPUID               (0xD0000000UL)

typedef struct
{
    gpio_irq_callback_t callback;
    uint32_t            events;
} gpio_irq_route_t;

static gpio_irq_route_t g_routes[GPIO_NUM];
static uint32_t         g_inte_base = IO_PROC0_INTE0;
static uint32_t         g_ints_base = IO_PROC0_INTS0;
static bool             gb_installed = false;

/*!
 * @brief IO_IRQ_BANK0 handler: clear and dispatch every pending pin.
 *
 * @param[in] intno  Interrupt number (IO_IRQ_BANK0).
 */
static void
gpio_irq_isr (UINT intno)
{
    uint32_t reg;
    uint32_t slot;

    for (reg = 0U; reg < GPIO_IRQ_REG_COUNT; reg++)
    {
        uint32_t const status = in_w(g_ints_base + (reg * 4U));

        for (slot = 0U; slot < GPIO_PER_REG; slot++)
        {
            uint32_t const pin    = (reg * GPIO_PER_REG) + slot;
            uint32_t const events =
                (status >> (slot * 4U)) & GPIO_IRQ_EVENT_MASK;

            if ((0UL != events) && (pin < GPIO_NUM))
            {
                /* Edge bits are write-1-to-clear; level bits clear
                   themselves when the level goes away. */
                out_w(IO_INTR0 + (reg * 4U),
                      (events & GPIO_IRQ_EDGE_MASK) << (slot * 4U));

                if ((NULL != g_routes[pin].callback) &&
                    (0UL != (g_routes[pin].events & events)))
                {
                    g_routes[pin].callback(pin, events);
                }
            }
        }
    }

    ClearInt(intno);
}

/*!
 * @brief Install the router.  Call once, from usermain().
 *
 * @return E_OK, or the tk_def_int() error.
 */
ER
gpio_irq_init (void)
{
    T_DINT  dint = {0};
    ER      er   = E_OK;

    if (!gb_installed)
    {
        if (0UL != in_w(SIO_CPUID))
        {
            g_inte_base = IO_PROC1_INTE0;
            g_ints_base = IO_PROC1_INTS0;
        }

        dint.intatr = TA_HLNG;
        dint.inthdr = (FP) gpio_irq_isr;
        er = tk_def_int(IO_IRQ_BANK0, &dint);

        if (E_OK == er)
        {
            EnableInt(IO_IRQ_BANK0, GPIO_IRQ_PRIORITY);
            gb_installed = true;
        }
    }

    return (er);
}

/*!
 * @brief Route a pin's interrupt events to a callback and enable them.
 *
 * @param[in] pin       GPIO number.
 * @param[in] events    GPIO_IRQ_xxx bits.
 * @param[in] callback  Called in interrupt context with (pin, events).
 *
 * @return true on success; false if the router is not installed, the
 *         arguments are bad, or another callback already owns the pin.
 */
bool
gpio_irq_register (uint32_t pin, uint32_t events,
                   gpio_irq_callback_t callback)
{
    bool b_ok = false;

    if (gb_installed && (pin < GPIO_NUM) && (NULL != callback) &&
        (0UL != (events & GPIO_IRQ_EVENT_MASK)))
    {
        if ((NULL == g_routes[pin].callback) ||
            (callback == g_routes[pin].callback))
        {
            uint32_t const reg  = pin / GPIO_PER_REG;
            uint32_t const slot = pin % GPIO_PER_REG;

            g_routes[pin].events  |= events;
            g_routes[pin].callback = callback;
            __asm volatile ("dmb" : : : "memory");

            /* Discard stale edges, then enable (atomic SET alias). */
            out_w(IO_INTR0 + (reg * 4U),
                  (GPIO_IRQ_EDGE_MASK << (slot * 4U)));
            out_w(g_inte_base + (reg * 4U) + IO_REG_ALIAS_SET,
                  ((events & GPIO_IRQ_EVENT_MASK) << (slot * 4U)));
            b_ok = true;
        }
    }

    return (b_ok);
}

/*** end of file ***/
