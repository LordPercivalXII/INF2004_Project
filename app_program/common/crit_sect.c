/** @file crit_sect.c
 *
 * @brief Cross-core critical section: local interrupt mask (DI) plus a
 *        SIO hardware spinlock.  See crit_sect.h for the rules.
 *
 * Interrupts are masked BEFORE the spinlock is claimed: an interrupt on
 * this core that tried to claim the same (non-recursive) spinlock while
 * the interrupted code held it would spin forever.
 */

#include <tk/tkernel.h>

#include "crit_sect.h"

/* RP2040 datasheet 2.3.1.7: SIO SPINLOCK0..31.  Reading claims the lock
   (non-zero = claimed, zero = held by the other core); writing releases. */
#define SIO_SPINLOCK0_ADDR  (0xD0000100UL)
#define SIO_SPINLOCK_ADDR   (SIO_SPINLOCK0_ADDR + (CRIT_SPINLOCK_ID * 4UL))
#define SIO_SPINLOCK_FREE   (1UL)   /* any value releases */

/*!
 * @brief Enter the critical section.  Callable from tasks and from
 *        interrupt handlers.
 *
 * @return The caller's interrupt state, for crit_exit().
 */
uint32_t
crit_enter (void)
{
    UINT saved_state;

    DI(saved_state);

    /* The other core holds it for a few instructions at most. */
    while (0UL == in_w(SIO_SPINLOCK_ADDR))
    {
        /* Spin. */
    }

    /* Accesses inside the section must not move above the claim. */
    __asm volatile ("dmb" : : : "memory");

    return ((uint32_t) saved_state);
}

/*!
 * @brief Leave the critical section entered by crit_enter().
 *
 * @param[in] saved_state  Value returned by the matching crit_enter().
 */
void
crit_exit (uint32_t saved_state)
{
    /* Accesses inside the section must complete before the release. */
    __asm volatile ("dmb" : : : "memory");
    out_w(SIO_SPINLOCK_ADDR, SIO_SPINLOCK_FREE);

    EI((UINT) saved_state);
}

/*** end of file ***/
