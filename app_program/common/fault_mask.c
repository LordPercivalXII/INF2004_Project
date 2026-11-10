/** @file fault_mask.c
 *
 * @brief Shared 5-bit fault bitmask.  Every RMW of the shared byte is
 *        done inside the cross-core critical section (crit_sect.h).
 */

#include "fault_mask.h"
#include "crit_sect.h"

/* The only copy.  Written only inside crit_enter()/crit_exit(). */
static volatile uint8_t g_fault_mask = 0U;

/*!
 * @brief Latch fault bits.  Callable from tasks and interrupt handlers.
 *
 * @param[in] bits  FAULT_BIT_xxx; bits outside FAULT_MASK_ALL are ignored.
 */
void
fault_mask_set (uint8_t bits)
{
    uint32_t const saved = crit_enter();

    g_fault_mask = (uint8_t) (g_fault_mask | (bits & FAULT_MASK_ALL));
    crit_exit(saved);
}

/*!
 * @brief Clear fault bits.  Callable from tasks and interrupt handlers.
 *
 * @param[in] bits  FAULT_BIT_xxx to clear.
 */
void
fault_mask_clear (uint8_t bits)
{
    uint32_t const saved = crit_enter();

    g_fault_mask = (uint8_t) (g_fault_mask & (uint8_t) ~bits);
    crit_exit(saved);
}

/*!
 * @brief Read the mask without changing it (a single byte load).
 */
uint8_t
fault_mask_read (void)
{
    return (g_fault_mask);
}

/*!
 * @brief Atomically read and clear the requested bits (snapshot).
 *
 * @param[in] bits  FAULT_BIT_xxx to take; FAULT_MASK_ALL for all.
 *
 * @return The requested bits as they were before clearing.
 */
uint8_t
fault_mask_take (uint8_t bits)
{
    uint32_t const saved    = crit_enter();
    uint8_t const  snapshot = (uint8_t) (g_fault_mask & bits);

    g_fault_mask = (uint8_t) (g_fault_mask & (uint8_t) ~bits);
    crit_exit(saved);

    return (snapshot);
}

/*** end of file ***/
