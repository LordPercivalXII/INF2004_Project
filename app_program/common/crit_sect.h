/** @file crit_sect.h
 *
 * @brief Short cross-core critical section for read-modify-write (RMW)
 *        updates of small shared variables on the dual-core RP2040.
 *
 * Why this exists (Tribunal 1, Version C gap 1):
 *   The Cortex-M0+ has no LDREX/STREX, so "x |= bit" on a shared byte is
 *   a non-atomic load, OR and store.  An interrupt on the same core, or a
 *   task or interrupt on the OTHER core, can run between the load and the
 *   store and its update is then lost.
 *
 * micro T-Kernel mapping of the FreeRTOS primitives named in the tribunal:
 *
 *   FreeRTOS (tribunal document)        micro T-Kernel (this module)
 *   ----------------------------------  ---------------------------------
 *   taskENTER_CRITICAL()  (task)        crit_enter(): DI() + SIO spinlock
 *   taskENTER_CRITICAL_FROM_ISR() (ISR) crit_enter(): the same call
 *   taskEXIT_CRITICAL[_FROM_ISR]()      crit_exit(saved)
 *
 * One call serves both contexts: micro T-Kernel's DI()/EI() save and
 * restore PRIMASK, which is legal and nest-safe in a task and in an
 * interrupt handler alike (unlike taskENTER_CRITICAL(), which must not
 * be called from an ISR).  DI() alone only masks the CALLING core, so a
 * SIO hardware spinlock is also taken to exclude the other core.
 *
 * Rules for callers:
 *   - Hold for a few instructions only (a load, a mask, a store).
 *   - Never call a micro T-Kernel service call (tk_xxx) or tm_printf()
 *     while holding the section.
 *   - Do not nest crit_enter() inside another crit_enter(): the spinlock
 *     is not recursive and the core would deadlock against itself.
 */

#ifndef CRIT_SECT_H
#define CRIT_SECT_H

#include <stdint.h>

/*
 * SIO hardware spinlock used by the application.  The kernel reserves
 * spinlocks 0-2 (kernel/sysdepend/cpu/rp2040/smp_atomic.c); 31 is outside
 * every range the kernel or the Pico SDK hands out.
 */
#define CRIT_SPINLOCK_ID    (31U)

uint32_t crit_enter(void);
void     crit_exit(uint32_t saved_state);

#endif /* CRIT_SECT_H */

/*** end of file ***/
