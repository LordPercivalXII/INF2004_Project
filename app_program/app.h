/** @file app.h
 *
 * @brief Definitions shared by the whole team.  Changed only by team
 *        agreement (TEAM.md).
 *
 * Anything that belongs to ONE subsystem goes in that subsystem's own
 * header instead (app_program/subsystemN/subsystemN.h).
 *
 * Coding standard: BARR-C:2018 (Embedded C Coding Standard).  Kernel types
 * (ID, ER, PRI, TMO, ...) are used only where a micro T-Kernel API call
 * needs them; all other data uses the C99 fixed-width types.
 */

#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>     /* micro T-Kernel API (tk_xxx_yyy).          */
#include <tm/tmonitor.h>    /* T-Monitor API: tm_printf() console output. */
#include <bsp/libbsp.h>     /* Board helpers: gpio_set_pin/val, pwm_...   */

/* Stack for every application task, in bytes (tm_printf needs ~1 KB). */
#define APP_TASK_STACK_SIZE     (2048)

/* Core affinity argument for app_start_task(). */
#define APP_CORE_ANY            (0U)

/*
 * Bytes a message buffer needs to hold 'count' messages of 'msg_size'
 * bytes: each message is rounded up to 4 bytes and has a 4-byte header.
 */
#define APP_MBF_BUFSZ(count, msg_size) \
    ((count) * ((((msg_size) + 3U) & ~3U) + 4U))

/* Create and start a task; prc = TP_PRC1, TP_PRC2 or APP_CORE_ANY. */
ID   app_start_task(FP entry, PRI pri, UW prc, char const * p_name);

/*
 * Create and start a task that a cyclic handler wakes every period_ms
 * (replaces FreeRTOS vTaskDelayUntil).  The task body calls
 * tk_slp_tsk(TMO_FEVR) once per period.
 */
ID   app_start_periodic_task(FP entry, PRI pri, UW prc,
                             char const * p_name, uint32_t period_ms);

/* Print a readable message if a kernel call failed (negative code). */
void app_check(int32_t result, char const * p_what);

/*
 * Every subsystem provides these.  usermain() calls ALL the _init()
 * functions first and only then ALL the _start() functions, so every
 * kernel object exists before any task can use it.
 */
ER   subsystem1_init(void);
ER   subsystem2_init(void);
ER   subsystem3_init(void);
ER   subsystem4_init(void);
ER   subsystem5_init(void);
ER   controller_init(void);

ER   subsystem1_start(void);
ER   subsystem2_start(void);
ER   subsystem3_start(void);
ER   subsystem4_start(void);
ER   subsystem5_start(void);
ER   controller_start(void);

#if APP_TEST
void subsystem1_test(void);
void subsystem2_test(void);
void subsystem3_test(void);
void subsystem4_test(void);
void subsystem5_test(void);
#endif /* APP_TEST */

#endif /* APP_H */

/*** end of file ***/
