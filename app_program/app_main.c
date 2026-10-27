/** @file app_main.c
 *
 * @brief START HERE.  usermain() is the program's entry point.
 *
 * Port of the FreeRTOS main.c of the INF2004 autonomous robot car.
 *
 *   FreeRTOS                          micro T-Kernel 3.0 (this file)
 *   --------------------------------  ---------------------------------
 *   main() + vTaskStartScheduler()    usermain() in the initial task
 *   xQueueCreate / xEventGroupCreate  each owner's *_init() (tk_cre_*)
 *   xTaskCreate                       each owner's *_start()
 *   vApplicationMallocFailedHook      kernel object IDs checked (< E_OK)
 *
 * Start-up sequence:
 *   1. board_init() and gpio_irq_init(): shared hardware (TIMER, the
 *      IO_BANK0 GPIO interrupt) used by more than one subsystem.
 *   2. every module creates its kernel objects   (*_init)
 *   3. every module creates and starts its tasks (*_start)
 *   4. the initial task sleeps forever.  If usermain() returned, the
 *      kernel would shut the whole system down.
 *
 * Modules (one folder each, see TEAM.md):
 *   controller/  Vehicle controller: sole owner of the mission state
 *   subsystem1/  Buddy 1: telemetry (JSON lines on the console)
 *   subsystem2/  Buddy 2: motors, wheel encoders, speed PID
 *   subsystem3/  Buddy 3: IR line tracking + Code 39 barcodes
 *   subsystem4/  Buddy 4: MPU6050 IMU (shock, hump, motion state)
 *   subsystem5/  Buddy 5: ultrasonic obstacle scanner on a servo
 *   common/      shared types, configuration and board drivers
 */

#include "app.h"
#include "common/robot_config.h"
#include "common/drv_board.h"
#include "common/drv_gpio_irq.h"

/*!
 * @brief Create a task and start it.
 *
 * @param[in] entry   Task function: void f(INT stacd, void * p_exinf).
 * @param[in] pri     Initial priority (1 = highest).
 * @param[in] prc     TP_PRC1 (core 0), TP_PRC2 (core 1) or APP_CORE_ANY.
 * @param[in] p_name  Name used in error messages.
 *
 * @return The task ID, or a negative error code.
 */
ID
app_start_task (FP entry, PRI pri, UW prc, char const * p_name)
{
    T_CTSK  ctsk = {0};
    ID      tid  = E_OK;
    ER      er   = E_OK;

    ctsk.task    = entry;
    ctsk.itskpri = pri;
    ctsk.stksz   = APP_TASK_STACK_SIZE;
    ctsk.tskatr  = (TA_HLNG | TA_RNG3);

#if TK_SUPPORT_SMP
    if (APP_CORE_ANY != prc)
    {
        ctsk.tskatr |= TA_ASSPRC;
        ctsk.assprc  = prc;
    }
#else
    (void) prc;
#endif

    tid = tk_cre_tsk(&ctsk);

    if (tid < E_OK)
    {
        app_check(tid, p_name);
    }
    else
    {
        er = tk_sta_tsk(tid, 0);
        app_check(er, p_name);

        if (er < E_OK)
        {
            tid = er;
        }
    }

    return (tid);
}

/*!
 * @brief Cyclic handler shared by every periodic task: wakes the task
 *        whose ID is the handler's extended information.  Runs in
 *        interrupt context, so it only signals.
 *
 * @param[in] p_exinf  Task ID, stored as a pointer.
 */
static void
app_periodic_handler (void * p_exinf)
{
    (void) tk_wup_tsk((ID) (intptr_t) p_exinf);
}

/*!
 * @brief Create and start a task paced by its own cyclic handler.
 *
 * The cyclic handler keeps an exact period however long one cycle of
 * the task takes, like vTaskDelayUntil() did.  If the task overruns,
 * wake-ups queue up and it catches up, also like vTaskDelayUntil().
 *
 * @param[in] entry      Task function.
 * @param[in] pri        Initial priority.
 * @param[in] prc        Core affinity.
 * @param[in] p_name     Name used in error messages.
 * @param[in] period_ms  Period, ms (multiple of the 10 ms tick).
 *
 * @return The task ID, or a negative error code.
 */
ID
app_start_periodic_task (FP entry, PRI pri, UW prc, char const * p_name,
                         uint32_t period_ms)
{
    T_CCYC  ccyc = {0};
    ID      tid  = app_start_task(entry, pri, prc, p_name);
    ID      cid  = E_OK;

    if (tid >= E_OK)
    {
        ccyc.exinf  = (void *) (intptr_t) tid;
        ccyc.cycatr = (TA_HLNG | TA_STA);
        ccyc.cychdr = (FP) app_periodic_handler;
        ccyc.cyctim = (RELTIM) period_ms;
        ccyc.cycphs = (RELTIM) period_ms;
        cid = tk_cre_cyc(&ccyc);
        app_check(cid, p_name);

        if (cid < E_OK)
        {
            tid = cid;
        }
    }

    return (tid);
}

/*!
 * @brief Print a readable message if a kernel call failed.
 *
 * @param[in] result  Return value of the kernel call.
 * @param[in] p_what  Description of the call.
 */
void
app_check (int32_t result, char const * p_what)
{
    if (result < E_OK)
    {
        tm_printf((UB *) "ERROR %s: %d\n", p_what, (int) result);
    }
}

/*!
 * @brief Application entry point, called by the kernel's initial task.
 *
 * @return Never returns in normal operation.
 */
INT
usermain (void)
{
    T_RVER  ver;

    (void) tk_ref_ver(&ver);
    tm_printf((UB *) "\n=== INF2004 autonomous car on micro T-Kernel"
              " (spec 0x%04x) simulation=%d ===\n",
              ver.spver, ROBOT_SIMULATION);

    /* Phase 0: shared hardware used by more than one subsystem. */
    app_check(board_init(), "board_init");
    app_check(gpio_irq_init(), "gpio_irq_init");

    /* Phase 1: every module creates its kernel objects. */
    app_check(controller_init(), "controller_init");
    app_check(subsystem1_init(), "subsystem1_init");
    app_check(subsystem2_init(), "subsystem2_init");
    app_check(subsystem3_init(), "subsystem3_init");
    app_check(subsystem4_init(), "subsystem4_init");
    app_check(subsystem5_init(), "subsystem5_init");

    /* Phase 2: every module starts its tasks. */
    app_check(subsystem1_start(), "subsystem1_start");
    app_check(subsystem2_start(), "subsystem2_start");
    app_check(subsystem3_start(), "subsystem3_start");
    app_check(subsystem4_start(), "subsystem4_start");
    app_check(subsystem5_start(), "subsystem5_start");
    app_check(controller_start(), "controller_start");

    tm_printf((UB *) "all tasks started\n\n");

#if APP_TEST
    tm_printf((UB *) "[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
    tm_printf((UB *) "[TEST] === test run end ===\n");
#endif /* APP_TEST */

    /* Sleep forever: returning from usermain() shuts the kernel down. */
    (void) tk_slp_tsk(TMO_FEVR);

    return (0);
}

/*** end of file ***/
