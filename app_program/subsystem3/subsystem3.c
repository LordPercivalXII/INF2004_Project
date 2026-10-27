/** @file subsystem3.c
 *
 * @brief Subsystem 3 -- line + barcode: creates objects, starts tasks.
 *
 *   FreeRTOS                               micro T-Kernel
 *   -------------------------------------  ------------------------------
 *   s_edge_queue   (xQueueCreate, 128)     gh_barcode_edge_mbf (tk_cre_mbf)
 *   s_result_queue (length 1, overwrite)   result slot + gh_barcode_result_mtx
 */

#include "subsystem3.h"
#include "../common/robot_config.h"

ID gh_barcode_edge_mbf   = E_ID;
ID gh_barcode_result_mtx = E_ID;

ER
subsystem3_init (void)
{
    T_CMBF  cmbf = {0};
    T_CMTX  cmtx = {0};
    ER      er   = E_OK;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.maxmsz = (INT) sizeof(barcode_element_t);
    cmbf.bufsz  = (SZ) APP_MBF_BUFSZ(BARCODE_EDGE_QUEUE_LENGTH,
                                     sizeof(barcode_element_t));
    gh_barcode_edge_mbf = tk_cre_mbf(&cmbf);

    cmtx.mtxatr = TA_INHERIT;
    gh_barcode_result_mtx = tk_cre_mtx(&cmtx);

    if (gh_barcode_edge_mbf < E_OK)
    {
        er = gh_barcode_edge_mbf;
    }
    else if (gh_barcode_result_mtx < E_OK)
    {
        er = gh_barcode_result_mtx;
    }
    else
    {
        er = E_OK;
    }

    return (er);
}

ER
subsystem3_start (void)
{
    ID tid = app_start_periodic_task((FP) line_task, S3_PRI_LINE,
                                     APP_CORE_ANY, "line_task",
                                     ROBOT_LINE_PERIOD_MS);

    if (tid >= E_OK)
    {
        tid = app_start_task((FP) barcode_task, S3_PRI_BARCODE,
                             APP_CORE_ANY, "barcode_task");
    }

    return ((tid < E_OK) ? tid : E_OK);
}

/*** end of file ***/
