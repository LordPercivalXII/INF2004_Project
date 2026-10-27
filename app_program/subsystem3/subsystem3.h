/** @file subsystem3.h
 *
 * @brief Subsystem 3 -- Buddy 3: IR line tracking + Code 39 barcode.
 *        Student ID: <student ID>.  Port of src/buddy3_line_barcode/.
 *
 * Tasks:
 *   line_task     periodic 10 ms (NFR6): averages the 1 kHz IR samples,
 *                 calibrates at boot (FR7), computes the line error,
 *                 detects junctions, posts SYSTEM_EVENT_LINE (IF-01).
 *   barcode_task  event-driven (FR8): debounces GP28 edges, collects
 *                 bar/space widths, decodes Code 39, hands the result
 *                 to line_task (FR9), which sends it with the next
 *                 line event.
 *
 * Interrupts:
 *   TIMER alarm 3 at 1 kHz (drv_ir_sampler.c): ADC0, ADC1 + GP28 level.
 *   GP28 edges via the shared GPIO router (drv_barcode_sensor.c).
 *
 * Owns: the ADC (the kernel "adca" driver is disabled), TIMER alarm 3,
 *       GP26, GP27, GP28.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "../common/robot_types.h"

#define S3_PRI_LINE         (14)    /* band 14-15 (TEAM.md) */
#define S3_PRI_BARCODE      (15)

/* One finished bar or space, from the edge interrupt to barcode_task. */
typedef struct
{
    uint32_t width_us;      /* duration of the element that just ended */
    uint32_t end_us;        /* timestamp of the edge that ended it     */
    uint8_t  is_black;      /* 1 = bar, 0 = space                      */
} barcode_element_t;

/* One decoded barcode, from barcode_task to line_task. */
typedef struct
{
    uint16_t command;                       /* barcode_command_t */
    char     text[BARCODE_TEXT_MAX + 1U];
} barcode_result_t;

/* Private to subsystem 3: kernel objects created in subsystem3_init(). */
extern ID gh_barcode_edge_mbf;      /* ISR -> barcode_task              */
extern ID gh_barcode_result_mtx;    /* guards the one-slot result       */

void     line_task(INT stacd, void * p_exinf);
void     barcode_task(INT stacd, void * p_exinf);
void     barcode_result_put(barcode_result_t const * p_result);
bool     barcode_result_take(barcode_result_t * p_result);
uint16_t barcode_to_command(char const * p_text);
float    line_compute_error(float left, float center, bool b_detected,
                            float last_error);

#endif /* SUBSYSTEM3_H */

/*** end of file ***/
