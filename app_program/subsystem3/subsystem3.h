/** @file subsystem3.h
 *
 * @brief Subsystem 3 -- Buddy 3: IR line tracking, Code 39 barcodes and
 *        junction routing.  Student ID: <student ID>.
 *
 * Behaviour:
 *   1. barcode_task decodes a Code 39 symbol crossed in EITHER direction
 *      (code39_decode_dir) and records which way it was read.
 *   2. line_task stores the decoded character in RAM as the pending
 *      waypoint (route.c), plus a short history of recent characters.
 *   3. The car keeps line-following; nothing changes until a junction
 *      (left, centre and right sensors all black for JUNCTION_MIN_MS).
 *   4. At the junction line_task looks the stored character up in the
 *      route table and sends the action (left / right / straight /
 *      U-turn) to the controller with that SYSTEM_EVENT_LINE.  The route
 *      table is NOT in the code: it arrives over MQTT on MQTT_ROUTE_TOPIC
 *      (payload format in route.c) and can be changed at any time.
 *
 * Tasks:
 *   line_task     periodic 10 ms: IR averaging, calibration, line error,
 *                 junctions, route table updates, SYSTEM_EVENT_LINE.
 *   barcode_task  event-driven: debounce GP28 edges, decode Code 39.
 *
 * Interrupts: TIMER alarm 3 at 1 kHz (drv_ir_sampler.c); GP28 edges via
 * the shared GPIO router (drv_barcode_sensor.c).
 *
 * Interfaces (TEAM.md section 5):
 *   IF-01  S3 -> controller: SYSTEM_EVENT_LINE (line_event_t).
 *   IF-06  WiFi service task -> S3: route table text, message buffer
 *          gh_route_cfg_mbf, sent with mqtt_utk_route_rx() (TMO_POL).
 *
 * Owns: the ADC, TIMER alarm 3, GP26, GP27, GP28.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "../common/robot_types.h"
#include "../common/robot_config.h"

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
    char     text[BARCODE_TEXT_MAX + 1U];
    bool     b_reversed;    /* crossed right-to-left */
} barcode_result_t;

/* ---- Route table and waypoint memory (route.c) ---- */

#define ROUTE_TABLE_SIZE    (128U)      /* indexed by 7-bit ASCII     */
#define ROUTE_UNMAPPED      (0xFFU)

typedef struct
{
    uint8_t action[ROUTE_TABLE_SIZE];   /* barcode_command_t or UNMAPPED */
    uint8_t default_action;             /* for characters not mapped     */
} route_table_t;

typedef struct
{
    char     symbol;                    /* the character that routes   */
    char     text[BARCODE_TEXT_MAX + 1U];
    bool     b_reversed;
    uint32_t time_ms;
} route_waypoint_t;

typedef struct
{
    uint32_t applied;       /* pairs accepted                        */
    uint32_t rejected;      /* tokens that could not be understood   */
    bool     b_cleared;     /* "CLEAR" seen                          */
} route_parse_result_t;

typedef struct
{
    bool             b_pending;
    route_waypoint_t pending;
    route_waypoint_t history[ROUTE_HISTORY_LENGTH];  /* newest first */
    uint32_t         history_count;
    uint32_t         mapped_count;
    uint8_t          default_action;
    uint32_t         config_count;  /* route messages applied         */
    uint32_t         config_drops;  /* lost: buffer full / too long   */
} route_status_t;

/* Kernel objects created in subsystem3_init(). */
extern ID gh_barcode_edge_mbf;      /* edge ISR -> barcode_task        */
extern ID gh_barcode_result_mtx;    /* one-slot result                 */
extern ID gh_route_cfg_mbf;         /* IF-06: MQTT -> line_task        */
extern ID gh_route_mtx;             /* route table + waypoint memory   */

/* Called by the WiFi service task (lib/liblwip/.../mqtt_utk.c) for each
 * message on MQTT_ROUTE_TOPIC.  Never blocks. */
void mqtt_utk_route_rx(char const * p_payload, uint32_t length);

/* Private to subsystem 3. */
void     line_task(INT stacd, void * p_exinf);
void     barcode_task(INT stacd, void * p_exinf);
void     barcode_result_put(barcode_result_t const * p_result);
bool     barcode_result_take(barcode_result_t * p_result);
float    line_compute_error(float left, float center, bool b_detected,
                            float last_error);

void     route_table_reset(route_table_t * p_table);
void     route_table_parse(route_table_t * p_table, char const * p_text,
                           uint32_t length, route_parse_result_t * p_result);
uint8_t  route_table_lookup(route_table_t const * p_table, char symbol);
char const * route_action_name(uint8_t action);

void     route_init(void);
void     route_poll_config(void);
void     route_store_waypoint(barcode_result_t const * p_result,
                              uint32_t now_ms);
uint8_t  route_take_junction_action(char * p_symbol);
void     route_get_status(route_status_t * p_status);

#endif /* SUBSYSTEM3_H */

/*** end of file ***/
