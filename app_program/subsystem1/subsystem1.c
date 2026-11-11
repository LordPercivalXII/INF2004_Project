/** @file subsystem1.c
 *
 * @brief Subsystem 1 -- telemetry: creates its objects, starts its task.
 */

#include "subsystem1.h"
#include "../common/robot_config.h"
#include "telemetry_codec.h"

ID gh_telemetry_mbf = E_ID;

/* Buddy 1: telemetry handoff to the Wi-Fi/MQTT task */
ID gh_mqtt_telemetry_mbf = E_ID;

/*!
 * @brief Create the telemetry message buffer (was a FreeRTOS queue).
 */
ER
subsystem1_init (void)
{
    T_CMBF cmbf = {0};

    /* Existing controller-to-telemetry buffer */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.maxmsz = (INT) sizeof(telemetry_message_t);
    cmbf.bufsz  = (SZ) APP_MBF_BUFSZ(ROBOT_QUEUE_LENGTH,
                                     sizeof(telemetry_message_t));

    gh_telemetry_mbf = tk_cre_mbf(&cmbf);

    if (gh_telemetry_mbf < E_OK)
    {
        return gh_telemetry_mbf;
    }

    /* Telemetry-to-MQTT buffer: 7-byte binary packets when packed
       (Tribunal 1), whole snapshots for the JSON payload otherwise. */
    cmbf.maxmsz = (INT) S1_MQTT_MESSAGE_SIZE;
    cmbf.bufsz  = (SZ) APP_MBF_BUFSZ(ROBOT_QUEUE_LENGTH,
                                     S1_MQTT_MESSAGE_SIZE);
    gh_mqtt_telemetry_mbf = tk_cre_mbf(&cmbf);

    if (gh_mqtt_telemetry_mbf < E_OK)
    {
        return gh_mqtt_telemetry_mbf;
    }

    return E_OK;
}

/*!
 * @brief Start the telemetry task.
 */
ER
subsystem1_start (void)
{
    ID const tid = app_start_task((FP) telemetry_task, S1_PRI_TELEMETRY,
                                  APP_CORE_ANY, "telemetry_task");

    return ((tid < E_OK) ? tid : E_OK);
}

/*** end of file ***/
