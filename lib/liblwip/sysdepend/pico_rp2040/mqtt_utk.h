/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/*
 * Call once, from cyw43_utk_task() the first time the link reaches
 * CYW43_LINK_UP (see cyw43_utk.c). Starts the MQTT connection to
 * MQTT_BROKER_IP:MQTT_BROKER_PORT (config/wifi_secrets.h) and, once
 * connected, a periodic heartbeat publish on MQTT_TOPIC. Safe to call more
 * than once -- a second call while already connecting/connected is a no-op.
 */
void mqtt_utk_start(void);

/*
 * Buddy 1: Process pending telemetry on the Wi-Fi task.
 * Must only be called by the task that owns lwIP.
 */
void mqtt_utk_telemetry_poll(void);

/*
 * Buddy 1: Retrieve one pending telemetry snapshot as JSON.
 * Returns the payload length, or 0 if no snapshot is available.
 * The implementation must not block.
 */
uint32_t mqtt_utk_telemetry_read(char *buffer, uint32_t buffer_size);

/*
 * Junction route table for subsystem 3 (app_program/subsystem3/route.c).
 * Every complete message received on MQTT_ROUTE_TOPIC is passed to this
 * hook, from the WiFi service task.  The application's definition must
 * only copy the payload and never block (tk_snd_mbf with TMO_POL).  A weak
 * empty default in mqtt_utk.c is used if the application has none.
 */
#include <stdint.h>
void mqtt_utk_route_rx(const char *payload, uint32_t length);

#endif /* MQTT_UTK_H */
