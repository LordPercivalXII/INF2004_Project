
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/apps/mqtt.h"
#include "secrets.h"

#define MQTT_TOPIC "robot/telemetry/test"

static volatile bool mqtt_connected = false;
static mqtt_client_t *mqtt_client;

static void mqtt_connection_callback(
    mqtt_client_t *client,
    void *arg,
    mqtt_connection_status_t status)
{
    (void)client;
    (void)arg;

    mqtt_connected = (status == MQTT_CONNECT_ACCEPTED);
    printf("MQTT connection status=%d\n", status);
}

static void mqtt_publish_callback(void *arg, err_t err)
{
    (void)arg;
    printf("MQTT publish result=%d\n", err);
}

int main(void)
{
    stdio_init_all();
    sleep_ms(5000);

    if (cyw43_arch_init() != 0) {
        printf("WiFi initialization failed\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi...\n");

    int rc = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID,
        WIFI_PASSWORD,
        CYW43_AUTH_WPA2_AES_PSK,
        15000
    );

    if (rc != 0) {
        printf("WiFi connection failed: %d\n", rc);
        return 1;
    }

    printf("WiFi connected! IP: %s\n",
           ip4addr_ntoa(netif_ip4_addr(netif_default)));

    ip_addr_t broker_ip;

    if (!ipaddr_aton(MQTT_BROKER_IP, &broker_ip)) {
        printf("Invalid broker IP\n");
        return 1;
    }

    mqtt_client = mqtt_client_new();

    if (mqtt_client == NULL) {
        printf("MQTT client creation failed\n");
        return 1;
    }

    struct mqtt_connect_client_info_t client_info = {0};
    client_info.client_id = "pico-buddy1";
    client_info.keep_alive = 30;

    cyw43_arch_lwip_begin();

    err_t err = mqtt_client_connect(
        mqtt_client,
        &broker_ip,
        MQTT_PORT,
        mqtt_connection_callback,
        NULL,
        &client_info
    );

    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("MQTT connect request failed: %d\n", err);
        return 1;
    }

    printf("Waiting for MQTT connection...\n");

    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (!mqtt_connected &&
           to_ms_since_boot(get_absolute_time()) - start < 10000) {
        sleep_ms(100);
    }

    if (!mqtt_connected) {
        printf("MQTT connection timed out or was rejected\n");
        return 1;
    }

    printf("MQTT connected successfully!\n");

    while (true) {
        const char *message = "Hello from Pico W!";

        cyw43_arch_lwip_begin();

        err_t publish_result = mqtt_publish(
            mqtt_client,
            MQTT_TOPIC,
            message,
            strlen(message),
            0,
            0,
            mqtt_publish_callback,
            NULL
        );

        cyw43_arch_lwip_end();

        printf("Publish request result=%d\n", publish_result);

        sleep_ms(1000);
    }
}
