/*
 * Buddy 1 - WiFi smoke test (standalone, no FreeRTOS).
 * Proves the Pico W radio, your network credentials and DHCP work
 * on the real robot before integrating into the FreeRTOS telemetry task.
 *
 * Expected USB-serial output:
 *   Connecting to <ssid> ...
 *   Connected. IP: 192.168.x.x
 *   alive, link status=3        (3 == CYW43_LINK_UP)
 * The onboard LED (on the CYW43 chip) blinks at ~1 Hz once connected.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "secrets.h"

#define CONNECT_TIMEOUT_MS (15000U)
#define BLINK_HALF_PERIOD_MS (500U)

int main(void)
{
    stdio_init_all();
    sleep_ms(3000); /* give the USB serial port time to enumerate */

    if (cyw43_arch_init() != 0)
    {
        printf("FAIL: cyw43_arch_init\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();

    printf("Connecting to %s ...\n", WIFI_SSID);
    int rc = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                                CYW43_AUTH_WPA2_AES_PSK,
                                                CONNECT_TIMEOUT_MS);
    if (rc != 0)
    {
        printf("FAIL: wifi connect rc=%d\n", rc);
        return 1;
    }
    printf("Connected. IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));

    for (;;)
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(BLINK_HALF_PERIOD_MS);

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(BLINK_HALF_PERIOD_MS);

        int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

        printf("alive, link status=%d\n", status);

        if (status == CYW43_LINK_UP)
        {
            printf("Connected. IP: %s\n",
                   ip4addr_ntoa(netif_ip4_addr(netif_default)));
        }
    }
}