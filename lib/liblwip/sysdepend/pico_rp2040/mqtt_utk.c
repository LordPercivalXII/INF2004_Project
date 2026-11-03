/*
 * mqtt_utk.c
 * MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 * Runs entirely on the processor-1-owned task that owns
 * the CYW43439 and lwIP networking stack.
 *
 * Buddy 1 extension:
 * Publish robot telemetry as JSON over MQTT using a
 * non-blocking application-side telemetry bridge.
 *
 * This file deliberately avoids tk/tkernel.h and
 * pico-sdk/cyw43 headers to prevent namespace conflicts.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

/* Route table topic for subsystem 3. */
#ifndef MQTT_ROUTE_TOPIC
#define MQTT_ROUTE_TOPIC "inf2004/car/route"
#endif

#define MQTT_ROUTE_QOS 1

/* Buddy 1: remote speed commands. */
#define MQTT_CONTROL_TOPIC "inf2004/car/control"
#define MQTT_CONTROL_QOS 1

/* Buddy 1: wireless robot telemetry topic. */
#ifndef MQTT_TELEMETRY_TOPIC
#define MQTT_TELEMETRY_TOPIC "inf2004/car/telemetry"
#endif

#define MQTT_UTK_HEARTBEAT_MS 5000
#define MQTT_UTK_KEEPALIVE_S 60
#define MQTT_UTK_RETRY_MS 3000

#define MQTT_UTK_RX_TOPIC_MAX 64
#define MQTT_UTK_RX_DATA_MAX 128

/* Buddy 1: outgoing JSON telemetry buffer. */
#define MQTT_UTK_TELEMETRY_MAX 896

static mqtt_client_t *client;
static ip_addr_t broker_addr;
static u8_t connecting;

static char rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t rx_len;
static u8_t rx_truncated;

static void heartbeat_timeout(void *arg);

/*
 * Weak default so the library links without subsystem 3.
 */
__attribute__((weak))
void mqtt_utk_route_rx(const char *payload, uint32_t length)
{
    (void)payload;
    (void)length;
}

/* Weak default so the MQTT library links without Subsystem 1. */
__attribute__((weak))
void mqtt_utk_control_rx(const char *payload, uint32_t length)
{
    (void)payload;
    (void)length;
}

/*
 * Compare MQTT topic strings without string.h.
 */
static u8_t topic_is(const char *topic, const char *wanted)
{
    u32_t i = 0;

    while (topic[i] != '\0' && topic[i] == wanted[i])
    {
        i++;
    }

    return (topic[i] == wanted[i]) ? 1 : 0;
}

/*
 * MQTT incoming publish callback.
 */
static void incoming_publish_cb(void *arg,
                                const char *topic,
                                u32_t tot_len)
{
    u32_t i;

    (void)arg;
    (void)tot_len;

    for (i = 0;
         i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0';
         i++)
    {
        rx_topic[i] = topic[i];
    }

    rx_topic[i] = '\0';

    rx_len = 0;
    rx_truncated = 0;
}

/*
 * MQTT incoming data callback.
 */
static void incoming_data_cb(void *arg,
                             const u8_t *data,
                             u16_t len,
                             u8_t flags)
{
    u16_t i;

    (void)arg;

    for (i = 0; i < len; i++)
    {
        if (rx_len < (MQTT_UTK_RX_DATA_MAX - 1))
        {
            UB c = data[i];

            if (c == '\r' || c == '\n' || c == '\t')
            {
                c = ' ';
            }
            else if (c < 0x20 || c >= 0x7f)
            {
                c = '?';
            }

            rx_data[rx_len++] = c;
        }
        else
        {
            rx_truncated = 1;
        }
    }

    if ((flags & MQTT_DATA_FLAG_LAST) != 0)
    {
        rx_data[rx_len] = '\0';

        tm_printf(
            (UB *)"mqtt: RX [%s] \"%s\"%s\n",
            rx_topic,
            rx_data,
            rx_truncated ? " (truncated)" : ""
        );

        /*
         * Subsystem 3 route table handoff.
         */
        if (topic_is(rx_topic, MQTT_ROUTE_TOPIC) &&
            !rx_truncated)
        {
            mqtt_utk_route_rx(
                (const char *)rx_data,
                (uint32_t)rx_len
            );
        }

        /* Buddy 1: dispatch validated speed commands to Subsystem 1. */
        if (topic_is(rx_topic, MQTT_CONTROL_TOPIC) && !rx_truncated)
        {
            mqtt_utk_control_rx((const char *)rx_data, (uint32_t)rx_len);
        }
    }
}

/*
 * MQTT subscription completion callback.
 */
static void subscribe_done_cb(void *arg, err_t result)
{
    const char *topic =
        (arg != NULL) ? (const char *)arg : MQTT_TOPIC;

    if (result == ERR_OK)
    {
        tm_printf(
            (UB *)"mqtt: subscribed to %s\n",
            topic
        );
    }
    else
    {
        tm_printf(
            (UB *)"mqtt: subscribe failed, err=%d\n",
            (int)result
        );
    }
}

/*
 * MQTT publishing completion callback.
 */
static void publish_done_cb(void *arg, err_t result)
{
    (void)arg;

    if (result != ERR_OK)
    {
        tm_printf(
            (UB *)"mqtt: publish failed, err=%d\n",
            (int)result
        );
    }
}

/*
 * Existing MQTT heartbeat.
 */
static void heartbeat_timeout(void *arg)
{
    static unsigned int count = 0;
    UB payload[48];
    int len;

    (void)arg;

    if (client != NULL && mqtt_client_is_connected(client))
    {
        len = tm_sprintf(
            payload,
            (const UB *)"hb %u",
            count++
        );

        tm_printf(
            (UB *)"mqtt: TX [" MQTT_TOPIC "] \"%s\"\n",
            payload
        );

        (void)mqtt_publish(
            client,
            MQTT_TOPIC,
            payload,
            (u16_t)len,
            0,
            0,
            publish_done_cb,
            NULL
        );
    }

    /*
     * Re-arm the existing heartbeat timer.
     */
    sys_timeout(
        MQTT_UTK_HEARTBEAT_MS,
        heartbeat_timeout,
        NULL
    );
}

/*
 * Buddy 1: Publish one queued telemetry snapshot.
 *
 * Called by cyw43_utk_task(), the task that owns lwIP.
 *
 * The telemetry bridge retrieves a snapshot without
 * blocking and formats it into a JSON payload.
 */
void mqtt_utk_telemetry_poll(void)
{
    char payload[MQTT_UTK_TELEMETRY_MAX];
    uint32_t len;
    err_t err;

    /*
     * Only publish when MQTT is connected.
     * Leave queued telemetry untouched otherwise.
     */
    if (client == NULL || !mqtt_client_is_connected(client))
    {
        return;
    }

    /*
     * Read one pending JSON telemetry message.
     * Returns zero when no snapshot is available.
     */
    len = mqtt_utk_telemetry_read(
        payload,
        (uint32_t)sizeof(payload)
    );

    if (len == 0U)
    {
        return;
    }

    /*
     * Reject an invalid payload length.
     */
    if (len >= sizeof(payload))
    {
        tm_printf(
            (UB *)"mqtt: telemetry payload too large\n"
        );
        return;
    }

    /*
     * Publish telemetry to the MQTT broker.
     */
    err = mqtt_publish(
        client,
        MQTT_TELEMETRY_TOPIC,
        payload,
        (u16_t)len,
        0,  /* QoS 0 */
        0,  /* Not retained */
        publish_done_cb,
        NULL
    );

    if (err != ERR_OK)
    {
        tm_printf(
            (UB *)"mqtt: telemetry publish failed, err=%d\n",
            (int)err
        );
    }
}

/*
 * Retry MQTT connection.
 */
static void retry_timeout(void *arg)
{
    (void)arg;

    connecting = 0;
    mqtt_utk_start();
}

/*
 * MQTT connection callback.
 */
static void connection_cb(mqtt_client_t *c,
                          void *arg,
                          mqtt_connection_status_t status)
{
    (void)c;
    (void)arg;

    if (status == MQTT_CONNECT_ACCEPTED)
    {
        err_t err;

        connecting = 0;

        tm_printf(
            (UB *)"mqtt: connected to " MQTT_BROKER_IP "\n"
        );

        /*
         * Register incoming MQTT callbacks.
         */
        mqtt_set_inpub_callback(
            client,
            incoming_publish_cb,
            incoming_data_cb,
            NULL
        );

        /*
         * Existing heartbeat subscription.
         */
        err = mqtt_subscribe(
            client,
            MQTT_TOPIC,
            0,
            subscribe_done_cb,
            NULL
        );

        if (err != ERR_OK)
        {
            tm_printf(
                (UB *)"mqtt: mqtt_subscribe() failed, err=%d\n",
                (int)err
            );
        }

        /*
         * Existing subsystem 3 route subscription.
         */
        err = mqtt_subscribe(
            client,
            MQTT_ROUTE_TOPIC,
            MQTT_ROUTE_QOS,
            subscribe_done_cb,
            (void *)MQTT_ROUTE_TOPIC
        );

        if (err != ERR_OK)
        {
            tm_printf(
                (UB *)"mqtt: route subscribe failed, err=%d\n",
                (int)err
            );
        }

        /* Buddy 1: subscribe to dashboard speed commands. */
        err = mqtt_subscribe(
            client,
            MQTT_CONTROL_TOPIC,
            MQTT_CONTROL_QOS,
            subscribe_done_cb,
            (void *)MQTT_CONTROL_TOPIC
        );

        if (err != ERR_OK)
        {
            tm_printf((UB *)"mqtt: control subscribe failed, err=%d\n", (int)err);
        }

        /*
         * Ensure only one heartbeat timer chain exists.
         */
        sys_untimeout(
            heartbeat_timeout,
            NULL
        );

        sys_timeout(
            MQTT_UTK_HEARTBEAT_MS,
            heartbeat_timeout,
            NULL
        );
    }
    else
    {
        tm_printf(
            (UB *)"mqtt: connection status %d, retrying in %ds\n",
            (int)status,
            MQTT_UTK_RETRY_MS / 1000
        );

        sys_timeout(
            MQTT_UTK_RETRY_MS,
            retry_timeout,
            NULL
        );
    }
}

/*
 * Initialize or reconnect MQTT.
 */
void mqtt_utk_start(void)
{
    struct mqtt_connect_client_info_t client_info = {0};
    err_t err;

    if (connecting ||
        (client != NULL && mqtt_client_is_connected(client)))
    {
        return;
    }

    if (client == NULL)
    {
        client = mqtt_client_new();

        if (client == NULL)
        {
            tm_printf(
                (UB *)"mqtt: mqtt_client_new() failed\n"
            );
            return;
        }
    }

    if (!ip4addr_aton(
            MQTT_BROKER_IP,
            ip_2_ip4(&broker_addr)))
    {
        tm_printf(
            (UB *)"mqtt: bad MQTT_BROKER_IP\n"
        );
        return;
    }

    IP_SET_TYPE(
        &broker_addr,
        IPADDR_TYPE_V4
    );

    client_info.client_id = MQTT_CLIENT_ID;
    client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

    connecting = 1;

    err = mqtt_client_connect(
        client,
        &broker_addr,
        MQTT_BROKER_PORT,
        connection_cb,
        NULL,
        &client_info
    );

    if (err != ERR_OK)
    {
        tm_printf(
            (UB *)"mqtt: mqtt_client_connect() failed, err=%d\n",
            (int)err
        );

        connecting = 0;
    }
}

/*** end of file ***/
