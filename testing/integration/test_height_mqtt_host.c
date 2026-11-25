/* PC test: ramp height on the serial console == ramp height on the dashboard.
 * Encodes heights with the firmware's MQTT ramp record (telemetry.c) and
 * prints them next to the console text (telemetry_format_fixed); the
 * Python half decodes the bytes with dashboard/server.py and compares.
 *   gcc -std=gnu99 -w -I../../app_program -Ihost_stubs -I../../include \
 *       test_height_mqtt_host.c ../../app_program/common/robot_packed.c \
 *       ../../app_program/subsystem1/telemetry_codec.c -o th && \
 *   ./th | python3 check_height_mqtt.py
 */
#include <stdio.h>
#include <stdarg.h>
#include <tk/tkernel.h>
INT tm_sprintf(UB *, const UB *, ...);
INT tm_putstring(const UB *);
#define USE_BIT_PACKING 1
#define TM_MQTT 0
#include "../../app_program/subsystem1/telemetry.c"

ID gh_telemetry_mbf = 1, gh_mqtt_telemetry_mbf = 1, gh_control_mbf = 1;
uint32_t board_millis(void) { return 0; }
char const *imu_height_rejection_name(imu_height_rejection_t r) { (void)r; return ""; }
INT tk_rcv_mbf(ID a, void *b, TMO c) { (void)a; (void)b; (void)c; return 0; }
INT tk_snd_mbf(ID a, void *b, INT n, TMO c) { (void)a; (void)b; (void)n; (void)c; return 0; }
INT tm_putstring(const UB *s) { (void)s; return 0; }
INT tm_sprintf(UB *d, const UB *f, ...)
{
    va_list a; int n;
    va_start(a, f); n = vsprintf((char *)d, (const char *)f, a); va_end(a);
    return n;
}

int main(void)
{
    /* rise, live peak, completed hump (m) */
    static const float cases[][3] = {
        {0.0f, 0.0f, 0.0f},       {0.0234f, 0.0312f, 0.0f},
        {-0.0269f, 0.0565f, 0.0f}, {0.0986f, 0.0986f, 0.0981f},
        {0.1504f, 0.1504f, 0.1496f}, {0.2101f, 0.2101f, 0.2099f},
        {0.0005f, 0.0015f, 0.0025f}, {-0.0049f, 0.0f, 0.0687f},
    };
    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++)
    {
        telemetry_message_t m = {0};
        uint8_t p[4];
        char r[16], k[16], h[16];
        m.imu.terrain_rise_m = cases[i][0];
        m.imu.terrain_peak_m = cases[i][1];
        m.hump_height_m      = cases[i][2];
        telemetry_encode_height(&m, p);
        telemetry_format_fixed(r, cases[i][0], 3U);
        telemetry_format_fixed(k, cases[i][1], 3U);
        telemetry_format_fixed(h, cases[i][2], 3U);
        printf("%s %s %s %u %u %u %u\n", r, k, h, p[0], p[1], p[2], p[3]);
    }
    return 0;
}
