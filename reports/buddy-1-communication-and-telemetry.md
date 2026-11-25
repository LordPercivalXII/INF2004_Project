# Buddy 1: Communication and Telemetry Report

**Subsystem:** Wi-Fi communication, MQTT, command handling, telemetry and dashboard integration

**Source:** `app_program/subsystem1/`

**Configuration reviewed:** `app_program/subsystem1/subsystem1.h`, `app_program/common/robot_config.h`

**Network configuration reviewed:** `config/wifi_secrets.h`

**Dashboard:** `dashboard/`

**Latest recorded test evidence:** 24 November 2026

## 1. Objective

Buddy 1 provides the communication interface between the autonomous robot and the external monitoring and control dashboard.

The subsystem is responsible for transmitting robot telemetry, receiving user commands, supporting wireless communication through MQTT and providing visibility into the robot's operating state.

The design separates communication processing from time-critical robot control tasks through RTOS message buffers and independent task execution.

The main objectives are:

- Establish wireless communication using the Raspberry Pi Pico W.

- Transmit robot telemetry through MQTT.

- Receive and forward supported user commands to the controller.

- Reduce communication overhead through packed binary telemetry.

- Provide live monitoring through the dashboard.

- Support connection monitoring and recovery.

## 2. Communication architecture

The subsystem uses micro T-Kernel message buffers to transfer telemetry between the controller, telemetry task and wireless communication task.

The telemetry data flow is:

```text
Robot subsystems
       |
       v
Controller
       |
       v
gh_telemetry_mbf
       |
       v
Telemetry task
       |
       v
Telemetry encoding
       |
       v
gh_mqtt_telemetry_mbf
       |
       v
Wi-Fi / MQTT task
       |
       v
MQTT broker
       |
       v
Dashboard backend
       |
       v
Robot control dashboard
```

The command path operates in the opposite direction:

```text
User command
       |
       v
MQTT broker
       |
       v
MQTT command reception
       |
       v
telemetry_submit_command()
       |
       v
Controller message buffer
       |
       v
Robot controller
```

This separation allows telemetry formatting and network communication to be handled independently of the controller's main execution path.

## 3. RTOS task and message buffer implementation

The subsystem creates two independent message buffers during `subsystem1_init()`.

| Message buffer | Purpose |
|---|---|
| `gh_telemetry_mbf` | Receives telemetry snapshots from the controller |
| `gh_mqtt_telemetry_mbf` | Transfers telemetry records to the wireless MQTT task |

Both message buffers use FIFO task-wait ordering through `TA_TFIFO`.

The initialisation function checks the return value of each `tk_cre_mbf()` call and returns an error if buffer creation fails.

The telemetry task is started through `subsystem1_start()` using `app_start_task()`.

The current subsystem configuration is:

| Parameter | Value |
|---|---|
| Microcontroller | Raspberry Pi Pico W (RP2040) |
| RTOS | micro T-Kernel Version 3.00 |
| Telemetry task priority | 20 |
| Assigned priority band | 20-21 |
| Core affinity | `APP_CORE_ANY` |
| Receive timeout configuration | 100 ms |
| Fixed-point text buffer | 16 bytes |
| Message buffer task-wait ordering | `TA_TFIFO` |
| Packed MQTT message size | 4 bytes |
| Packed MQTT queue length | `ROBOT_QUEUE_LENGTH * 4` |

The telemetry task is assigned to the lowest application priority band so that time-critical control and sensor tasks can receive scheduling precedence over telemetry processing.

The independent message buffers reduce direct coupling between telemetry generation and wireless transmission. However, buffer separation alone does not guarantee loss-free communication when the network is unavailable or the buffers become full.

## 4. Communication interfaces

The subsystem exposes the following public interfaces:

| Interface | Function / object | Purpose |
|---|---|---|
| IF-04 | `telemetry_publish()` | Submit a telemetry snapshot |
| IF-04 | `gh_telemetry_mbf` | Controller-to-telemetry message buffer |
| IF-02 | `telemetry_submit_command()` | Forward a user command to the controller |
| MQTT reception | `mqtt_utk_control_rx()` | Receive supported MQTT speed commands |
| Internal MQTT handoff | `gh_mqtt_telemetry_mbf` | Transfer telemetry to the MQTT task |
| Initialisation | `subsystem1_init()` | Create subsystem message buffers |
| Task startup | `subsystem1_start()` | Start the telemetry task |

The communication interfaces use structured telemetry and control command types to separate data exchange from the internal implementation of individual robot subsystems.

### MQTT network configuration

The Pico W is configured to connect to a dedicated Wi-Fi hotspot for robot communication and testing.

| Parameter | Configuration |
|---|---|
| Wi-Fi network | `PIHOTSPOT` |
| MQTT broker address | `192.168.137.1` |
| MQTT broker port | `1883` |
| MQTT client identifier | `buddy1-pico` |
| Heartbeat topic | `mtk3smp-pico/heartbeat` |
| Route command topic | `inf2004/car/route` |

The network configuration is defined in `config/wifi_secrets.h`. The MQTT broker acts as an intermediary between the embedded robot and external monitoring applications. These settings target a controlled local testing network. Wi-Fi credentials are excluded from this report. Active credentials should be rotated if exposed and maintained in an untracked local configuration file in future.

## 5. Packed telemetry and resource efficiency

The subsystem supports packed binary telemetry and an unpacked snapshot configuration through the `USE_BIT_PACKING` build setting.

When bit packing is enabled:

```text
S1_MQTT_MESSAGE_SIZE = 4 bytes
S1_MQTT_QUEUE_LENGTH = ROBOT_QUEUE_LENGTH * 4
```

Each packed MQTT record occupies 32 bits.

The queue length configuration provides capacity for four packed records per original telemetry queue entry.

The application payload size for four packed records is:

```text
4 records * 4 bytes = 16 bytes
```

When bit packing is disabled, the MQTT handoff buffer is configured using `sizeof(telemetry_message_t)` and `ROBOT_QUEUE_LENGTH`.

The telemetry implementation also provides fixed-point formatting for decimal values.

The following formatting cases have been verified:

| Input | Decimal places | Expected output |
|---|---:|---|
| 0.35 | 3 | `0.350` |
| -1.25 | 3 | `-1.250` |
| 24.04 | 1 | `24.0` |

The recorded test firmware reported:

```text
[PACK] USE_BIT_PACKING=1 (message B / buffer B)
[PACK] mqtt payload   4 / 384
```

This confirms that the tested firmware used a configured packed MQTT message size of four bytes and reported a buffer size of 384 bytes.

The packed representation is intended to reduce application payload size and communication overhead compared with text-based telemetry.

A host-side comparison of one binary status record (`01 0F 80 94`) against equivalent selected-field JSON measured 4 bytes versus 66 bytes, saving 62 bytes (93.94% reduction; 16.5:1 size ratio). This is not a comparison of complete telemetry snapshots. Evidence: `testing/subsystem1/logs/2026-11-24_payload_comparison.txt`.

The four-byte record size does not include MQTT topic information, protocol headers, TCP/IP headers or Wi-Fi transmission overhead.

## 6. Dashboard integration

The dashboard provides an external interface for monitoring the autonomous robot and issuing supported commands.

The dashboard integration is intended to expose telemetry such as:

| Telemetry category | Information |
|---|---|
| Robot status | Current mission and operating state |
| Motion | Speed and distance travelled |
| Navigation | Line sensor and barcode information |
| Terrain | Pitch, roll and hump measurements |
| Obstacles | Obstacle distance and width |
| Safety | Fault, pickup and collision indications |

The dashboard provides a central monitoring interface for information generated by multiple robot subsystems.

The displayed values must be checked against the corresponding received MQTT records to verify decoding accuracy and ensure that the dashboard does not display stale or incorrect information.

Host-side dashboard integration and command publication have been verified using an isolated local MQTT broker and synthetic telemetry. Physical robot command execution and sensor-to-dashboard accuracy require separate hardware validation.

## 7. Recorded results

The automated tests were executed on the Raspberry Pi Pico W using the simulation-enabled micro T-Kernel test firmware.

| Item | Configuration |
|---|---|
| Test date | 24 November 2026 |
| Tested commit | `798f6df` |
| Build command | `make TEST=1 -j8` |
| Simulation mode | Enabled |
| Bit packing | Enabled |
| Serial port | COM5 |

The recorded Subsystem 1 results are:

| Test | Result | Scope |
|---|---|---|
| `S1-U01` | PASS | Positive fixed-point formatting |
| `S1-U02` | PASS | Negative fixed-point formatting |
| `S1-U03` | PASS | Single-decimal formatting |
| `S1-U04` | PASS | Telemetry message buffer creation |
| `S1-U06` | PASS | MQTT telemetry message buffer creation |
| `S1-U07` | PASS | Four-byte packed MQTT message size |
| `S1-U08` | PASS | Four-frame queue length configuration |
| `S1-U09` | PASS | Independent telemetry and MQTT buffers |

The recorded summary was:

```text
[TEST] subsystem1 SUMMARY 8 passed, 0 failed
```

The automated test pass rate was:

```text
Pass rate = (8 / 8) * 100 = 100%
```

Test definitions and individual results are recorded in:

`testing/subsystem1/TEST_LOG.md`

The original serial evidence is stored in:

`testing/subsystem1/logs/2026-11-24_unit.txt`

The eight automated tests verify the implemented formatting and message buffer checks. They do not independently establish complete end-to-end wireless communication or dashboard accuracy.

### Host-side telemetry codec test (S1-U10)

| Item | Result |
|---|---|
| Date | 24 November 2026 |
| Tested commit | `e61bdc4` |
| Branch at execution | `main` |
| Environment | WSL, GCC, C99 |
| Test source | `app_program/subsystem1/test_telemetry_codec.c` |
| Result | PASS |
| Exit code | 0 |
| Evidence | `testing/subsystem1/logs/2026-11-24_codec.txt` |

Compilation and execution from the repository root:

```bash
cd app_program/subsystem1
gcc -std=c99 -Wall -Wextra test_telemetry_codec.c telemetry_codec.c ../common/robot_packed.c -o /tmp/test_codec && /tmp/test_codec
```

Observed output:

```text
test_telemetry_codec: all tests passed
Exit code: 0
```

The suite verifies the tested binary encoding and boundary handling cases. All eight on-target checks and one host-side suite passed. The host-side suite is counted separately from the eight Pico W tests and does not establish successful MQTT delivery.

### Host-side MQTT and dashboard integration tests

The following tests were performed on 24 November 2026 using the laptop, Mosquitto and Flask. The MQTT integration broker used `127.0.0.1:18884`, separate from the physical robot configuration. Synthetic telemetry was used to exercise the dashboard without the shared robot.

| Test scope | Observed result | Status |
|---|---|---|
| Local MQTT publish/subscribe | Subscriber received `Buddy1 MQTT communication test` on `buddy1/test` | PASS (host) |
| Dashboard broker connection | Broker connected; waiting for telemetry | PASS (host) |
| Live telemetry display | Speed 0.150 m/s, Following state, distance 25.0 cm, line error 0.050, peak height 2.50 cm and one completed hump | PASS (synthetic) |
| Stale telemetry | Dashboard returned to waiting state and cleared live readings | PASS (host) |
| Broker disconnection | Disconnected status displayed and command controls disabled | PASS (host) |
| Broker reconnection | Live telemetry resumed; speed 0.200 m/s, distance 35.0 cm and line error 0.030 displayed | PASS (host) |
| Command publication | MQTT subscriber received speed, start, stop and route messages | PASS (host) |
| Mission history | Synthetic mission created and displayed with recorded events and summary values | PASS (synthetic) |
| JSON export | Mission record and event timeline exported as JSON | PASS (synthetic) |

The command subscriber captured:

```text
inf2004/car/control SPEED:0.25
inf2004/car/control START
inf2004/car/control STOP
inf2004/car/route CLEAR,A:S,B:L,C:R,D:U
```

These observations verify dashboard-to-broker publication, not command execution by the Pico W or controller.

The development mission endpoint `/api/dev/test-mission` returned success with nine synthetic samples. The exported mission (ID 14) contained eight recorded events, a seven-second elapsed time, initial speed of 0.15 m/s, final reported distance of 1.75 m, one avoidance entry and one completed hump with a peak height of 0.025 m. These are synthetic input values, not physical measurements. The export also contains earlier mission summaries whose provenance is not established by this test. The separate MQTT-driven mission history screenshot shows a running mission in the integration test.

#### Evidence files

All paths below are relative to `testing/subsystem1/logs/`.

| Evidence | File |
|---|---|
| Host MQTT output | `2026-11-24_mqtt_host.txt` |
| Host MQTT screenshot | `2026-11-24_mqtt_host.png` |
| Broker connection | `2026-11-24_dashboard_mqtt_connected.png` |
| Live synthetic telemetry | `2026-11-24_dashboard_live_telemetry.png` |
| MQTT-driven mission history | `2026-11-24_dashboard_mqtt_mission.png` |
| Stale telemetry | `2026-11-24_dashboard_stale_telemetry.png` |
| Broker disconnection | `2026-11-24_dashboard_mqtt_disconnected.png` |
| Broker reconnection | `2026-11-24_dashboard_mqtt_reconnected.png` |
| Command publication | `2026-11-24_dashboard_commands.png` |
| Synthetic mission export | `2026-11-24_dashboard_mission.json` |

The codec evidence was recorded in commit `87e2bb3`; the tested source commit was `e61bdc4`. Dashboard and host MQTT evidence is listed above.

### Additional binary MQTT verification (2026-11-24)

Synthetic four-byte records were published to `inf2004/car/telemetry` using Mosquitto at `127.0.0.1:18884`. Flask decoded the records and the dashboard displayed the received fields. The exact source commit for this additional session was not captured; the terminal showed branch `main`. Evidence filenames are recorded below.

| Check | Observed result | Evidence |
|---|---|---|
| Four-byte binary status record via local MQTT | Speed 0.150 m/s, Following, distance 40.0 cm; binary=true | `testing/subsystem1/logs/2026-11-24_binary_status.txt`, `testing/subsystem1/logs/2026-11-24_binary_status_dashboard.png` |
| Obstacle width decoding | 25.0 cm displayed | `testing/subsystem1/logs/2026-11-24_binary_width.txt`, `testing/subsystem1/logs/2026-11-24_binary_width_dashboard.png` |
| Pickup/collision indication | Warning displayed; subsequent screenshot showed no collision; no physical stop verified | `testing/subsystem1/logs/2026-11-24_binary_pickup.txt`, `testing/subsystem1/logs/2026-11-24_binary_pickup_dashboard.png` |
| Hump record decoding | Last hump 2.50 cm; previously received width retained | `testing/subsystem1/logs/2026-11-24_binary_hump.txt`, `testing/subsystem1/logs/2026-11-24_binary_hump_dashboard.png` |
| Ramp and IMU decoding | Pitch 10 degrees, rise 1.50 cm, peak 2.50 cm, count 1, calibrated and valid tracking | `testing/subsystem1/logs/2026-11-24_binary_ramp_imu.txt`, `testing/subsystem1/logs/2026-11-24_binary_ramp_imu_dashboard.png` |
| Barcode A decoding | A, junction 1, STRAIGHT | `testing/subsystem1/logs/2026-11-24_binary_barcode.txt`, `testing/subsystem1/logs/2026-11-24_binary_barcode_dashboard.png` |
| Barcode B decoding | B, junction 2, LEFT | `testing/subsystem1/logs/2026-11-24_binary_barcode_b.txt`, `testing/subsystem1/logs/2026-11-24_binary_barcode_b_dashboard.png` |
| Binary mission event recording and export | Two barcode events A/STRAIGHT and B/LEFT; running mission, peak 2.50 cm, count 1 | `testing/subsystem1/logs/2026-11-24_binary_mission_history.json`, `testing/subsystem1/logs/2026-11-24_binary_mission_history.png` |
| Application payload comparison | 4 bytes versus 66 bytes; 93.94% reduction | `testing/subsystem1/logs/2026-11-24_payload_comparison.txt` |

The status packet `01 0F 80 94` was compared with the equivalent selected-field JSON `{"mission":1,"fault":0,"speed":0.15,"line":0.0,"obstacle_cm":40.0}`. Reduction = (66 - 4) / 66 × 100 = 93.94%. This is a single status payload comparison, not a full snapshot or network traffic measurement. Route confirmation remained unavailable without the robot. The collision warning appeared while mission status remained Following; physical stopping is not established. The mission history remained running with stale telemetry, and restart persistence was not tested. Additional reset tests were excluded.

## 8. Communication reliability and verification status

During the recorded test run, MQTT heartbeat transmission and reception were observed:

```text
mqtt: TX [mtk3smp-pico/heartbeat] "hb 4"
mqtt: RX [mtk3smp-pico/heartbeat] "hb 4"
```

This demonstrates MQTT heartbeat activity during the test session.

However, telemetry publishing errors were also recorded:

```text
mqtt: telemetry publish failed, err=-1
```

The cause of these publishing errors has not yet been established.

The MQTT publishing implementation, connection state and available communication resources should be examined before concluding that telemetry transmission is reliable.

| Function | Verification status |
|---|---|
| Telemetry formatting | PASS |
| Telemetry message buffer creation | PASS |
| MQTT message buffer creation | PASS |
| Packed message size configuration | PASS |
| Packed queue length configuration | PASS |
| Independent message buffers | PASS |
| Wi-Fi connectivity | PARTIAL |
| MQTT broker communication | PASS (host); partial on Pico W |
| MQTT heartbeat | Observed |
| MQTT telemetry publishing | Errors observed |
| Host-side binary codec verification | PASS |
| Actual binary payload transmission | PASS (host); Pico W transmission not verified |
| MQTT command publication | PASS (host) |
| MQTT command forwarding to controller | NOT YET VERIFIED |
| Connection recovery | PASS (dashboard/broker); Pico W recovery pending |
| Dashboard synthetic telemetry display | PASS (host) |
| Physical sensor-to-dashboard accuracy | NOT YET VERIFIED |

The standalone telemetry codec suite passed in WSL using GCC at commit `e61bdc4`. This verifies the tested encoding and boundary cases independently of the MQTT network.

The remaining communication tests are defined in `testing/subsystem1/TEST_LOG.md`.

### Standalone dashboard verification outcome

Host-side tests demonstrated broker connection, synthetic telemetry reception, stale-data handling, disconnection indication, reconnection and command publication. Mission history and JSON export were also exercised. Additional binary tests verified width, hump, ramp/IMU, pickup indication and barcode events. Restart persistence and remaining untested fields were not verified.

## 9. Limitations and future improvements

The recorded tests establish the tested embedded formatting, buffer configuration, codec behaviour and local MQTT/dashboard functionality. Physical robot integration was not fully verified within the recorded scope. Intermittent Pico W MQTT publishing errors were observed, and pickup and barcode updates were reported as inconsistent during later physical operation. The cause was not established. Host command publication does not prove physical execution. The payload comparison measures application bytes for one status record, not full network traffic. Future work should prioritise comparing actual MQTT packets against controller telemetry and resolving publishing errors.

## 10. Conclusion

Buddy 1 implements the telemetry and communication framework using micro T-Kernel message buffers, a dedicated telemetry task, MQTT and a browser-based monitoring and control dashboard. Independent message buffers separate controller telemetry from the wireless publishing handoff, and the packed configuration uses four-byte telemetry records.

All eight recorded Subsystem 1 automated checks passed on the simulation-enabled Pico W firmware. The separate host-side telemetry codec suite also passed at commit `e61bdc4`, with evidence recorded in `87e2bb3`.

Host-side integration tests verified MQTT publish/subscribe communication, dashboard telemetry display, stale-data handling, broker disconnection and reconnection, command publication, mission history and JSON export using synthetic inputs. The recorded evidence demonstrates the tested software functionality without requiring the shared robot.

MQTT heartbeat activity was observed on the Pico W, but telemetry publishing errors were also recorded. Complete physical robot telemetry delivery, controller command execution and embedded connection recovery remain unverified. Additional local binary tests verified decoding, barcode mission events and a 93.94% selected-status payload reduction. The recorded scope totals 33 passing checks and suites (8 on-target, 1 host codec suite, 24 host checks), not full requirement coverage. The results establish the recorded subsystem and host-side functionality, with physical telemetry consistency remaining a limitation.
