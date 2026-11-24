# Subsystem 1 - test log

**Subsystem:** Buddy 1: Telemetry, Wi-Fi, MQTT communication and dashboard integration  
**Student ID:** 2502722  
**Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | Fixed-point 0.35 → "0.350" | | auto | `subsystem1_test.c` | PASS |
| S1-U02 | Fixed-point -1.25 → "-1.250" | | auto | `subsystem1_test.c` | PASS |
| S1-U03 | Fixed-point 1 decimal | | auto | `subsystem1_test.c` | PASS |
| S1-U04 | Telemetry message buffer exists | IF-04 | auto | `subsystem1_test.c` | PASS |
| S1-U05 | JSON snapshot and heartbeat every 1 s | IF-04 | manual | Verify current telemetry implementation and observe console | One snapshot and heartbeat per second, if still applicable |
| S1-U06 | MQTT telemetry message buffer exists | | auto | `subsystem1_test.c` | PASS |
| S1-U07 | MQTT message size configuration | | auto | `subsystem1_test.c` | PASS |
| S1-U08 | MQTT queue length configuration | | auto | `subsystem1_test.c` | PASS |
| S1-U09 | Independent telemetry and MQTT message buffers | IF-04 | auto | `subsystem1_test.c` | PASS |
| S1-M01 | Wi-Fi connectivity | | manual | Configure Wi-Fi credentials and boot Pico W | Wi-Fi connection established |
| S1-M02 | MQTT broker connection | | manual | Start Mosquitto and boot Pico W | MQTT connection established |
| S1-M03 | MQTT telemetry publishing | IF-04 | manual | Subscribe to configured telemetry topic | Telemetry messages received |
| S1-M04 | Packed telemetry payload | | manual | Enable bit packing and inspect MQTT payload | Each packed record contains four bytes |
| S1-M05 | MQTT speed command | IF-02 | manual | Send a supported speed command | Valid command forwarded to controller |
| S1-M06 | MQTT disconnection | | manual | Stop broker under controlled conditions | Disconnection detected |
| S1-M07 | MQTT reconnection | | manual | Restart broker | Connection and telemetry publishing resume |
| S1-M08 | Dashboard live telemetry | | manual | Run broker, backend and dashboard | Received telemetry displayed |
| S1-M09 | Dashboard mission status | | manual | Observe mission state transitions | Display matches received mission state |
| S1-M10 | Dashboard speed display | | manual | Change robot speed | Display matches received speed telemetry |
| S1-M11 | Dashboard obstacle distance | | manual | Receive obstacle distance telemetry | Received distance displayed correctly |
| S1-M12 | Dashboard obstacle width | | proposed manual | Receive obstacle width telemetry | Received width displayed correctly |
| S1-M13 | Dashboard pickup and collision indication | | proposed manual | Trigger supported event under safe conditions | Corresponding warning displayed |
| S1-M14 | Dashboard width reset | | proposed manual | Refresh dashboard after receiving width | Width resets according to intended behaviour |
| S1-M15 | Dashboard collision display reset | | proposed manual | Resolve event and refresh dashboard | Old indication resets without masking active faults |

## Results

Add a new row for every test run. Never edit or delete previous results.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-24 | S1-U01 | `798f6df` | That Hmuu Shwe Eain | PASS | `logs/2026-11-24_unit.txt` | Fixed-point output "0.350" verified |
| 2026-11-24 | S1-U02 | `798f6df` | That Hmuu Shwe Eain | PASS | `logs/2026-11-24_unit.txt` | Negative fixed-point output "-1.250" verified |
| 2026-11-24 | S1-U03 | `798f6df` | That Hmuu Shwe Eain | PASS | `logs/2026-11-24_unit.txt` | Single decimal output "24.0" verified |
| 2026-11-24 | S1-U04 | `798f6df` | That Hmuu Shwe Eain | PASS | `logs/2026-11-24_unit.txt` | Telemetry message buffer exists |
| 2026-11-24 | S1-U06 | `798f6df` | That Hmuu Shwe Eain | PASS | `logs/2026-11-24_unit.txt` | MQTT telemetry message buffer exists |
| 2026-11-24 | S1-U07 | `798f6df` | That Hmuu Shwe Eain | PASS | `logs/2026-11-24_unit.txt` | Packed MQTT message size configured as 4 bytes |
| 2026-11-24 | S1-U08 | `798f6df` | That Hmuu Shwe Eain | PASS | `logs/2026-11-24_unit.txt` | MQTT queue length configured for four frames per snapshot |
| 2026-11-24 | S1-U09 | `798f6df` | That Hmuu Shwe Eain | PASS | `logs/2026-11-24_unit.txt` | Independent telemetry and MQTT buffers verified |

## Test environment

| Item | Configuration |
|---|---|
| Test date | 2026-11-24 |
| Hardware | Raspberry Pi Pico W (RP2040) |
| RTOS | micro T-Kernel Version 3.00 |
| Tested commit | `798f6df` |
| Branch | `buddy1/wifi-telemetry` |
| Build environment | WSL |
| Build command | `make TEST=1 -j8` |
| Firmware | `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_test_sim_speed015.uf2` |
| Simulation mode | Enabled |
| Bit packing | Enabled (`USE_BIT_PACKING=1`) |
| Serial port | COM5 |
| Evidence | `logs/2026-11-24_unit.txt` |

## Automated test summary

| Metric | Result |
|---|---|
| Subsystem 1 automated tests executed | 8 |
| Passed | 8 |
| Failed | 0 |
| Pass rate | 100% |

All eight automated Subsystem 1 tests passed on the Pico W using the simulation-enabled test firmware.

The tests verified fixed-point formatting, message buffer creation, packed MQTT message size configuration, queue length configuration and independent telemetry message buffers.

These results verify the implemented unit checks but do not establish complete end-to-end MQTT communication, actual binary encoding correctness or dashboard functionality.

## Test run observations (2026-11-24)

- micro T-Kernel started successfully and reported that all tasks had started.
- Bit packing was enabled, with a configured MQTT payload size of 4 bytes and buffer size of 384 bytes.
- MQTT heartbeat transmission and reception were observed.
- Intermittent MQTT telemetry publishing errors (`err=-1`) were observed and require further investigation during communication testing.
- Subsystem 2 test S2-U03 failed during the shared test run. This is outside the Subsystem 1 unit test scope.
- The captured serial output contains a gap during Subsystem 4 testing. No missing results have been inferred.
- All eight Subsystem 1 automated test results and the subsystem summary were captured successfully.
- Manual communication, hardware and dashboard tests remain pending.

## Notes

- Automated tests are executed using the test firmware built with `TEST=1`.
- Packed telemetry configuration is controlled by `USE_BIT_PACKING`.
- Manual hardware tests must be conducted under safe and controlled conditions.
- Results are recorded only after the corresponding test has been executed.
- Proposed tests require verification against the current implementation.
- S1-U05 requires confirmation against the current telemetry implementation before it can be retained as an active test or marked as dropped.
- Integration tests involving multiple subsystems should also be documented in `testing/integration/TEST_LOG.md`.
- Failed tests must retain their original result, with a new result added after the fix.
