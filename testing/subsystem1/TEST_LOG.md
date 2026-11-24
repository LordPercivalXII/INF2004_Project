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
| S1-U10 | Binary telemetry codec encoding and boundary handling | IF-04 | auto (host) | Compile and execute `test_telemetry_codec.c` using GCC in WSL | All codec assertions pass |
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
| 2026-11-24 | S1-U10 | `HOST_COMMIT` | That Hmuu Shwe Eain | PASS | `logs/2026-11-24_codec.txt` (pending save confirmation) | All host-side telemetry codec assertions passed |

## Test environment

### On-target automated testing

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

### Host-side telemetry codec testing

| Item | Configuration |
|---|---|
| Test date | 2026-11-24 |
| Tested commit | `HOST_COMMIT` |
| Branch | `main` |
| Execution environment | Windows Subsystem for Linux (WSL) |
| Compiler | GCC |
| Language standard | C99 |
| Compiler flags | `-Wall -Wextra` |
| Test source | `app_program/subsystem1/test_telemetry_codec.c` |
| Implementation source | `app_program/subsystem1/telemetry_codec.c` |
| Shared packing source | `app_program/common/robot_packed.c` |
| Physical hardware required | No |
| MQTT broker required | No |
| Result | PASS |
| Evidence | `logs/2026-11-24_codec.txt` (pending save confirmation) |

### Host-side test execution

Run the following commands from the repository root:

```bash
cd app_program/subsystem1

gcc -std=c99 -Wall -Wextra \
    test_telemetry_codec.c \
    telemetry_codec.c \
    ../common/robot_packed.c \
    -o /tmp/test_codec

/tmp/test_codec
```

Observed output:

```text
test_telemetry_codec: all tests passed
```

The test executable completed successfully with all codec assertions passing.

## Automated test summary

| Metric | Result |
|---|---|
| On-target Subsystem 1 automated tests executed | 8 |
| On-target tests passed | 8 |
| On-target tests failed | 0 |
| On-target pass rate | 100% |
| Host-side codec test suites executed | 1 |
| Host-side codec test suites passed | 1 |
| Host-side codec test suites failed | 0 |
| Total recorded automated test cases and suites | 9 |
| Total passed | 9 |
| Total failed | 0 |
| Recorded automated pass rate | 100% |

All eight automated Subsystem 1 tests passed on the Pico W using the simulation-enabled test firmware.

The on-target tests verified fixed-point formatting, message buffer creation, packed MQTT message size configuration, queue length configuration and independent telemetry message buffers.

The additional host-side telemetry codec test suite passed successfully using GCC in WSL. This provides further verification of the binary telemetry encoding implementation and its tested boundary conditions.

The host-side codec suite contains multiple assertions but is recorded as one test suite under S1-U10.

These results verify the implemented automated checks and tested binary encoding behaviour. They do not establish complete end-to-end MQTT communication, successful over-the-air transmission or dashboard functionality.

## Host-side telemetry codec verification

The standalone telemetry codec test suite was executed independently of the embedded hardware and MQTT network.

The test suite exercises the following implementation areas:

- Mission state encoding.
- Fault mask encoding.
- Speed conversion and quantisation.
- Line error encoding.
- Hump height conversion.
- Obstacle distance encoding.
- Numeric saturation and boundary handling.
- Invalid output buffer size handling.
- Starting state encoding.
- IMU waiting state encoding.
- Fault state prioritisation.

All assertions in the executed test suite passed.

This test provides verification of the codec independently of RTOS scheduling, Wi-Fi connectivity and MQTT broker availability.

Actual transmission of packed telemetry records remains subject to manual communication and integration testing.

## Test run observations (2026-11-24)

### On-target automated test run

- micro T-Kernel started successfully and reported that all tasks had started.
- Bit packing was enabled, with a configured MQTT payload size of 4 bytes and buffer size of 384 bytes.
- MQTT heartbeat transmission and reception were observed.
- Intermittent MQTT telemetry publishing errors (`err=-1`) were observed and require further investigation during communication testing.
- Subsystem 2 test S2-U03 failed during the shared test run. This is outside the Subsystem 1 unit test scope.
- The captured serial output contains a gap during Subsystem 4 testing. No missing results have been inferred.
- All eight Subsystem 1 automated test results and the subsystem summary were captured successfully.

### Host-side codec test run

- The telemetry codec test was compiled using GCC in WSL.
- Compilation completed without visible compiler errors or warnings.
- The standalone test executable ran successfully.
- The terminal reported `test_telemetry_codec: all tests passed`.
- No Pico W hardware or MQTT broker connection was required.
- Binary encoding correctness was verified for the cases covered by the existing test suite.
- End-to-end MQTT transmission remains pending verification.

## Remaining verification

| Test ID | Verification | Status |
|---|---|---|
| S1-U05 | Confirm current JSON snapshot and heartbeat behaviour | Pending review |
| S1-M01 | Wi-Fi connectivity | Partial evidence |
| S1-M02 | MQTT broker connection | Partial evidence |
| S1-M03 | MQTT telemetry publishing | Pending verification; errors observed |
| S1-M04 | Actual four-byte MQTT payload transmission | Pending |
| S1-M05 | MQTT speed command forwarding | Pending |
| S1-M06 | MQTT disconnection detection | Pending |
| S1-M07 | MQTT reconnection and recovery | Pending |
| S1-M08 | Dashboard live telemetry | Pending |
| S1-M09 | Dashboard mission status | Pending |
| S1-M10 | Dashboard speed display | Pending |
| S1-M11 | Dashboard obstacle distance | Pending |
| S1-M12 | Dashboard obstacle width | Proposed |
| S1-M13 | Dashboard pickup and collision indication | Proposed |
| S1-M14 | Dashboard width reset | Proposed |
| S1-M15 | Dashboard collision display reset | Proposed |

## Notes

- On-target automated tests are executed using the test firmware built with `TEST=1`.
- Host-side codec tests are executed independently using GCC.
- Packed telemetry configuration is controlled by `USE_BIT_PACKING`.
- Host-side codec testing does not replace on-target communication testing.
- Manual hardware tests must be conducted under safe and controlled conditions.
- Results are recorded only after the corresponding test has been executed.
- Proposed tests require verification against the current implementation.
- S1-U05 requires confirmation against the current telemetry implementation before it can be retained as an active test or marked as dropped.
- Integration tests involving multiple subsystems should also be documented in `testing/integration/TEST_LOG.md`.
- Failed tests must retain their original result, with a new result added after the fix.
- Wi-Fi credentials must not be included in test evidence or technical reports.
