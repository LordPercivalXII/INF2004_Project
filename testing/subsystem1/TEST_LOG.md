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

## Notes

- Automated tests are executed using the test firmware built with `TEST=1`.
- Packed telemetry configuration is controlled by `USE_BIT_PACKING`.
- Manual hardware tests must be conducted under safe and controlled conditions.
- Results are recorded only after the corresponding test has been executed.
- Proposed tests require verification against the current implementation.
- S1-U05 requires confirmation against the current telemetry implementation before it can be retained as an active test or marked as dropped.
- Integration tests involving multiple subsystems should also be documented in `testing/integration/TEST_LOG.md`.
- Failed tests must retain their original result, with a new result added after the fix.
