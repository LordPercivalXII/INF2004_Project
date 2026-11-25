# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | full track run: the car follows the line from start to finish, reading barcodes and taking the routed action at each junction | FR7, FR8, FR9 | manual | 1. `SIM=0` build at the test commit. 2. Publish the route on MQTT. 3. Place the car at the start, still and level, and wait for IMU calibration. 4. START. 5. Video the whole run. | car reaches the end of the track without leaving the line or being touched; every junction action matches the route; no fault is reported |
| SYS-02 | pickup safety: lifting the car while it drives stops the motors | FR10 (IMU safety), IF-07 | manual | 1. Run SYS-01. 2. Lift the car partway through the run. 3. Put it back down. | motors stop as soon as the car is lifted and stay stopped until the operator acknowledges the fault |
| SYS-03 | ramp hump height: the car climbs the ramp and reports its height | FR11, IF-04 | manual | 1. Run the car over the test ramp. 2. Save the console. 3. Read the dashboard. | `[S4] height` hump on the console and the dashboard is within 5 % of the measured ramp height; car keeps following the line |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-20 | SYS-01 | `79d6848` | 2502840, 2502346 | NOT GRADED | <https://youtu.be/GMN_msi8kEA> | Added 2026-11-25 from `videos/README.md` ("Full Track IMU Pickup": full track run). The video index does not say whether the run met every expected result. Watch the video, then record PASS or FAIL in a new row. No console log. |
| 2026-11-20 | SYS-02 | `79d6848` | 2502840, 2502346 | PASS | <https://youtu.be/GMN_msi8kEA> | Added 2026-11-25 from `videos/README.md`: "IMU pickup stops the motors when lifted". The video index does not record whether the fault stayed latched until acknowledged. |
| 2026-11-20 | SYS-03 | `79d6848` | 2502346 | PASS | <https://youtu.be/uVAXxTT2UN4> | Added 2026-11-25 from `videos/README.md` ("IMU Ramp Height Detection", FR11). Graded PASS by the team from the video; the reported and measured heights were not written down. |
| 2026-11-24 | SYS-03 | `7119ddd`+fix | Team (real car) | FAIL | `../subsystem4/logs/2026-11-24_ramp_serial_hump_zero.txt` (**not in the repo**) | Added 2026-11-25 from S4-M02. On the real ramp the hump read 0.000 m: the LSM303 board is mounted turned 180°. Fixed in firmware with `IMU_MOUNT_ROTATED_180` (host-verified, S4-F05). A real-ramp retest is still needed. The log file named in S4-M02 was never committed. |
