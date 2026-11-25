# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | IF-01: line, motion, IMU, obstacle events reach the controller | IF-01 | manual | SIM=1, watch telemetry `line`, `distance`, `hump_m`, `obstacle_cm` | all four change over 10 s |
| INT-02 | IF-02: START/STOP command changes `mission` | IF-02 | manual | call `telemetry_submit_command()` from a test hook | mission 1 ↔ 0 |
| INT-03 | IF-03: controller steering reaches motion PID | IF-03 | manual | SIM=1, `distance` increases while mission = 1 | increases |
| INT-04 | IF-04: telemetry snapshot every 1 s | IF-04 | manual | watch console | 1 line/s |
| INT-05 | IF-05: obstacle sets SYSTEM_BIT_OBSTACLE, avoidance runs | IF-05 | manual | SIM=1, every 12th scan reports 24 cm ahead | mission 2 then back to 1 |
| INT-06 | barcode `*L*`/`*R*` reaches the controller (S3 → controller) | IF-01 | manual | SIM=1, every 10 s | `barcode` alternates 1 / 2 |
| INT-07 | S3 line error steers Buddy 2 motion | IF-01, IF-03 | manual | `TEST=1 SIM=1`; capture telemetry and height traces for 10 s after auto-start | `mission` stays 1, `line` changes, `distance` increases, and `left_m_s` / `right_m_s` respond differently to line drift |
| INT-08 | S3 routed junction action completes through Buddy 2 motion | IF-01, IF-03 | manual | `TEST=1 SIM=1`; scan Code 39 `*A*` across GP28, then observe the simulated junction 3 s later | `barcode` is 65 (`A`), `S3` reports `A -> LEFT`, the trace shows the turn and return to forward wheel speeds, and no fault is reported |
| INT-09 | Real S3 line tracking steers Buddy 2 hardware motion | IF-01, IF-03, FR7 | manual | Build `TEST=0 SIM=0 AUTO_START_MS=15000 S3_LINE_DEBUG=1 BASE_SPEED=0.20`; calibrate both sensors across line and floor during the 3 s boot calibration, then observe after 15 s | calibration reports measured white/black counts (not defaults), `S3 line` readings show the line/error changing, telemetry `mission` becomes 1, `distance` increases slower than the 0.35 m/s run, and wheel motion responds to steering; no fault is reported |
| INT-J1 | junction with stored LEFT/RIGHT/U-turn barcode: drive to centre, turn, resume | S3 -> controller -> S2 | manual | route `A:L,B:R,C:S,D:U` on MQTT; scan `*A*`, drive to a cross | console `CTRL: junction -> turn 90 deg after 10 mm`, car pivots on the cross and follows the left branch |
| INT-J2 | junction with no barcode stored goes straight | S3 -> controller | manual | drive over a cross without scanning first | `CTRL: junction -> STRAIGHT (no barcode stored)`, no turn |
| INT-J3 | barcode stored but no route table published | S3 | manual | scan `*A*` before publishing the route | `S3: junction: 'A' -> STRAIGHT (route table empty ...)` |
| INT-J4 | after a junction turn the car keeps going the way it turned | controller | manual | LEFT junction, then RIGHT junction; also stop the pivot slightly short | car settles onto the branch on the turn side; never spins back the other way |
| INT-J5 | barcode scan session: decode-now request from `line_task` and early/partial decodes (S3 `line_task` ↔ `barcode_task`) | S3 internal, supports INT-J4 | auto (host) | build and run `test_barcode_session_host.c` (command in its header) | `all barcode session tests passed` |
| INT-J6 | line sensor black/white levels learned while driving, so junction detection works on real tape (S3 → controller junction events) | IF-01, FR7 | auto (host) | build and run `test_line_learn_host.c` (command in its header) | tape normalises ≥ 0.70 (junction level) after learning; 2-cycle spike ignored; lifted front does not corrupt black level; program exits 0 |
| INT-H1 | ramp height on the serial console equals ramp height on the dashboard (S4 → S1 MQTT ramp record → `dashboard/server.py`) | IF-04, FR11 | auto (host) | `./th \| python3 check_height_mqtt.py` (command in `test_height_mqtt_host.c` header; needs `dashboard/requirements.txt`) | every case `OK`, `status frame keeps 8-bit hump: OK`, `ALL CONSISTENT` |
| INT-C1 | S4 start-up IMU calibration completes on a still car under sensor noise, and never on a handled or tilted car (gates mission start) | S4 → controller, FR10 | auto (host) | build and run `test_imu_calibration_host.c` (command in its header) | new rule calibrates in ≈2 s with 0/20 failures at every noise level; `all IMU calibration checks passed` |
| INT-P1 | packed wire formats of the inter-task queues: sizes, round trips, rounding, saturation | IF-01, IF-03, IF-07 (`common/robot_packed.h`) | auto (host) | build and run `test_packing_host.c` (command in its header) | `test_packing_host: all tests passed` |

**Hardware safety:** INT-09's image can start the motors automatically after boot
once the IMU is healthy. Keep the track clear and be ready to cut motor power.
Flash a normal non-auto-start image after the test; the auto-start repeats on
every reboot.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-24 | INT-J4 | `7119ddd`+fix | Claude (host test) | PASS | `logs/2026-11-24_junction_defer_host.txt` | Barcode close to junction: junction during an open scan waits (≤ `LINE_JUNCTION_DEFER_MAX_MS`), asks barcode_task to decode now and uses that barcode; a barcode's own wide bar is not taken as a junction; age sent to controller. Old 300 ms hold dropped such junctions. Needs real-car confirmation. |
| 2026-11-24 | INT-H1 | `7119ddd`+fix | Claude (host test) | PASS | `logs/2026-11-24_height_mqtt_consistency.txt` | Ramp height console vs dashboard: firmware MQTT ramp record decoded by `dashboard/server.py` equals the `[S4] height` console text to the mm for 8 cases (negative rise, 15 and 21 cm ramps); status frame's 7-bit hump no longer overrides the 8-bit value. |
| 2026-11-20 | INT-J1 | `1dd3f74` | 2500197 | PASS | <https://youtu.be/rHV7J_s4asU> | LEFT barcode read, car turns left at the junction and resumes. Added 2026-11-25 from `subsystem3/TEST_LOG.md` (S3-M03) and `videos/README.md`; console line not captured. |
| 2026-11-20 | INT-J1 | `1dd3f74` | 2500197 | PASS | <https://youtu.be/oEcwkz3duA4> | RIGHT barcode read, car turns right at the junction and resumes. Added 2026-11-25, same sources. |
| 2026-11-20 | INT-J1 | `1dd3f74` | 2500197 | PASS | <https://youtu.be/jiD_DmmuEf0> | U-turn barcode read, car U-turns at the junction. Added 2026-11-25, same sources. |
| 2026-11-24 | INT-04 | `798f6df` | 2502722 (run); read for INT-04 on 2026-11-25 | PASS | `../subsystem1/logs/2026-11-24_unit.txt` | `TEST=1 SIM=1` image: one telemetry snapshot line per second, 44.0 s to 57.0 s with no gaps (earlier part of the capture was lost). |
| 2026-11-24 | INT-01 | `798f6df` | 2502722 (run); read for INT-01 on 2026-11-25 | PARTIAL | `../subsystem1/logs/2026-11-24_unit.txt` | Same `SIM=1` run. `line` changes (−1.000…0.980), `obst` 100 → 24 → 100 cm, IMU `pitch` reaches −12.7°. `dist` stays 0.000 m because the mission never left IDLE, and `hump_m` is not in the console snapshot. Re-run with the mission started. |
| 2026-11-24 | INT-05 | `e61bdc4` + uncommitted | S5 owner (run); read for INT-05 on 2026-11-25 | PASS (log missing) | `../subsystem5/logs/2026-11-24_avoidance_rejoin.txt` (**not in the repo**) | Real car, `SIM=0`. Taken from the S5-M01 PASS row: `avoid done`, then FOLLOW (mission 2 → 1) with line errors 0.055 and 0.283. The log file named there was never committed; commit it or treat this row as unsupported. |
| 2026-11-24 | INT-05 | `7f5eef0` + uncommitted (flashed build unverified) | S5 owner (run); read for INT-05 on 2026-11-25 | PARTIAL | `../subsystem5/logs/2026-11-24_parallel_rejoin_attempt.txt` | Real car, `SIM=0` (not the `SIM=1` method). `S5: STOP obstacle in path at 17 cm`, mission FOLLOW → AVOID at 4.0 s, bypass states run to SWEEP. Return to FOLLOW (mission 1) is not in the capture. |
| 2026-11-24 | INT-J5 | `7119ddd`+fix | Claude (host test) | PASS | `logs/2026-11-24_junction_defer_host.txt` | Row added 2026-11-25; output was already in the log. 7 of 7 checks ok. |
| 2026-11-24 | INT-J6 | `7119ddd`+fix | Claude (host test) | PASS | `logs/2026-11-24_junction_defer_host.txt` | Row added 2026-11-25; output was already in the log. Learned white 300 / black 1500 counts; tape 0.64 → 1.00 ≥ 0.70. The exit code was not logged; the 2026-11-25 rerun exits 0. |
| 2026-11-25 | INT-J4 | `4fccb06` | Claude (host test) | PASS | `logs/2026-11-25_host_suite.txt` | Rerun at `main` head: all junction deferral tests passed. Real-car confirmation is still needed. |
| 2026-11-25 | INT-J5 | `4fccb06` | Claude (host test) | PASS | `logs/2026-11-25_host_suite.txt` | 7 of 7 checks ok. |
| 2026-11-25 | INT-J6 | `4fccb06` | Claude (host test) | PASS | `logs/2026-11-25_host_suite.txt` | Same output as 2026-11-24; exit 0. |
| 2026-11-25 | INT-H1 | `4fccb06` | Claude (host test) | PASS | `logs/2026-11-25_host_suite.txt` | 8 cases all consistent; status frame keeps 8-bit hump. |
| 2026-11-25 | INT-C1 | `4fccb06` | Claude (host test) | PASS | `logs/2026-11-25_host_suite.txt` | First recorded run. New rule: 2.00 s, 0/20 failures at 3–9 mg rms noise (old rule fails 20/20 at 9 mg). Handled and tilted car never calibrate. Bias recovered to within 1 mg. |
| 2026-11-25 | INT-P1 | `4fccb06` | Claude (host test) | FAIL | `logs/2026-11-25_host_suite.txt` | First recorded run. `test_obstacle` asserts `estimated_width_cm == 18.0` and gets 18.2. Since `5a0d51b` (2026-11-20) the firmware packs width at 0.1 cm, so the test is out of date, not the firmware. All other checks pass when only this assert is replaced. Note: 9-bit × 0.1 cm now caps width at 51.1 cm. Fix: update the assert to 18.2, then re-run. |
