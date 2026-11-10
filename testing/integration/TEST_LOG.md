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
