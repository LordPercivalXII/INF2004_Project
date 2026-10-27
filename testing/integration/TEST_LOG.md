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

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
