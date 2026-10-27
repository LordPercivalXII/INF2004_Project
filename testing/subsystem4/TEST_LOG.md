# Subsystem 4 — test log

**Subsystem:** Buddy 4 — IMU  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | 5-tap moving average |  | auto | `subsystem4_test.c` | PASS |
| S4-U02 | motion classification thresholds |  | auto | `subsystem4_test.c` | PASS |
| S4-U03 | tilt 45° roll |  | auto | `subsystem4_test.c` | PASS |
| S4-U04 | MPU6050 found on iica (GP4/GP5) | FR10 | manual | SIM=0, watch boot log | no "MPU6050 not found" message |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
