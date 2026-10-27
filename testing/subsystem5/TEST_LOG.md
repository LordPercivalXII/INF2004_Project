# Subsystem 5 — test log

**Subsystem:** Buddy 5 — obstacle scanner  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | bypass side selection |  | auto | `subsystem5_test.c` | PASS |
| S5-U02 | hit at 24 cm detected |  | auto | `subsystem5_test.c` | PASS |
| S5-U03 | single-hit width ≈ beam footprint |  | auto | `subsystem5_test.c` | PASS |
| S5-U04 | far reading sets left clearance |  | auto | `subsystem5_test.c` | PASS |
| S5-U05 | servo sweeps 45/90/135° and distance tracks a hand | IF-01 | manual | SIM=0, watch servo + telemetry | obstacle_cm follows the hand |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
