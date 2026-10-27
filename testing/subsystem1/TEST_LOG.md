# Subsystem 1 — test log

**Subsystem:** Buddy 1 — telemetry  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | fixed-point 0.35 → "0.350" |  | auto | `subsystem1_test.c` | PASS |
| S1-U02 | fixed-point -1.25 → "-1.250" |  | auto | `subsystem1_test.c` | PASS |
| S1-U03 | fixed-point 1 decimal |  | auto | `subsystem1_test.c` | PASS |
| S1-U04 | telemetry message buffer exists | IF-04 | auto | `subsystem1_test.c` | PASS |
| S1-U05 | JSON snapshot + heartbeat every 1 s | IF-04 | manual | Flash normal image, watch console | one snapshot and one heartbeat per second |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
