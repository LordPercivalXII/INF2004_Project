# Subsystem 2 — test log

**Subsystem:** Buddy 2 — motion  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | PID output equals target with zero error |  | auto | `subsystem2_test.c` | PASS |
| S2-U02 | PID increases output when wheel is slow |  | auto | `subsystem2_test.c` | PASS |
| S2-U03 | encoder counts advance while driving (SIM=1) | IF-03 | auto | `subsystem2_test.c` | PASS |
| S2-U04 | wheels turn forward at 0.35 m/s on START | IF-03 | manual | SIM=0, wheels off the ground | both wheels forward, similar speed |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
