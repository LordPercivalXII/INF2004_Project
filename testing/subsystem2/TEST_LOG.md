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
| S2-U05 | PID clamps output and prevents integral windup |  | auto | `subsystem2_test.c` | PASS |
| S2-M01 | each motor drives only its own wheel, forward, encoder count positive | FR4, FR5 | manual | `S2_ENCODER_CALIBRATION=1`, `S2_MOTOR_DIR_TEST=1`, SIM=0, wheels off the ground | `[DIR]` lines: only the tested side counts, positive |
| S2-M02 | straight run 5 s at 0.20 m/s, distance vs ruler | FR5, FR6 | manual | `S2_BENCH_DRIVE=1`, SIM=0, floor, ruler | encoder mm close to ruler mm; left/right ticks within a few % |
| S2-M03 | +90 deg and 180 deg point turns | FR6 | manual | `S2_BENCH_DRIVE=1`, SIM=0, floor | turn completes once and stops near the target angle |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-28 | S2-M01 | (this commit) | Pieter | PASS | serial monitor | left +1405, right +1225, both together +4511/+4668 |
| 2026-10-28 | S2-M02 | (this commit) | Pieter | PARTIAL | serial monitor | encoders: 875/880 mm; ruler distance not recorded |
| 2026-10-28 | S2-M03 | (this commit) | Pieter | PARTIAL | serial monitor | encoder angle about 91 deg and 160 deg; measure real angles |
