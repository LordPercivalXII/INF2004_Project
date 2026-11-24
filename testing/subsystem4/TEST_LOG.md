# Subsystem 4 — test log

**Subsystem:** Buddy 4 — IMU  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | 5-tap moving average |  | auto | `subsystem4_test.c` | PASS |
| S4-U02 | 45° roll tilt calculation |  | auto | `subsystem4_test.c` | PASS |
| S4-U03 | 30° pitch tilt calculation |  | auto | `subsystem4_test.c` | PASS |
| S4-U04 | LSM303 acceleration decode | FR10 | auto | `subsystem4_test.c` | PASS |
| S4-M01 | LSM303DLHC found on iica (GP4/GP5) | FR10 | manual | SIM=0, watch boot log | no sensor initialisation failure |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-24 | S4-U01 | `4d7805f` | Copilot host test | PASS | `logs/2026-11-24_host.txt` | 5-tap ring and moving average |
| 2026-11-24 | S4-U02 | `4d7805f` | Copilot host test | PASS | `logs/2026-11-24_host.txt` | 45° roll calculation |
| 2026-11-24 | S4-U03 | `4d7805f` | Copilot host test | PASS | `logs/2026-11-24_host.txt` | 30° pitch calculation |
| 2026-11-24 | S4-U04 | `4d7805f` | Copilot host test | PASS | `logs/2026-11-24_host.txt` | 12-bit acceleration decode and scale |
| 2026-11-24 | S4-SUITE | `4d7805f` | Copilot host test | FAIL | `logs/2026-11-24_host.txt` | Full host suite before fixes: 96 passed, 73 failed |
| 2026-11-24 | S4-SUITE | `4d7805f+worktree` | Copilot host test | PASS | `logs/2026-11-24_host_fixed.txt` | Full host suite after fixes: 169 passed, 0 failed |
| 2026-11-24 | S4-M01 | `4d7805f` | Team | PASS | Provided Pico serial console, 24 Nov 2026 | LSM303 init=1, read=1, valid=1, calibration 200/200 |
