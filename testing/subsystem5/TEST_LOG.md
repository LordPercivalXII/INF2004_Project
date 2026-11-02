# Subsystem 5 — test log

**Subsystem:** Buddy 5 — obstacle scanner  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | bypass side selection |  | auto | `subsystem5_test.c` | PASS |
| S5-U02 | hit at 24 cm detected (dropped: replaced by S5-U10) |  | auto | removed | — |
| S5-U03 | single-hit width ≈ beam footprint (dropped: replaced by S5-U11) |  | auto | removed | — |
| S5-U04 | far reading sets left clearance (dropped: replaced by S5-U10) |  | auto | removed | — |
| S5-U05 | servo sweeps 60/75/90/105/120° and distance tracks a hand | IF-01 | manual | SIM=0, watch servo + telemetry | obstacle_cm follows the hand |
| S5-U06 | obstacle right: centre +17.5°, offset ≈ +7 cm |  | auto | `subsystem5_test.c` | PASS |
| S5-U07 | obstacle at scan edge: that side blocked, pass the other side |  | auto | `subsystem5_test.c` | PASS |
| S5-U08 | nothing in range: no obstacle, width 0 |  | auto | `subsystem5_test.c` | PASS |
| S5-U09 | profile matches a real object: width, centre, closest point | IF-01 | manual | SIM=0, place a box of known width at known spots, read the `S5: obstacle` line | values within a few cm / degrees |
| S5-U10 | obstacle ahead: both side clearances stay clear (own hits not counted) |  | auto | `subsystem5_test.c` | PASS |
| S5-U11 | obstacle ahead: centre 0°, width ≈ 4.2 cm, closest 24 cm |  | auto | `subsystem5_test.c` | PASS |
| S5-U12 | repeat reading at the same angle replaces the old one |  | auto | `subsystem5_test.c` | PASS |
| S5-U13 | decision: continue / turn left / turn right / reverse from the profile | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U14 | a close reading in the car's path triggers a stop; one beside it does not (dropped: replaced by S5-U17) | Stage 4 | auto | removed | — |
| S5-U15 | manoeuvre: stop, bypass, find line, align, verify, done | Stage 5 | auto | `subsystem5_test.c` | PASS |
| S5-U16 | three reverses at one spot, then gives up as boxed in | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U17 | driving: three close readings in a row stop the car; one bad echo does not | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-M07 | servo stays still until something is within 35 cm, then scans once and repeats only if still close | Stage 1 | manual | bench build, hand or box moved in from far away | `S5: STOP` near 35 cm, then a scan; no sweeping with a clear path |
| S5-M01 | box on a straight: car goes round it and rejoins the line going the same way | Stage 4, 5, IF-01, IF-03 | manual | SIM=0, floor; tune the distances in `avoidance.c` | mission 1 → 2 → 1, no contact |
| S5-M02 | box placed on the line, car at driving speed | Stage 4 | manual | SIM=0, floor, save the console output | car stops short of the box; `S5: STOP obstacle in path at x cm` is printed |
| S5-M03 | box beside the line, outside the car's path | Stage 4 | manual | SIM=0, floor | car keeps following |
| S5-M04 | dead end: both sides blocked | Stage 4 | manual | SIM=0, floor, clear space behind | up to 3 reverses, then mission 3, fault 4 |
| S5-M05 | box on a curve of the competition track | Stage 5 | manual | SIM=0, track | car rejoins the line, or stops with fault 3 |
| S5-M06 | servo swings on request | IF-01 | manual | `S5_ULTRASONIC_HW` = 1, SIM=1, run `scan_recorder.ps1`, click **Test servo** | sensor swings left and right twice, then returns to centre |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
