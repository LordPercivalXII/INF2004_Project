# Subsystem 5 — test log

**Subsystem:** Buddy 5 — adaptive servo ultrasonic scanning and obstacle profiling  ·  **Student ID:** 2502371  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.


## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | bypass side selection: the side needing the smaller sideways move | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U05 | servo sweeps 60/75/90/105/120° and distance tracks a hand | Stage 1, IF-01 | manual | SIM=0, car stopped in front of a box, watch servo + console | coarse scan at the 5 angles, then a fine scan over the box |
| S5-U06 | obstacle right: centre +17.5°, offset ≈ +7 cm | Stage 3 | auto | `subsystem5_test.c` | PASS |
| S5-U07 | obstacle at scan edge: that side blocked, pass the other side | Stage 3, 4 | auto | `subsystem5_test.c` | PASS |
| S5-U08 | nothing in range: no obstacle, width 0 | Stage 3 | auto | `subsystem5_test.c` | PASS |
| S5-U09 | profile matches a real object: width, centre, closest point | Stage 3, IF-01 | manual | SIM=0, place a box of known width at known spots, read the `S5: obstacle` line | values within a few cm / degrees |
| S5-U10 | obstacle ahead: both side clearances stay clear (own hits not counted) | Stage 3 | auto | `subsystem5_test.c` | PASS |
| S5-U11 | obstacle ahead: centre 0°, width ≈ 4.2 cm, closest 24 cm | Stage 3 | auto | `subsystem5_test.c` | PASS |
| S5-U12 | repeat reading at the same angle replaces the old one | Stage 2 | auto | `subsystem5_test.c` | PASS |
| S5-U13 | decision: continue / turn left / turn right / reverse (too close) from the profile ("no room on both sides" moved to S5-U44: a box past the scan's edges no longer reverses) | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U16 | three reverses at one spot, then gives up as boxed in | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U17 | driving: three close readings in a row stop the car; one bad echo does not | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U19 | turns aim at a heading: a short turn-out is undone exactly | Stage 5 | auto | `subsystem5_test.c` | PASS |
| S5-U30 | driving: a missed echo (sensor maximum) neither counts nor restarts the stop count | Stage 1, 4 | auto | `subsystem5_test.c` | PASS |
| S5-U31 | fine scan region: from one coarse step before the first hit to one after the last, inside 60..120°; none if the coarse scan hit nothing | Stage 2 | auto | `subsystem5_test.c` | PASS |
| S5-U32 | a real scan with the box at 28-30 cm: a 1 cm wobble does not change the box or the decision (the box is every reading within 10 cm of its nearest one) | Stage 3, 4 | auto | `subsystem5_test.c` | PASS |
| S5-U33 | too close for the 60° way out: backs up just enough (keeping 4 cm clear of the box) before turning out | Stage 4, 5 | auto | `subsystem5_test.c` | PASS |
| S5-U35 | line never found: two full sweeps, then "line not found" | Stage 5 | auto | `subsystem5_test.c` | PASS |
| S5-U36 | confirming the line after the bypass: something within 20 cm ahead is a new obstacle, so stop and scan it | Stage 4, 5 | auto | `subsystem5_test.c` | PASS |
| S5-U37 | going round: within 9 cm ahead stops and waits (BLOCKED), 40 cm does not; 3 readings beyond 15 cm carry on with the same step (a missed echo neither counts nor restarts, Tribunal 1 A5), and the bypass still finishes | Stage 5 | auto | `subsystem5_test.c` | PASS |
| S5-U38 | blocked: a reading within 15 cm restarts the clear count; still there after 8 s, gives up (blocked, fault 4), not before | Stage 5 | auto | `subsystem5_test.c` | PASS |
| S5-U39 | turning: within 9 cm ahead stops the turn, 12 cm does not; when clear the turn finishes only what is left (40 of 60°) | Stage 5 | auto | `subsystem5_test.c` | PASS |
| S5-U40 | after backing up the scan looks to 60 cm instead of 40: a 40 cm wide box at 46 cm is seen (a normal scan misses it) and gone round; only a scan after a back-up looks further | Stage 3, 4 | auto | `subsystem5_test.c` | PASS |
| S5-U44 | a box running past the scan's edges (+-30 deg) at a safe distance is not a dead end: unseen is not blocked, so the car turns (tie: left) with the offset from the last seen reading instead of reversing 3 times and faulting boxed in; too close (12 cm) still reverses | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-M01 | box on a straight: car goes round it and rejoins the line going the same way | Stage 4, 5, IF-01, IF-03 | manual | SIM=0, floor; tune the distances in `avoidance.c` | mission 1 → 2 → 1, no contact |
| S5-M02 | box placed on the line, car at driving speed | Stage 1, 4 | manual | SIM=0, floor, save the console output | car stops short of the box; `S5: STOP obstacle in path at x cm` with x about 15-20 |
| S5-M03 | box beside the line, outside the car's path | Stage 4 | manual | SIM=0, floor | car keeps following, or stops, scans and prints `CONTINUE` |
| S5-M04 | dead end: both sides blocked | Stage 4 | manual | SIM=0, floor, clear space behind | up to 3 reverses, then mission 3, fault 4 |
| S5-M05 | box on a curve of the competition track | Stage 5 | manual | SIM=0, track | car rejoins the line, or stops with fault 3 |
| S5-M07 | servo stays straight ahead while driving; scans only once the car has stopped for an obstacle, and never while the car is moving | Stage 1, 2 | manual | SIM=0, floor, box on the line; watch the servo | no sweeping while following; one coarse + fine scan after `S5: STOP`; after `avoid done` the servo stays ahead |
| S5-M09 | a second box placed in the car's path while it goes round the first | Stage 5 | manual | SIM=0, floor | car stops short of it (`S5: avoid blocked ahead`, fault 4); no contact |
| S5-M12 | a second box on the line just after the point where the car rejoins it | Stage 4, 5 | manual | SIM=0, floor; second box 15-25 cm past the first box's far side | car rejoins, stops for the second box (`S5: avoid WAIT_SCAN`), scans it and goes round it; no contact |
| S5-M13 | hand put in front of the car while it goes round a box, then taken away after ~2 s; repeat during a turn | Stage 5 | manual | SIM=0, floor, save the console output | `S5: avoid BLOCKED`, car stays stopped while the hand is there, then carries on the same step and rejoins the line; no fault. Hand left for 8 s: `S5: avoid blocked ahead`, mission 3, fault 4 |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-20 | S5-M02 | `7ea61ec` + uncommitted | | FAIL | Serial: STOP at 11, 13, 14, 15 cm | Expected 15-20 cm. Runs 2026-11-18 to 2026-11-20. |
| 2026-11-20 | S5-U09 | `7ea61ec` + uncommitted | | FAIL | Serial: widths 13.6, 14.2, 14.9, 15.6, 16.2, 20.4, 22.4 cm | Box is 14.3 cm. Runs 2026-11-18 to 2026-11-20. |
| 2026-11-20 | S5-M01 | `7ea61ec` + uncommitted | | FAIL | Tester report | Car does not rejoin the line properly after the bypass. |
| 2026-11-20 | S5-M07 | `7ea61ec` + uncommitted | | FAIL | Serial: scans while IDLE, no STOP | Before the gate fix. Not confirmed after the change. |
| 2026-11-24 | S5-M01 | `e61bdc4` + uncommitted | | PASS | `logs/2026-11-24_avoidance_rejoin.txt` | Successful bypass: `avoid done`, then FOLLOW with line errors 0.055 and 0.283. |
| 2026-11-24 | S5-M01 | `7f5eef0` + uncommitted (flashed build unverified) | | PASS | `logs/2026-11-24_parallel_rejoin_attempt.txt` | User-marked PASS for the avoidance run. Capture shows turn-out to 16 cm and turn-in/search; it ends during SWEEP before line reacquisition is shown. |
