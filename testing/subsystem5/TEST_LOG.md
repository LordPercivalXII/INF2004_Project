# Subsystem 5 — test log

**Subsystem:** Buddy 5 — adaptive servo ultrasonic scanning and obstacle profiling  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

Test IDs never change meaning and are never reused. Tests for features removed in the rewrite (branch `s5-rewrite`) stay listed as dropped. S5-U22, S5-U23 and S5-U28 were used for code that never reached `main`, so they are not reused.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | bypass side selection: the side needing the smaller sideways move | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U02 | hit at 24 cm detected (dropped: replaced by S5-U10) |  | auto | removed | — |
| S5-U03 | single-hit width ≈ beam footprint (dropped: replaced by S5-U11) |  | auto | removed | — |
| S5-U04 | far reading sets left clearance (dropped: replaced by S5-U10) |  | auto | removed | — |
| S5-U05 | servo sweeps 60/75/90/105/120° and distance tracks a hand | Stage 1, IF-01 | manual | SIM=0, car stopped in front of a box, watch servo + console | coarse scan at the 5 angles, then a fine scan over the box |
| S5-U06 | obstacle right: centre +17.5°, offset ≈ +7 cm | Stage 3 | auto | `subsystem5_test.c` | PASS |
| S5-U07 | obstacle at scan edge: that side blocked, pass the other side | Stage 3, 4 | auto | `subsystem5_test.c` | PASS |
| S5-U08 | nothing in range: no obstacle, width 0 | Stage 3 | auto | `subsystem5_test.c` | PASS |
| S5-U09 | profile matches a real object: width, centre, closest point | Stage 3, IF-01 | manual | SIM=0, place a box of known width at known spots, read the `S5: obstacle` line | values within a few cm / degrees |
| S5-U10 | obstacle ahead: both side clearances stay clear (own hits not counted) | Stage 3 | auto | `subsystem5_test.c` | PASS |
| S5-U11 | obstacle ahead: centre 0°, width ≈ 4.2 cm, closest 24 cm | Stage 3 | auto | `subsystem5_test.c` | PASS |
| S5-U12 | repeat reading at the same angle replaces the old one | Stage 2 | auto | `subsystem5_test.c` | PASS |
| S5-U13 | decision: continue / turn left / turn right / reverse from the profile | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U14 | a close reading in the car's path triggers a stop; one beside it does not (dropped: replaced by S5-U17) | Stage 4 | auto | removed | — |
| S5-U15 | manoeuvre: stop, bypass, find line, align, verify, done | Stage 5 | auto | `subsystem5_test.c` | PASS |
| S5-U16 | three reverses at one spot, then gives up as boxed in | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U17 | driving: three close readings in a row stop the car; one bad echo does not | Stage 4 | auto | `subsystem5_test.c` | PASS |
| S5-U18 | passing: side readings measure the obstacle's thickness (dropped: removed in the rewrite) | Stage 3, 5 | auto | removed | — |
| S5-U19 | turns aim at a heading: a short turn-out is undone exactly | Stage 5 | auto | `subsystem5_test.c` | PASS |
| S5-U20 | something close ahead while going round: carry on when clear, else scan a second obstacle (dropped: removed in the rewrite; see S5-U34) | Stage 5 | auto | removed | — |
| S5-U21 | driving: servo angle follows the steering (dropped: removed in the rewrite) | Stage 1 | auto | removed | — |
| S5-U24 | shallow box: rear edge bounded from the side readings (dropped: removed in the rewrite) | Stage 3, 5 | auto | removed | — |
| S5-U25 | turning hard: one reading within 12 cm stops the car (dropped: removed in the rewrite) | Stage 1 | auto | removed | — |
| S5-U26 | sensor aim from the wheel speeds (dropped: removed in the rewrite) | Stage 1 | auto | removed | — |
| S5-U27 | second obstacle scanned while driving past (dropped: removed in the rewrite) | Stage 4, 5 | auto | removed | — |
| S5-U29 | two fine sweeps keep the nearer reading (dropped: removed in the rewrite) | Stage 3 | auto | removed | — |
| S5-U30 | driving: a missed echo (sensor maximum) neither counts nor restarts the stop count | Stage 1, 4 | auto | `subsystem5_test.c` | PASS |
| S5-U31 | fine scan region: from one coarse step before the first hit to one after the last, inside 60..120°; none if the coarse scan hit nothing | Stage 2 | auto | `subsystem5_test.c` | PASS |
| S5-U32 | a real scan with the box at 28-30 cm: a 1 cm wobble does not change the box or the decision (the box is every reading within 10 cm of its nearest one) | Stage 3, 4 | auto | `subsystem5_test.c` | PASS |
| S5-U33 | too close for the 60° way out: backs up just enough before turning out | Stage 4, 5 | auto | `subsystem5_test.c` | PASS |
| S5-U34 | going round: a reading within 10 cm ahead stops the car and ends the manoeuvre (blocked); 40 cm does not | Stage 5 | auto | `subsystem5_test.c` | PASS |
| S5-U35 | line never found: two full sweeps, then "line not found" | Stage 5 | auto | `subsystem5_test.c` | PASS |
| S5-U36 | confirming the line after the bypass: something within 20 cm ahead is a new obstacle, so stop and scan it | Stage 4, 5 | auto | `subsystem5_test.c` | PASS |
| S5-M01 | box on a straight: car goes round it and rejoins the line going the same way | Stage 4, 5, IF-01, IF-03 | manual | SIM=0, floor; tune the distances in `avoidance.c` | mission 1 → 2 → 1, no contact |
| S5-M02 | box placed on the line, car at driving speed | Stage 1, 4 | manual | SIM=0, floor, save the console output | car stops short of the box; `S5: STOP obstacle in path at x cm` with x about 15-20 |
| S5-M03 | box beside the line, outside the car's path | Stage 4 | manual | SIM=0, floor | car keeps following, or stops, scans and prints `CONTINUE` |
| S5-M04 | dead end: both sides blocked | Stage 4 | manual | SIM=0, floor, clear space behind | up to 3 reverses, then mission 3, fault 4 |
| S5-M05 | box on a curve of the competition track | Stage 5 | manual | SIM=0, track | car rejoins the line, or stops with fault 3 |
| S5-M06 | servo swings on request (dropped: the bench servo test was removed in the rewrite) | IF-01 | manual | removed | — |
| S5-M07 | servo stays straight ahead while driving; scans only once the car has stopped for an obstacle, and never while the car is moving | Stage 1, 2 | manual | SIM=0, floor, box on the line; watch the servo | no sweeping while following; one coarse + fine scan after `S5: STOP`; after `avoid done` the servo stays ahead |
| S5-M08 | while driving past, the thickness is printed (dropped: removed in the rewrite) | Stage 3, 5 | manual | removed | — |
| S5-M09 | a second box placed in the car's path while it goes round the first | Stage 5 | manual | SIM=0, floor | car stops short of it (`S5: avoid blocked ahead`, fault 4); no contact |
| S5-M10 | following a curve, the sensor looks into it (dropped: removed in the rewrite) | Stage 1 | manual | removed | — |
| S5-M11 | box just past a tight curve or junction, sensor looking into the turn (dropped: removed in the rewrite) | Stage 1 | manual | removed | — |
| S5-M12 | a second box on the line just after the point where the car rejoins it | Stage 4, 5 | manual | SIM=0, floor; second box 15-25 cm past the first box's far side | car rejoins, stops for the second box (`S5: avoid WAIT_SCAN`), scans it and goes round it; no contact |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
