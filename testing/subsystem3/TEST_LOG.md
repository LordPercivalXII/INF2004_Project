# Subsystem 3 — test log

**Subsystem:** Buddy 3 — line tracking + barcode  ·  **Student ID:** 2500197  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

Printable test barcodes: `code39_test_sheet.pdf` (A B C D E F L R U 1 7 X, narrow 4 mm; regenerate with `make_code39_sheet.py`).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Code 39 read left-to-right | FR8 | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | Code 39 read right-to-left, direction reported | FR8, req. 1 | auto | `subsystem3_test.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | corrupted symbol rejected or correct | NFR5 | auto | `subsystem3_test.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | punctuation characters both ways | FR8 | auto | `subsystem3_test.c` | `[TEST] S3-U04 PASS` |
| S3-U05 | both edge sensors grey = centred, error 0 | FR7 | auto | `subsystem3_test.c` | `[TEST] S3-U05 PASS` |
| S3-U17 | drift left/right gives -/+ error; lost line steers to last side | FR7 | auto | `subsystem3_test.c` | `[TEST] S3-U17 PASS` |
| S3-U18 | junction after 2 both-black cycles, reported once per crossing | junction | auto | `subsystem3_test.c` | `[TEST] S3-U18 PASS` |
| S3-U06 | empty route table → straight | req. 4 | auto | `subsystem3_test.c` | `[TEST] S3-U06 PASS` |
| S3-U07 | `A:L,B:R,C:S,D:U` → left/right/straight/U-turn | req. 4 | auto | `subsystem3_test.c` | `[TEST] S3-U07 PASS` |
| S3-U08 | JSON payload; cumulative updates | req. 4 | auto | `subsystem3_test.c` | `[TEST] S3-U08 PASS` |
| S3-U09 | bad entries rejected, table unchanged | req. 4 | auto | `subsystem3_test.c` | `[TEST] S3-U09 PASS` |
| S3-U10 | `CLEAR` and default action `*` | req. 4 | auto | `subsystem3_test.c` | `[TEST] S3-U10 PASS` |
| S3-U11 | route message reaches line_task via MQTT hook (SIM=1) | IF-06 | auto | `subsystem3_test.c` | `[TEST] S3-U11 PASS` |
| S3-U12 | 1 kHz IR sampling | NFR6 | manual | SIM=0; toggle a spare pin in `ir_sample_isr`, scope | 1.000 kHz |
| S3-U13 | microsecond TIMER runs at 1 MHz | NFR5, NFR6 | auto | `subsystem3_test.c` | `[TEST] S3-U13 PASS` |
| S3-U14 | edge-biased scan + stray edges still decodes | FR8, NFR5 | auto | `subsystem3_test.c` | `[TEST] S3-U14 PASS` |
| S3-U15 | stretched first bar, right-to-left `F` decodes | FR8 | auto | `subsystem3_test.c` | `[TEST] S3-U15 PASS` |
| S3-U16 | speed varying 50% during the scan, right-to-left `R` decodes | FR8 | auto | `subsystem3_test.c` | `[TEST] S3-U16 PASS` |
| S3-M05 | line sensor hand check (`S3_LINE_HW 1`, `S3_LINE_DEBUG 1`, SIM=1) | FR7 | manual | slide car sideways over the line | centred: L, M ~50%, error ~0; car right of line: error negative; lifted/off line: LOST; on a cross: JUNCTION? |
| S3-M01 | route table from a real broker | IF-06, req. 4 | manual | `NET=lwip MQTT=1` build; `mosquitto_pub -r -t inf2004/car/route -m "A:L,B:R,C:S,D:U"` | console: `S3: route map: AL BR CS DU (others STRAIGHT)` |
| S3-M02 | barcode both ways on the track, every sheet character | req. 1, 2 | manual | push car over each barcode of `code39_test_sheet.pdf` forwards, then backwards | `S3: barcode "A" (left-to-right / right-to-left) stored` |
| S3-M03 | car keeps following, acts at junction | req. 3, 4 | manual | track: barcode `*B*` then a cross junction | no turn at barcode; `S3: junction: 'B' -> RIGHT`, car turns right |
| S3-M04 | remap between barcode and junction | req. 4 | manual | read `*D*`, publish `D:L` before the junction | car turns left |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| ≈2026-10-28 | S3-U13 | n/r (dev branch) | 2500197 | FAIL | `logs/2026-10-28_barcode_edge_dump.txt`, `logs/2026-10-28_barcode_scan_A_before_fix.txt` | Element widths implausible (0–1 µs bursts, gaps not matching the telemetry clock): microsecond timer not counting at 1 MHz. Fixed by the timer fix; re-checked in the next row. |
| ≈2026-10-28 | S3-U13 | n/r (dev branch) | 2500197 | PASS (indirect) | serial console, barcode debug dump (not archived) | After the timer fix, a `white lasted 9822286 us` gap spanned t=284 s→293 s on the telemetry clock, so µs timing is correct. The `[TEST] S3-U13` line itself was not captured; re-run with `make TEST=1`. |
| ≈2026-11-02 | S3-M02 | n/r (≈ `16e4265`) | 2500197 | FAIL | `logs/2026-11-02_barcode_hand_scan_A.txt` | `*A*` left-to-right: 9 good decodes, each routed (`'A' -> LEFT at next junction`, then `junction: 'A' -> LEFT`). Right-to-left `A` and `*F*` (F172/F188/F197) failed. Causes: slow scans split by the 150 ms end gap; first/last bar stretched up to 6× by hand speed; narrow bars lost to comparator chatter; F printout's last narrow gap not seen. Fixed (scan kept open up to 1 s, outermost bars forced narrow, chatter-only bars restored); new auto test S3-U15; printable `code39_test_sheet.pdf` added. 6 of the failing scans from this log decode as `A` after the fix (host replay). |
| ≈2026-11-02 | S3-M02 | n/r (dev branch) | 2500197 | FAIL | serial console, hand-swipe scans (not archived) | Still inconsistent at varying hand speed. Each scan began with a multi-second BLACK and ended `now BLACK`: sensor lifted off the paper reads black. Stray-element trimming removed pairs only, so a scan ending on white could never reach an odd (29, 39 …) element count. Glitch edges now 0 after pot adjustment; bars read longer than spaces. Fixed trimming and speed tolerance; new auto test S3-U16 (speed varying 50%). |
| ≈2026-11-03 | S3-M02 | n/r (≈ `ebc2700`) | 2500197 | PASS (with limits) | serial console, hand-swipe scans (not archived) | Owner satisfied with read output in both directions; limits noted: depends on sensor position/height and scanning range. Next: confirm on the moving car. |
| ≈2026-11-03 | S3-U14, S3-U15, S3-U16 | dev working copy | Claude (host PC) | PASS (host) | host build of `code39.c` + test logic | Logic of the new tests run on a PC against the firmware decoder; not an on-target run. Benchmark: 1.18 M synthetic reads, 0 wrong characters, 0 wrong directions. All firmware builds 0 warnings. |
| ≈2026-11-03 | S3-M05 | n/r | 2500197 | FAIL (setup) | photos of sensor bracket (not archived) | Line ≈1.8 cm wide; sensors ≈2.2–2.5 cm apart, each 2–4 mm outside a line edge, so both read white when centred and "centred" looks like "lost". Action: slide each board ≈3 mm inwards so each sensor sits on an edge. |
| ≈2026-11-03 | S3-U05, S3-U17, S3-U18 | dev working copy | Claude (host PC) | PASS (host) | host build of `line.c` | New line-error and junction tests pass on a PC, incl. re-arm at 0.52/0.52 after raising `LINE_JUNCTION_RELEASE` to 0.65. Not an on-target run. |
| ≈2026-11-10 | S3-M01 | n/r (≈ `8022d62`) | 2500197 | FAIL | serial console, real-car runs (not archived) | Car used `A:L,B:R,C:S,D:U` (not the dashboard's `CLEAR,…` message): stale retained route on the broker; in other runs no route arrived, so every junction went straight. Dashboard form defaults were also wrong (A = Straight). Fixed: dashboard defaults, robot echoes its active table on `inf2004/car/route/active`, telemetry shows last junction decision. |
| ≈2026-11-10 | S3-M03 | n/r | 2500197 | FAIL | dashboard screenshot + video (not archived) | Barcode `F` read and route `F:R` loaded, but dashboard showed "No junction yet": junction not detected, car acted well after the crossing. Junction detection re-implemented (`e514a4d`). |
| ≈2026-11-11 | S3-M01 | n/r | 2500197 | PASS | serial console, real-car run (not archived) | Route from broker applied: `[S3 route] barcode "F" (right-to-left) stored: 'F' -> RIGHT at next junction`, `S3: junction: 'F' -> RIGHT`. |
| ≈2026-11-11 | S3-M03 | n/r | 2500197 | FAIL (not S3) | serial console, real-car run (not archived) | S3 side correct: barcode `F` right-to-left, `junction: 'F' -> RIGHT`, controller `turn -90 deg now` at the junction, barcode cleared after. Car physically turned left: motor/encoder pins named LEFT drive the physical right wheel (S2). Fixed with `ROBOT_WHEELS_SWAPPED`; `LINE_ERROR_SIGN` flipped to match. |
| ≈2026-11-11 | S3-M03 | ≈ `0eb1d0e` | 2500197 | PARTIAL | on-car run, observed by tester | "Major improvement": turns the correct way at the junction, but ends facing slightly away from the line; car must advance ≈13.5 cm (sensor-to-axle distance) before pivoting. Fixed in controller; commit `0eb1d0e` "1st level junction working state". |
| 2026-11-20 | S3-M03 | `1dd3f74` | 2500197 | PASS | <https://youtu.be/oEcwkz3duA4> | RIGHT barcode read, car turns right at the junction and resumes. Also INT-J1 evidence. |
| 2026-11-20 | S3-M03 | `1dd3f74` | 2500197 | PASS | <https://youtu.be/rHV7J_s4asU> | LEFT barcode read, car turns left at the junction and resumes. Also INT-J1 evidence. |
| 2026-11-20 | S3-M03 | `1dd3f74` | 2500197 | PASS | <https://youtu.be/jiD_DmmuEf0> | U-turn barcode read, car U-turns at the junction. Also INT-J1 evidence. |

Notes on the rows above (added 2026-11-24): rows marked ≈ were recorded after the fact from development notes; the date is the nearest
subsystem 3 commit and "n/r" means the commit was not recorded at the time. Console output was not saved at the time; the files in `logs/` were recovered later from copies of the console output, and rows marked "not archived" have no surviving copy. Still to do for a complete record: flash `make TEST=1` at the hand-in commit and save the console to
`logs/YYYY-MM-DD_unit.txt` (S3-U01 to S3-U18), scope S3-U12 (1 kHz), and run S3-M04 (remap `D` before the junction).
