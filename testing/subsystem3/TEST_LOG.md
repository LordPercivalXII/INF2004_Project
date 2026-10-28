# Subsystem 3 — test log

**Subsystem:** Buddy 3 — line tracking + barcode  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Code 39 read left-to-right | FR8 | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | Code 39 read right-to-left, direction reported | FR8, req. 1 | auto | `subsystem3_test.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | corrupted symbol rejected or correct | NFR5 | auto | `subsystem3_test.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | punctuation characters both ways | FR8 | auto | `subsystem3_test.c` | `[TEST] S3-U04 PASS` |
| S3-U05 | line error sign + lost-line saturation | FR7 | auto | `subsystem3_test.c` | `[TEST] S3-U05 PASS` |
| S3-U06 | empty route table → straight | req. 4 | auto | `subsystem3_test.c` | `[TEST] S3-U06 PASS` |
| S3-U07 | `A:L,B:R,C:S,D:U` → left/right/straight/U-turn | req. 4 | auto | `subsystem3_test.c` | `[TEST] S3-U07 PASS` |
| S3-U08 | JSON payload; cumulative updates | req. 4 | auto | `subsystem3_test.c` | `[TEST] S3-U08 PASS` |
| S3-U09 | bad entries rejected, table unchanged | req. 4 | auto | `subsystem3_test.c` | `[TEST] S3-U09 PASS` |
| S3-U10 | `CLEAR` and default action `*` | req. 4 | auto | `subsystem3_test.c` | `[TEST] S3-U10 PASS` |
| S3-U11 | route message reaches line_task via MQTT hook (SIM=1) | IF-06 | auto | `subsystem3_test.c` | `[TEST] S3-U11 PASS` |
| S3-U12 | 1 kHz IR sampling | NFR6 | manual | SIM=0; toggle a spare pin in `ir_sample_isr`, scope | 1.000 kHz |
| S3-U13 | microsecond TIMER runs at 1 MHz | NFR5, NFR6 | auto | `subsystem3_test.c` | `[TEST] S3-U13 PASS` |
| S3-U14 | edge-biased scan + stray edges still decodes | FR8, NFR5 | auto | `subsystem3_test.c` | `[TEST] S3-U14 PASS` |
| S3-M01 | route table from a real broker | IF-06, req. 4 | manual | `NET=lwip MQTT=1` build; `mosquitto_pub -r -t inf2004/car/route -m "A:L,B:R,C:S,D:U"` | console: `S3: route map: AL BR CS DU (others STRAIGHT)` |
| S3-M02 | barcode both ways on the track | req. 1, 2 | manual | push car over `*A*` forwards, then backwards | `S3: barcode "A" (left-to-right / right-to-left) stored` |
| S3-M03 | car keeps following, acts at junction | req. 3, 4 | manual | track: barcode `*B*` then a cross junction | no turn at barcode; `S3: junction: 'B' -> RIGHT`, car turns right |
| S3-M04 | remap between barcode and junction | req. 4 | manual | read `*D*`, publish `D:L` before the junction | car turns left |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
