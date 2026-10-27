# Subsystem 3 — test log

**Subsystem:** Buddy 3 — line tracking + barcode  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Code 39 round trip "*L*" | FR8 | auto | `subsystem3_test.c` | PASS |
| S3-U02 | Code 39 decodes crossed backwards | FR8 | auto | `subsystem3_test.c` | PASS |
| S3-U03 | corrupted symbol rejected or correct | NFR5 | auto | `subsystem3_test.c` | PASS |
| S3-U04 | payload → turn command | FR9 | auto | `subsystem3_test.c` | PASS |
| S3-U05 | line error sign + lost-line saturation | FR7 | auto | `subsystem3_test.c` | PASS |
| S3-U06 | 1 kHz IR sampling | NFR6 | manual | SIM=0; toggle a spare pin in `ir_sample_isr`, scope | 1.000 kHz |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
