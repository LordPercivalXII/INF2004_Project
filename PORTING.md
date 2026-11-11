# FreeRTOS → micro T-Kernel 3.0 port notes

Source: `LordPercivalXII/INF2004_Project` (Pico SDK + FreeRTOS, CMake).
Target: `sirfonzie/INF2004_ProjectTemplate` (micro T-Kernel 3.0, dual-core
RP2040, make). All application code is in `app_program/` and follows
**BARR-C:2018**.

## Build

```sh
cd build_make
make -j8                    # USB console + WiFi, simulated sensors (SIM=1)
make SIM=0 -j8              # real hardware
make SIM=0 TEST=1 -j8       # real hardware + on-target self-tests
make CONSOLE=uart WIFI=none -j8   # no Pico SDK needed
```

`SIM` replaces the CMake option `ROBOT_SIMULATION` (default ON in the
original, default `SIM=1` here). It was added to `build_make/pico_rp2040.mk`
following the template README's "Adding your own option" steps, so the
image name gets a `_sim` suffix.

Verified: all of `make`, `SIM=0`, `TEST=1`, `SMP=0 SIM=0` (USB+WiFi, Pico
SDK 2.2.0) and the `CONSOLE=uart WIFI=none` variants build with **zero
warnings**; the app also compiles cleanly with `-Wall -Wextra -Wshadow`.
The Code 39 decoder was checked on a PC against the original on 200 000
randomly distorted barcodes (identical results). **Not yet run on a Pico**:
see "Check on hardware" below.

## File map

| FreeRTOS project | micro T-Kernel project |
|---|---|
| `src/main.c` | `app_program/app_main.c` (`usermain`) |
| `include/config.h` | `app_program/common/robot_config.h` |
| `include/common_types.h` | `app_program/common/robot_types.h` |
| `src/gpio_irq_router.c` | `app_program/common/drv_gpio_irq.c` |
| Pico SDK `gpio_*`, `pwm_*`, `time_us_32`, `busy_wait_us` | `app_program/common/drv_board.c` |
| `<math.h>` (`sqrtf`, `atan2f`, `sinf`, `tanf`, `fminf`…) | `app_program/common/rmath.c` |
| `src/controller/controller.c` | `app_program/controller/controller.c` |
| `src/buddy1_telemetry/` | `app_program/subsystem1/` (`telemetry.c`) |
| `src/buddy2_motion/` | `app_program/subsystem2/` (`motion.c`, `drv_motor.c`, `drv_encoder.c`) |
| `src/buddy3_line_barcode/` | `app_program/subsystem3/` (`line.c`, `barcode.c`, `code39.c`, `drv_ir_sampler.c`, `drv_barcode_sensor.c`) |
| `src/buddy4_imu/` | `app_program/subsystem4/` (`imu.c`, `imu_analysis.c`, `drv_mpu6050.c`) |
| `src/buddy5_obstacle/` | `app_program/subsystem5/` (`obstacle.c`, `obstacle_geometry.c`, `drv_ultrasonic.c`, `drv_servo.c`) |

## API mapping

| FreeRTOS | micro T-Kernel |
|---|---|
| `xTaskCreate` | `app_start_task()` → `tk_cre_tsk` + `tk_sta_tsk` |
| `vTaskDelayUntil` (periodic task) | `app_start_periodic_task()`: cyclic handler `tk_wup_tsk` + `tk_slp_tsk(TMO_FEVR)` |
| `vTaskDelay` | `tk_dly_tsk` |
| `xQueueCreate` (copy by value) | `tk_cre_mbf` (message buffer, copies bytes) |
| `xQueueSend(q, x, 0)` | `tk_snd_mbf(id, x, size, TMO_POL)` |
| `xQueueSendFromISR` + `portYIELD_FROM_ISR` | `tk_snd_mbf(.., TMO_POL)` (allowed in handlers; kernel dispatches on exit) |
| `xQueueReceive(q, x, t)` | `tk_rcv_mbf(id, x, t)` (returns size, or `E_TMOUT`) |
| `xQueueOverwrite` on a length-1 queue | one-slot struct guarded by a mutex (`tk_loc_mtx`/`tk_unl_mtx`) |
| `xEventGroupCreate/SetBits/ClearBits` | `tk_cre_flg` (`TA_WMUL`) / `tk_set_flg` / `tk_clr_flg(id, ~bits)` |
| `xTaskGetTickCount`, `to_ms_since_boot` | `board_millis()` → `tk_get_otm` |
| `configASSERT(create != NULL)` | object IDs checked `< E_OK`, reported by `app_check()` |
| `add_repeating_timer_us` (1 kHz) | RP2040 TIMER alarm 3 via `tk_def_int` + `EnableInt` |
| `gpio_set_irq_callback` (IO_IRQ_BANK0) | `gpio_irq_init()` / `gpio_irq_register()` via `tk_def_int` |
| `i2c_write_blocking` / `i2c_read_blocking` | kernel `iica` driver: `tk_swri_dev`, `TDN_I2C_EXEC` |
| `adc_read` | direct ADC registers from the sampler interrupt |
| `printf("%f")` | `tm_sprintf` + `telemetry_format_fixed()` (no `%f` in `tm_printf`) |
| `save_and_disable_interrupts` around encoder reads | single-writer 32-bit counters, each read atomic (DI/EI would not cover the other core) |

## Behaviour differences to know about

* **Tick is 10 ms, not 1 ms.** Periods 10/20 ms are kept exactly by cyclic
  handlers. The controller's event wait is 10 ms (was 2 ms); IR
  calibration samples every 10 ms (was 5 ms). Anything sub-10 ms (1 kHz
  sampling, barcode edge timing, echo width) is done in interrupts timed
  by the 1 MHz TIMER, as before.
* **Priorities** were remapped to micro T-Kernel bands (1 = highest),
  keeping the original order: motion, controller, line/barcode, IMU,
  obstacle, telemetry (TEAM.md §4). micro T-Kernel does not time-slice
  equal priorities, so each task has its own level.
* **Kernel configuration changed** (shared files):
  `config/config_device.h` — `DEVCNF_USE_IIC 1`, `DEVCNF_USE_ADC 0`;
  `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h` — I2C0 on GP4/GP5 (the
  template default GP8/GP9 clashes with the left motor).
* **PWM and TIMER resets.** The kernel only takes PWM out of reset when
  `USE_PTMR=1`; `board_init()` releases PWM and TIMER (never re-asserts).
* **Kept as in the original (on purpose):** in the controller, a barcode
  turn command is immediately followed by the normal line-steering command
  in the same event, so the turn only lasts until the next motion cycle.
  This is the original behaviour; change it in `ctrl_handle_line()` if the
  turn should be held.

## Check on hardware (first flash)

1. `make SIM=0 TEST=1` → all `[TEST]` lines PASS.
2. Telemetry JSON and heartbeat every second on the USB console.
3. Encoders: spin each wheel by hand; `distance` in telemetry changes sign
   correctly (original sign convention kept).
4. IR: during the 3 s calibration sweep the sensors across line and floor;
   verify `IR_ANALOG_BLACK_IS_HIGH` and `IR_BARCODE_BLACK_LEVEL`.
5. IMU: no `S4: MPU6050 not found on iica` message; a tap gives a shock.
6. Ultrasonic/servo: servo sweeps 60/75/90/105/120°, `obstacle_cm` tracks a hand.

## Tribunal 1 fixes (Week 8, post-tribunal)

Implemented from `CSC19_PostTri1` Versions B and C, kept on micro T-Kernel
3.0 (no FreeRTOS call was reintroduced). BARR-C:2018 as for the rest of
`app_program/`.

### FreeRTOS wording in the tribunal → what was implemented

| Tribunal decision (FreeRTOS terms) | micro T-Kernel implementation |
|---|---|
| `taskENTER_CRITICAL()` in tasks, `taskENTER_CRITICAL_FROM_ISR()` in ISRs | `crit_enter()`/`crit_exit()` (`common/crit_sect.c`): `DI()` + SIO spinlock 31. `DI`/`EI` save/restore PRIMASK, so the same call is legal in tasks and handlers; the spinlock covers the other core, which `DI` alone does not |
| 5-bit shared fault bitmask | `common/fault_mask.c`: `fault_mask_set/clear/take` inside the critical section; `take` = atomic snapshot-and-clear |
| `xQueueReceive(cmd_q_motion, 20 ms)` so HALT wakes Motion Control | cyclic handler kept (exact 20 ms PID period with a 10 ms tick); `motion_queue_command()` calls `tk_wup_tsk()` for STOP/BRAKE and the early wake-up zeroes both PWM outputs at once |
| Vehicle Controller polls `evt_q_terrain` first after `xQueueSelectFromSet()` | new 1-byte `gh_terrain_mbf`; `ctrl_poll_terrain()` runs at the top of each loop and before every queued IF-01 event |
| `xQueueOverwrite` 4-byte MQTT packet | `telemetry_encode()` → 4-byte message buffer → `mqtt_utk_telemetry_read()` |
| `(int8_t)(line_err_12b >> 4)` | same result without a shift of signed data (BARR-C): the 12-bit field is offset-binary, so the shift is unsigned |
| C bit-fields + `__attribute__((packed))` + `_Static_assert` | `common/robot_packed.h` (unsigned bit-fields only) |
| `#define USE_BIT_PACKING 1` | `make PACK=1` (default) / `PACK=0`; boot prints a `[PACK]` size table |

### Bit layouts that differ from the tribunal table, and why

| Queue | Tribunal | Implemented | Reason |
|---|---|---|---|
| `cmd_q_motion` | 2 B (mode:2, dir:2, speed:8, turn:4) | 4 B (mode:3, turn_id:5, arg_a:12, arg_b:12) | line steering sends a different speed to each wheel; there are 5 command types; turns need a matching ID |
| `nav_q_obstacle` | 2 B (6-bit width/clearance) | 9 B, 9-bit cm fields | 6 bits stop at 63 cm but the sensor reads 250 cm; the avoidance needs a signed bypass offset and the scan start time |
| `nav_q_line` + `cmd_q_barcode` | 2 B + 1 B | 2 B + 1 B + 1 B barcode character | as decided, plus the character for telemetry |
| MQTT | 4 B | 4 B as decided | the barcode no longer goes over MQTT (still on USB); byte 3 alternates hump/obstacle |

Turn IDs now count 1..31 (`ctrl_next_turn_id()`) to fit the 5-bit field.

### Measured sizes (arm-none-eabi-gcc 13.2, Cortex-M0+)

| Item | PACK=0 | PACK=1 |
|---|---|---|
| `cmd_q_motion` message / buffer reserved | 24 B / 336 B | 4 B / 96 B |
| MQTT message / buffer reserved | 204 B / 2496 B | 4 B / 96 B |
| MQTT payload on the wire | JSON, about 400 B | 4 B |
| IF-01 line message (sent every 10 ms) | 40 B (was 124 B before this change: whole union) | 16 B |
| IF-01 obstacle message | 60 B | 21 B |
| `evt_q_terrain` message / buffer (new) | 8 B / 96 B | 1 B / 64 B |

Message buffers come from the kernel's memory, not `.bss`, so
`arm-none-eabi-size` shows the same `.bss` for both builds; use the
`[PACK]` boot lines for the RAM comparison.

### Other changes

* **Shock stop (NFR10).** A new shock while following or avoiding enters
  recoverable fault 5: STOP is sent, the motion task is woken and the
  car resumes after `CTRL_SHOCK_STOP_MS` (2 s). Previously a shock only
  slowed the car; `CTRL_SHOCK_STOP_ENABLED 0` restores that. Expected
  path: IMU sample (≤10 ms) + controller wake + motion wake (<1 ms each),
  about 12 ms. `[NFR10]` console lines carry µs timestamps for the
  measurement.
* **junction_lock.** A 1-bit flag in the controller: after a junction is
  acted on, further junction flags are ignored until the car has moved
  `CTRL_JUNCTION_LOCK_M` (5 cm) past it, or past the end of its turn.
* **Servo settle.** `ULTRASONIC_SERVO_SETTLE_MS` 50 → 100 ms, applied only
  when the servo actually moved (the straight-ahead watch while driving
  no longer waits at all). Full scans already happen with the car stopped.
* **NO_ECHO.** An echo timeout (30 ms) or a full echo buffer in the GP16
  ISR sets `FAULT_BIT_NO_ECHO`. With nothing in range this bit is set
  routinely; it is reported, never acted on.
* **ISR counter races.** `barcode_sensor_take_counts/_overflow()` and
  `ultrasonic_take_drops()` read then cleared counters the ISR writes;
  now the ISR is the only writer and the task reports differences.

### Host tests

```sh
cd testing/integration && gcc -std=c99 -Wall -Wextra -I../../app_program/common \
    test_packing_host.c ../../app_program/common/robot_packed.c -o tp -lm && ./tp
cd app_program/subsystem1 && gcc -std=c99 -Wall -Wextra test_telemetry_codec.c \
    telemetry_codec.c ../common/robot_packed.c -o tc && ./tc
```

### Still to check on hardware

1. `[NFR10]` timestamps: tap the IMU while driving; stop must be ≤ 40 ms.
2. Junction lock on the real 20 mm tape at mission speed.
3. Full scan time with the 100 ms settle stays under 3 s (NFR8).
4. Dashboard shows mission, speed, line, hump, obstacle and fault from
   the binary packet.

## Barcode-routed junctions (turn on detection)

Flow, end to end:

1. **Store.** S3 decodes a Code 39 barcode and stores its character as the
   pending waypoint (`route_store_waypoint()`); a newer barcode replaces an
   unused one.
2. **Match.** At the next junction S3 looks the character up in the route
   table received over MQTT (`inf2004/car/route`, e.g. `A:L,B:R,C:S,D:U`)
   and consumes the waypoint (`route_take_junction_action()`).
3. **Default straight.** No barcode decoded, or a character the MQTT table
   does not map (and no `*:` default sent) -> STRAIGHT: the car keeps
   following the line.
4. **Execute at once.** The action travels with that same junction event.
   The controller commands the closed-loop pivot (left +90, right -90,
   U-turn 180) on that event; the old 10 mm drive to a pivot point
   (`JUNCTION_APPROACH_*`) is removed. A TURN also wakes the motion task,
   which zeroes the motors at once so the car does not roll past the
   junction; the pivot itself starts on the next 20 ms PID cycle.

`SYSTEM_BIT_JUNCTION_BUSY` (IF-05) is set while the controller's junction
lock is held (the turn, then 5 cm of travel). S3 does not consume a stored
barcode on a junction seen while it is set, so a repeat detection of the
same tape after the pivot cannot use up the barcode meant for the next
junction.

Console evidence: `[S3 route] barcode ... stored`, `S3: junction: 'A' -> LEFT`,
`CTRL: junction -> barcode 'A' -> turn 90 deg now t_us=...`.

Check on hardware: the barcode must finish decoding before the junction.
Decoding ends after the 150 ms end gap, so leave at least ~25 mm of white
between the last bar and the junction at 0.15 m/s; otherwise the
character is stored after the junction and used at the next one.

## Avoidance drift and encoder-stall fault (field log, 55 s)

Symptoms: during avoidance the car drifts right on legs meant to be
straight, with the left wheel visibly faster; one run ended in a
permanent `FAULT code 7` (encoder stall, left wheel, `ages=520/40 ms`).

Evidence in the log:
- only the left encoder rejects glitch edges (`left=68 right=0`);
- the left wheel reads slow or even backwards while driving forward
  (`left_m_s=-0.058 right_m_s=0.157`).

Cause: tick direction is channel B sampled on an A rising edge. A
glitch on A's falling edge looks like a rising edge with B in the
opposite state, so it counts a tick backwards and cancels a real one.
The left wheel reads slow, its PID drives it harder, and the car drifts
right. The encoder heading (right minus left ticks) is wrong too, so the
turns are off. If every falling edge glitches, the net count stops and
the stall watchdog fires while the wheel is turning.

Fixes:
1. `drv_encoder.c`: direction filter. A tick against the wheel's current
   direction is held back; 3 in a row are a real reversal (all counted),
   otherwise they are dropped and reported as
   `reversed glitches left=N right=N`.
2. `avoidance.c`: heading hold on every straight avoidance drive
   (REVERSE, LEG_OUT, LEG_PAST, SEARCH): P control on the encoder heading
   toward the leg's intended heading, trim up to +-0.04 m/s.
3. `obstacle_geometry.c`: the bypass offset now includes
   `OBSTACLE_CLEARANCE_MARGIN_CM` (5 cm); before, the car's side was
   planned exactly on the estimated obstacle edge.

Still check the hardware: a clean encoder should report almost no
rejected edges or reversed glitches. If the left counters keep climbing,
reseat the left encoder connector, keep its wires away from the motor
leads, and consider 4.7 kOhm external pull-ups on GP12/GP13 (the internal
pull-ups are about 50 kOhm).

## Dashboard: barcode over MQTT, and START that did nothing

**Barcode byte.** The MQTT packet is now 5 bytes: the tribunal's 4 plus
byte 4, the last decoded barcode character (ASCII 0x21..0x7E, 0 = none
since START). `dashboard/server.py` decodes 4- and 5-byte packets and puts
the character in the same `barcode` field the page already shows.

**START ignored.** `CONTROL_START` was only accepted in `IDLE`. After any
fault that does not auto-resume (encoder stall 7, line not reacquired 3,
boxed in 4, motion command 6, external 1), every START printed
`START ignored: state=3` and the dashboard timed out; only a reboot
cleared it, because nothing on the dashboard sends `CLEAR_FAULT`.
A START sent before the IMU had calibrated also waited silently.

Fixes:
- START from FAULT clears the fault (`ctrl_clear_fault()`, the operator's
  acknowledgement) and starts the 3 s countdown, if the IMU is healthy.
  If the cause is still there (a blocked wheel), the fault comes back.
- A START held for the IMU prints `CTRL: START waiting for IMU
  calibration` every 2 s.
- The packet's 3-bit state field uses two spare values: 4 = countdown
  running, 5 = START held for the IMU. The dashboard shows
  "Robot received START. Countdown running..." or "waiting for IMU
  calibration" instead of timing out.

If START still does nothing, the serial log shows where it stopped:
- no `mqtt: RX [inf2004/car/control] "START"`: the robot never received
  it (broker address or WiFi);
- `CTRL: START ignored ...`: received, with the reason;
- `CTRL: START received` but no `Countdown complete`: waiting for the IMU.
