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

## "START does nothing": the firmware was a SIM=1 build

The field log shows `simulation=1` in the boot banner. START worked end
to end (`mqtt: RX ... "START"` -> `CTRL: START received` ->
`Countdown complete, mission STARTED` -> FOLLOW, with the distance
rising), but in a simulation build `motor_set()` is compiled out, so the
real wheels never turn. `SIM ?= 1` is the documented default
(`pico_rp2040.mk`, README), so a plain `make` builds the simulator.

**Build the real car with `make SIM=0`** (the binary name has no `_sim`
suffix and the banner says `simulation=0`).

So a simulation build is never mistaken for a broken START again:
- on START, a SIM=1 build prints `*** SIMULATION build (SIM=1): the
  motors are NOT driven ... ***`;
- MQTT byte 4 bit 7 is a simulation flag; the dashboard's status line
  shows "SIMULATION firmware: motors not driven (rebuild with make SIM=0)".

The simulated IMU injects a shock every 8 s. With the NFR10 shock-stop
that faulted the simulated car every 8 s (`FAULT code 5` in the log), so
`CTRL_SHOCK_STOP_ENABLED` now defaults to on for SIM=0 and off for SIM=1.

## IMU start-up calibration took minutes (or never finished)

Symptom: `S4: calibration waiting ... cal=56/200`, then `cal=10`, `22`,
`103`, `37`... with the car still, and START held with
`CTRL: START waiting for IMU calibration`.

Cause (`subsystem4/imu_analysis.c`, `imu_calibrate()`): the window had
to reach 200 consecutive samples whose peak-to-peak span stayed within
0.04 g on every axis. A single noisy sample anywhere in the 2 s restarted
the window from zero. A late sample (timing gap) also restarted it.
The logged still-car samples wander about 20 mg in Z.

Fix:
- **Noise:** per-axis standard deviation over the window <= 12 mg
  (Welford running mean/variance), instead of peak-to-peak <= 40 mg.
- **Movement:** a sample more than 0.10 g from the window mean restarts
  the window at once. Picking the car up or touching it still blocks
  calibration.
- **Level:** unchanged (X, Y within 0.15 g, Z 0.85..1.15 g).
- **Timing gaps:** a late sample is skipped, not treated as movement.
- **Bias:** the window mean, as before.
- **Waiting message:** now shows the window count and measured noise, e.g.
  `(window 140/200, noise 6 mg, limit 12 mg)`.

Host simulation (`testing/integration/test_imu_calibration_host.c`, 20
runs each, Gaussian noise on a still car):

| Noise (rms) | Old rule | New rule |
|---|---|---|
| 3-5 mg | 2.0-2.3 s | 2.0 s |
| 7 mg | 3.8-4.9 s | 2.0 s |
| 9 mg | never (0/20 within 10 min) | 2.0 s |

A car being handled, or tilted, never calibrates under either rule.

## Barcode turned the wrong way ("F = right" turned left)

The firmware chain is consistent end to end (`F:R` -> `BARCODE_CMD_RIGHT`
-> -90 deg -> left wheel forward, right wheel back = clockwise). The logs
show the robot was not using the dashboard's table: it received
`"A:L,B:R,C:S,D:U"` (here **B = right**). The dashboard always sends
`CLEAR,...`, so that was a stale retained message from another publisher.
The dashboard form only shows what it *would* send.

Now visible on the dashboard:
- **Route echo:** the robot publishes the table it is using, retained, on
  `inf2004/car/route/active` (e.g. `A:L,B:R,C:S,D:U,*:S`; `*` = any
  other character). It is sent after every change and every reconnect.
  The dashboard shows it under "Route table on the robot" and flags when
  it differs from the form.
- **Last junction decision:** telemetry bytes 5-6 (packet is now 7 B)
  carry the barcode used at the last junction, the action chosen and a
  junction counter, e.g. `#3: 'F' -> RIGHT`.

To clear a stale retained route: press **Publish route settings** once
the robot is connected (it is retained, so it replaces the old one), or
`mosquitto_pub -h <broker> -t inf2004/car/route -r -n`.

If the dashboard shows `'F' -> RIGHT` and the serial log shows
`CTRL: junction -> barcode 'F' -> turn -90 deg`, but the car physically
turns left, then the left/right wheel wiring is mirrored. Check that
`PIN_MOTOR_LEFT_*` and `PIN_ENCODER_LEFT_*` really are the left wheel.

## Barcode reads: capture robustness

The decoder already corrects threshold bias, normalises speed and reads
both directions. The capture around it lost reads:
- **Overflow:** 99 elements of line-tape junk before a barcode set
  `b_overflow` and the decode was skipped. Now the oldest bar/space pair
  is dropped instead, so the newest elements are always kept.
- **Late decode:** decoding waited for 150 ms-1 s of silence, so a
  barcode followed by more edges could decode after the junction had
  passed. Now a space wider than `BARCODE_QUIET_FACTOR` (8) x the
  narrowest element, after at least one complete `*X*`, triggers an
  immediate decode.
- **Sensor hint:** a failed read now prints `narrowest bar X us, space Y
  us: space/bar = R`. Code 39 narrow bars and spaces are printed equally
  wide, so R measures the sensor: about 1.0 is ideal; above about 2.5
  narrow bars vanish. Your logs show about 1.8 (13 ms bars, 23-25 ms
  spaces). Lower the sensor (TCRT5000-type modules focus at a few mm) or
  turn the GP28 pot towards black, and watch R come down.

Host test: `testing/integration/test_barcode_session_host.c` (early
decode at a quiet zone, junk before a barcode, 120 junk elements =
former overflow, 5 ms bar/space bias, partial code not decoded).

## WiFi stuck at "joined AP, no IP yet"

`lib/libwifi/.../cyw43_utk.c` only retried after a *negative* link status
(`FAILED`, `SSID not found`, `authentication rejected`). A join that got
as far as `JOIN (1)`/`NOIP (2)` while the hotspot never answered DHCP, or
a link that later fell to `DOWN (0)`, waited forever.

Fixes:
- **Watchdog:** any state other than UP for 15 s
  (`WIFI_LINK_TIMEOUT_US`) prints `wifi: no IP after 15 s ...; leaving
  and joining again`, calls `cyw43_wifi_leave()` and rejoins 1 s later.
- **Auth fallback:** after `authentication rejected`, the next join
  switches between WPA2 (`CYW43_AUTH_WPA2_MIXED_PSK`) and WPA2/WPA3
  transition (`CYW43_AUTH_WPA3_WPA2_AES_PSK`). The join line now shows
  which mode it is trying.

iPhone hotspot checklist:
- **Maximize Compatibility ON:** 2.4 GHz and WPA2; the Pico W is
  2.4 GHz only.
- **Keep the Personal Hotspot screen open while the robot joins:** iOS
  stops advertising the hotspot when nobody is connected.
- **SSID exactly as on the phone:** some logs show `Wicus Phangs 1Phone 17`
  (digit 1), which gives `SSID not found`.

## Junction detected late, turn executed seconds later

**Late detection.** A junction needs both line sensors >= 0.80 of
calibrated black for 2 cycles (20 ms). The boot sweep (1 s) only
calibrates if the car is swept across the line; otherwise the fixed
defaults (white 230, black 2200) are used, as in every log so far. A
tape that really reads 1500 counts then normalises to 0.64 and only
crosses 0.80 when the sensors sit squarely on it.

Fix (`line.c`): while a mission runs, each sensor's steady minimum and
maximum (held >= 3 cycles, so spikes are ignored) are tracked and used as
white/black once they span `IR_MIN_CONTRAST_COUNTS`. The console prints
`S3: line sensor left learned while driving: white W, black B counts`.
A boot sweep is still best: it makes the very first junction exact too.

**Late execution.** Two problems in `motion.c`:
1. **No breakaway power.** The TURN stops the forward drive at once, so
   the pivot starts from rest. The wheel PID uses the target speed as its
   feed-forward duty (0.12 m/s -> ~19 % PWM with P) and its integral adds
   only ~1 %/s, so a pivot that needs more torque sat for seconds. Now a
   wheel commanded to move but at rest (< 0.02 m/s) gets at least
   `MOTOR_KICK_DUTY` (0.40) in the commanded direction. The encoder stall
   watchdog still stops a truly blocked wheel.
2. **Straight correction fought turns.** The correction compared raw
   left/right speeds in every mode, so in a pivot (opposite directions)
   and while steering it pulled the wheels towards equal speed. It now
   applies only when both targets are equal.

Tuning: raise `MOTOR_KICK_DUTY` if a pivot still hesitates; lower it if
the car jerks at START.

Host test: `testing/integration/test_line_learn_host.c`.

## Junction turns: overshoot, U-turn spin, start latency, barcode reuse

**Overshoot / U-turn spinning ~360 deg / "turns left when it should turn
right".** `ctrl_junction_turn_done()` seeded the line memory with
`last_seen_error = +-1` (line fully off to the turn side). The line
sensors straddle the line, so a pivot that lands centred on the new
branch reads white-white. The "line between the sensors" steering then
kept turning towards the turn side for 400 ms, followed by the
full-speed lost-line spin (1.3 s, ~250 deg). A correct right turn could
carry on round onto the left branch; a U-turn became ~360 deg.

Now after a junction pivot:
- `last_seen_error = 0`: white-white means centred; drive straight onto
  the branch.
- If the line is really not there, a gentle search sweep runs
  (`CTRL_POSTTURN_*`: 0.12 m/s, first sweep ~45 deg towards the turn
  side, then alternating and widening) for up to 3 s, before the normal
  lost-line search takes over.

**Start latency.** The TURN's early wake-up now drives both motors into
the pivot at `MOTOR_KICK_DUTY` immediately, instead of zeroing them and
coasting until the next 20 ms cycle.

**Barcode dropped after execution.** Once a junction action has been
carried out (turn finished, or straight on), the decoded barcode is
cleared (`CTRL: barcode 'F' executed; cleared`), and the dashboard's
"Last barcode scanned" returns to `--`. S3 also no longer consumes a
stored barcode at a junction seen while no mission is running (car
carried by hand).

## Junction never detected (car drives straight through the crossing)

Video and dashboard: barcode `F` read, route `F:R`, "No junction yet".
The car crossed a full 4-way junction without S3 reporting it, so
nothing was executed; what happened later was line following and the
lost-line search, not a late junction action.

Cause: the black level learned while driving took the darkest steady
reading. When the ramp lifts the front, the IR sensors see no floor and
read near maximum, and the dense printed barcode is darker than the tape.
Either set "black" far darker than the tape, so a crossing normalised
below `LINE_JUNCTION_LEVEL` and was never confirmed.

Fixes (`line.c`, `robot_config.h`):
- A sensor's black level is learned only while the other sensor reads
  white (< 0.30). One sensor over the tape is normal following; both dark
  at once (junction, lifted front, barcode) no longer moves "black".
- `LINE_JUNCTION_LEVEL` 0.80 -> 0.70, `LINE_JUNCTION_RELEASE` 0.65 -> 0.55,
  for more margin.
- `[S3 junction] near miss: ...` is printed whenever both sensors get
  dark (>= 0.40) without a confirmed junction, with the percentages, raw
  counts and calibrated black, so a missed junction is never silent.

## Left/right wheels were mirrored ("F = right" turned left)

Serial log of the failing run:
```
[S3 route] barcode "F" (right-to-left) stored: 'F' -> RIGHT at next junction
S3: junction: 'F' -> RIGHT
CTRL: junction -> barcode 'F' -> turn -90 deg now
```
Barcode, route and decision were all correct. -90 deg drives the wheel
named LEFT forward and the one named RIGHT backward (clockwise); the car
pivoted left. So the motor + encoder on GP10/11 + GP12/13 (named LEFT)
drive the physical RIGHT wheel. Line following worked only because the
line sensors are mirrored the same way and the two cancelled.

Also explained by this: "B turns right / F turns left" earlier; the
encoder log calling the left wheel slow while the physical left wheel ran
fast; the avoidance bypass side (servo angles are physical).

Fix (`robot_config.h`, `drv_encoder.c`): `ROBOT_WHEELS_SWAPPED` (1 on
hardware, 0 in simulation) assigns the pins as the wheels really are:

| | LEFT (physical) | RIGHT (physical) |
|---|---|---|
| Motor pins | GP8/GP9, sign -1 | GP10/GP11, sign +1 |
| Encoder pins | GP14/GP19, B high = -1 | GP12/GP13, B high = +1 |

The direction sign and encoder convention move with their pins, so each
physical motor behaves exactly as before. `LINE_ERROR_SIGN` becomes -1
so line following is unchanged.

Check on the car: a junction mapped RIGHT pivots clockwise (seen from
above), and line following still tracks. If line following steers away
from the line, the line sensors were not mirrored after all: set
`LINE_ERROR_SIGN` back to 1.0f for the swapped case and report it.
