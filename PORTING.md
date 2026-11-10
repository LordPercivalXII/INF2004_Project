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
