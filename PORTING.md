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
6. Ultrasonic/servo: servo sweeps 45/90/135°, `obstacle_cm` tracks a hand.
