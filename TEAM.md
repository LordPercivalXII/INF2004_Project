# Team and subsystems — INF2004 autonomous robot car

This firmware is the micro T-Kernel 3.0 port of the FreeRTOS robot car
(`LordPercivalXII/INF2004_Project`). See **`PORTING.md`** for how every
FreeRTOS construct was mapped and **`README.md`** for build and flash steps.

Rule: **only the owner edits a subsystem's folders.** Shared files
(`app_program/app.h`, `app_program/app_main.c`, `app_program/common/`,
`app_program/controller/`, `config/`, this file) change only by team
agreement.

Coding standard: **BARR-C:2018** for everything in `app_program/`.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Buddy 1 — telemetry: JSON snapshots + heartbeat on the console | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Buddy 2 — motion: motors, wheel encoders, per-wheel speed PID | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Buddy 3 — IR line tracking (FR7, NFR6) + Code 39 barcodes (FR8, FR9) | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | Buddy 4 — MPU6050 IMU: shock (FR10), hump height (FR11), tilt | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Buddy 5 — ultrasonic obstacle scanner on a servo | | | `app_program/subsystem5/` | `testing/subsystem5/` |
| — | Vehicle controller: mission state machine, avoidance (whole team) | | | `app_program/controller/` | `testing/integration/` |

## 2. Pins

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | S4 | MPU6050 INT (reserved, not used yet) |
| GP3 | | |
| GP4 | S4 | I2C0 SDA (kernel `iica`) |
| GP5 | S4 | I2C0 SCL (kernel `iica`) |
| GP6 | | |
| GP7 | | |
| GP8 | S2 | left motor PWM (slice 4A) |
| GP9 | S2 | left motor direction |
| GP10 | S2 | right motor PWM (slice 5A) |
| GP11 | S2 | right motor direction |
| GP12 | S2 | left encoder A (interrupt) |
| GP13 | S2 | left encoder B |
| GP14 | S2 | right encoder A (interrupt) |
| GP15 | S5 | scan servo PWM (slice 7B) |
| GP16 | S5 | ultrasonic echo (interrupt) |
| GP17 | S5 | ultrasonic trigger |
| GP18 | | |
| GP19 | S2 | right encoder B |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | S3 | left IR analog (ADC0) |
| GP27 | S3 | centre IR analog (ADC1) |
| GP28 | S3 | right IR digital: barcode edges (interrupt) + junction |

## 3. Peripherals

**Each peripheral is driven by exactly one task, its owning task.**

| Peripheral | Owner subsystem | Owning task | Pins / notes |
|---|---|---|---|
| ADC | S3 | `line_task` starts it; then only the TIMER alarm 3 interrupt reads it | GP26, GP27. Kernel `adca` driver **disabled** (`DEVCNF_USE_ADC 0`) |
| TIMER alarm 3 (IRQ 3) | S3 | `line_task` (1 kHz IR sampler) | |
| I2C0 | S4 | `imu_task` via kernel `iica` | GP4/GP5 set in `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h`; `DEVCNF_USE_IIC 1` |
| PWM slices 4, 5 | S2 | `motion_task` | GP8, GP10 (~1.9 kHz) |
| PWM slice 7 | S5 | `obstacle_task` | GP15 (50 Hz, 1 µs/count) |
| IO_BANK0 GPIO interrupt (IRQ 13) | *shared* | router `common/drv_gpio_irq.c`, installed by `usermain()` | callbacks: S2 GP12/GP14, S3 GP28, S5 GP16 |
| TIMER counter, PWM reset | *shared* | `common/drv_board.c` `board_init()` in `usermain()` | read-only µs clock for everyone |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4).
The bands follow the FreeRTOS priorities of the original (motion and
controller highest, telemetry lowest), so they are **not** in subsystem
order.

| Owner | Priority band | Tasks (name – priority – core – period) |
|---|---|---|
| S2 | 10–11 | `motion_task` – 10 – any – 20 ms (cyclic handler) |
| controller | 12–13 | `controller_task` – 12 – any – event-driven, 10 ms wait |
| S3 | 14–15 | `line_task` – 14 – any – 10 ms (cyclic); `barcode_task` – 15 – any – event-driven |
| S4 | 16–17 | `imu_task` – 16 – any – 10 ms (cyclic) |
| S5 | 18–19 | `obstacle_task` – 18 – any – free-running scan |
| S1 | 20–21 | `telemetry_task` – 20 – any – event-driven, 100 ms wait |

Interrupts (priority 2): GPIO router (IRQ 13), IR sampler TIMER alarm 3 (IRQ 3).

## 5. Interfaces between subsystems

The receiver creates the kernel object and publishes its ID in its header.

| ID | From → To | Mechanism | Object ID name | Data / meaning |
|---|---|---|---|---|
| IF-01 | S2, S3, S4, S5 → controller | message buffer (owned by controller), `TMO_POL` | `gh_event_mbf` via `controller_post_event()` | `system_event_t` (`common/robot_types.h`), 24 deep |
| IF-02 | S1 → controller | message buffer (owned by controller) | `gh_control_mbf` via `telemetry_submit_command()` | `control_command_t`, 12 deep |
| IF-03 | controller → S2 | message buffer (owned by S2) | `gh_motion_cmd_mbf` via `motion_send_command()` | `motion_command_t`, 12 deep; S2 keeps the newest |
| IF-04 | controller → S1 | message buffer (owned by S1) | `gh_telemetry_mbf` via `telemetry_publish()` | `telemetry_message_t`, 12 deep |
| IF-05 | controller → any | event flag (owned by controller) | `gh_system_flg` | `SYSTEM_BIT_READY/MISSION_RUN/OBSTACLE/FAULT/WIFI_UP` |
| IF-06 | MQTT (WiFi service task) → S3 | message buffer (owned by S3), `TMO_POL` | `gh_route_cfg_mbf` via `mqtt_utk_route_rx()` | route table text from `MQTT_ROUTE_TOPIC`, e.g. `A:L,B:R,C:S,D:U` (format in `subsystem3/route.c`) |

IF-01 line events carry `barcode` (decoded character, ASCII) once per read and, on a junction, `junction_action` (left/right/straight/U-turn) resolved by S3 from the stored character.

Internal (inside one subsystem): S3 `gh_barcode_edge_mbf` (edge ISR → `barcode_task`, 128 deep), S3 `gh_route_mtx` (route table + waypoint memory),
S3 `gh_barcode_result_mtx` + one-slot result (`barcode_task` → `line_task`),
S5 `gh_echo_mbf` (echo ISR → `obstacle_task`, 4 deep).

## 6. Kernel object budget

| Object | Limit | Ctrl | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 2 | 1 | 1 |
| Event flag | 16 | 1 | | | | | |
| Mutex | 12 | | | | 2 | | |
| Message buffer | 12 | 2 | 1 | 1 | 2 | | 1 |
| Cyclic handler | 12 | | | 1 | 1 | 1 | |

The kernel's `iica` driver also creates one event flag. Total: 7 tasks,
7 message buffers, 3 cyclic handlers — well inside `config/config.h`.
