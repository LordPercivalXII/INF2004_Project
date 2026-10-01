# INF2004 Autonomous Robotic Car

Modular RP2040 application using the Raspberry Pi Pico SDK and FreeRTOS. The Vehicle Controller is the sole owner of mission state; subsystems exchange copied messages through FreeRTOS queues and the system event group.

## Build

Install the Raspberry Pi Pico SDK, an ARM GCC toolchain, CMake, Ninja or Make, and an official FreeRTOS-Kernel checkout. In PowerShell, set the SDK and kernel paths before configuring:

```powershell
$env:PICO_SDK_PATH = "C:\path\to\pico-sdk"
$env:FREERTOS_KERNEL_PATH = "C:\path\to\FreeRTOS-Kernel"
cmake -S . -B build -G Ninja -DROBOT_SIMULATION=ON
cmake --build build
```

Set `ROBOT_SIMULATION=OFF` for sensor and actuator I/O. The default is simulation so task timing, queue traffic, and controller behavior can be exercised without energizing motors. Flash `build/robot_car.uf2` to the Pico. Telemetry is emitted as JSON lines over USB CDC.

## Module boundaries

- `controller/` owns mission state and translates line, IMU, and obstacle events into motion commands.
- `buddy1_telemetry/` consumes telemetry and provides a queue-safe command submission API.
- `buddy2_motion/` owns motor PWM, encoder capture, and the speed-control loop.
- `buddy3_line_barcode/` samples the left/center line sensors and the right barcode sensor.
- `buddy4_imu/` reads MPU-6050-compatible I2C registers and reports shock, turn rate, and an approximate hump height.
- `buddy5_obstacle/` performs centered range checks, coarse/fine servo scans, width estimation, and clearance selection.
- `gpio_irq_router.c` multiplexes the RP2040's single GPIO IRQ callback so encoder and echo interrupts coexist.

Subsystem messages are copied into `g_system_event_queue`; commands and telemetry use dedicated queues. Queue sends from periodic sensor paths are non-blocking. A full event queue drops that sample rather than stalling a time-sensitive task.

## Integration assumptions to verify

The default GPIO map in `include/config.h` is a proposed map, not a verified Robo Pico pinout. Check it against the carrier schematic and the actual motor driver, encoder, sensor, and servo modules before building with `ROBOT_SIMULATION=OFF`. The IMU driver assumes an MPU-6050-compatible device at I2C address `0x68`, accelerometer range ±2 g, and gyroscope range ±250 degrees/s. The barcode decoder currently assumes an 8-bit pulse-width frame with a long start gap; replace that framing with the track's published barcode format before navigation tests.

RP2040 has no built-in Wi-Fi. Buddy 1 currently supplies heartbeat/JSON telemetry over USB and a thread-safe command-queue API; it does not claim a live MQTT connection. Add an adapter for the team's actual radio (for example, an ESP AT modem or a Pico W/CYW43 board) and MQTT library, then route received, validated commands through `Buddy1Telemetry_SubmitCommand`. Topic names, payload schema, reconnect policy, and credentials belong in that adapter, not in the controller.

The obstacle bypass distances/timings and PID gains are initial tuning values. Validate them at low speed in a clear test area, establish motor polarity and encoder scaling, and retain a physical power cut-off during motor integration. The hump-height equation is an approximate ballistic estimate from integrated vertical acceleration; it is not a calibrated geometric measurement and needs empirical validation on the challenge track.# INF2004_Project