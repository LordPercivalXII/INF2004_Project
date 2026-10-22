# INF2004 Autonomous Robotic Car

Modular RP2040 application using the Raspberry Pi Pico SDK and FreeRTOS. The Vehicle Controller is the sole owner of mission state; subsystems exchange copied messages through FreeRTOS queues and the system event group.

## 🚀 Quick Build (Windows)

```powershell
# 1. Run setup script (installs dependencies & compiles)
.\setup_windows.ps1

# 2. Flash to Pico
.\flash_pico.ps1
```

---

## 📋 Windows Setup & Build

### What You Need to Install

1. **CMake** (≥3.13)
   - Download: https://cmake.org/download/
   - ✅ Check "Add CMake to system PATH" during installation

2. **Ninja Build System**
   - Download: https://github.com/ninja-build/ninja/releases
   - Extract to: `C:\tools\ninja\`
   - Add `C:\tools\ninja\` to system PATH (Environment Variables)

3. **ARM GCC Toolchain** (Choose one method)
   
   **Option A - Easiest (via winget):**
   ```powershell
   winget install GNU.Arm.EmbeddedToolchain.GCC
   ```
   
   **Option B - Manual Download:**
   - Download: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
   - Choose: `arm-gnu-toolchain-*-mingw-w64-i686-arm-none-eabi.exe`
   - ✅ Check "Add path to environment variable" during installation
   
   **Option C - Alternative (SourceForge):**
   - https://sourceforge.net/projects/mingw-arm-gnu-toolchain/

4. **Raspberry Pi Pico SDK**
   ```powershell
   cd C:\pico
   git clone https://github.com/raspberrypi/pico-sdk.git
   cd pico-sdk
   git submodule update --init
   ```

5. **FreeRTOS-Kernel**
   ```powershell
   cd C:\pico
   git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git
   ```

### Configure Paths

Edit `setup_windows.ps1` (lines 8-9):
```powershell
$PICO_SDK_PATH = "C:\pico\pico-sdk"
$FREERTOS_KERNEL_PATH = "C:\pico\FreeRTOS-Kernel"
```

Edit `build_commands.bat` (lines 6-7):
```batch
set PICO_SDK_PATH=C:\pico\pico-sdk
set FREERTOS_KERNEL_PATH=C:\pico\FreeRTOS-Kernel
```

### Build Commands

**Using PowerShell (Recommended):**
```powershell
# Build everything
.\setup_windows.ps1

# Flash to Pico
.\flash_pico.ps1
```

**Using Batch Commands:**
```batch
# Build
build_commands.bat build

# Flash
build_commands.bat flash

# Other options
build_commands.bat help               # Show all commands
build_commands.bat clean              # Remove build directory
build_commands.bat rebuild            # Clean + build
build_commands.bat configure-hw       # Setup for real hardware
build_commands.bat wifi-test          # Build WiFi test only
build_commands.bat mqtt-test          # Build MQTT test only
```

### Manual CMake (if scripts don't work)

```powershell
# Set environment variables
$env:PICO_SDK_PATH = "C:\pico\pico-sdk"
$env:FREERTOS_KERNEL_PATH = "C:\pico\FreeRTOS-Kernel"

# Configure
cmake -S . -B build -G Ninja -DROBOT_SIMULATION=ON

# Build
cmake --build build
```

### Flash to Pico

**Automatic (Recommended):**
```powershell
.\flash_pico.ps1
```

**Manual:**
1. Hold **BOOTSEL** button on Pico
2. Plug USB into computer
3. Release BOOTSEL
4. Pico appears as removable drive "RPI-RP2"
5. Copy `build/robot_car.uf2` to the drive
6. Pico auto-resets and firmware runs ✅

---

## 🍎 macOS Setup & Build

### What You Need to Install

```bash
# Using Homebrew
brew install cmake ninja gcc git

# Or download directly:
# CMake: https://cmake.org/download/
# ARM Toolchain: https://developer.arm.com/tools-and-software/open-source-software/gnu-toolchain/gnu-rm/downloads
```

### Download SDK & Kernel

```bash
mkdir -p ~/pico
cd ~/pico

# Pico SDK
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

# FreeRTOS-Kernel
cd ~/pico
git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git
```

### Build

```bash
cd ~/path/to/INF2004_Project

export PICO_SDK_PATH="$HOME/pico/pico-sdk"
export FREERTOS_KERNEL_PATH="$HOME/pico/FreeRTOS-Kernel"

# Configure
cmake -S . -B build -G Ninja -DROBOT_SIMULATION=ON

# Build
cmake --build build
```

### Flash to Pico

```bash
# 1. Put Pico in BOOTSEL mode (hold BOOTSEL + plug USB)
# 2. Pico appears as mounted drive
# 3. Copy firmware:
cp build/robot_car.uf2 /Volumes/RPI-RP2/
```

---

## 🐧 Linux Setup & Build

### What You Need to Install

**Ubuntu/Debian:**
```bash
sudo apt-get install cmake ninja-build build-essential git

# ARM Toolchain
sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi
```

**Fedora:**
```bash
sudo dnf install cmake ninja-build gcc git arm-none-eabi-toolchain
```

### Download SDK & Kernel

```bash
mkdir -p ~/pico
cd ~/pico

git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

cd ~/pico
git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git
```

### Build

```bash
cd ~/path/to/INF2004_Project

export PICO_SDK_PATH="$HOME/pico/pico-sdk"
export FREERTOS_KERNEL_PATH="$HOME/pico/FreeRTOS-Kernel"

cmake -S . -B build -G Ninja -DROBOT_SIMULATION=ON
cmake --build build
```

### Flash to Pico

```bash
# 1. Put Pico in BOOTSEL mode
# 2. Mount appears, typically at /media/[username]/RPI-RP2
# 3. Copy firmware:
cp build/robot_car.uf2 /media/[username]/RPI-RP2/
```

---

## Build Modes

**Simulation Mode** (default, `ROBOT_SIMULATION=ON`):
- No hardware needed
- Safe for testing algorithms
- Motors won't spin
- Deterministic timing

**Hardware Mode** (`ROBOT_SIMULATION=OFF`):
- Real sensors and motors active
- ⚠️ **Keep wheels elevated during testing!**
- ⚠️ **Verify GPIO map in `include/config.h` before building!**

---

## Build Outputs

- `build/robot_car.uf2` - Main firmware (flash this to Pico)
- `build/robot_car.hex` - Hex format
- `build/robot_car.elf` - Debug symbols
- `build/wifi_smoke_test.uf2` - WiFi test
- `build/mqtt_smoke_test.uf2` - MQTT test

---

## Telemetry

Connect serial monitor (115200 baud) to see JSON telemetry:
```json
{"type":"telemetry","speed":0.5,"heading":180.0}
{"type":"status","mode":"active","battery":4.2}
```

---

## Module boundaries

- `controller/` owns mission state and translates line, IMU, and obstacle events into motion commands.
- `buddy1_telemetry/` consumes telemetry and provides a queue-safe command submission API.
- `buddy2_motion/` owns motor PWM, encoder capture, and the speed-control loop.
- `buddy3_line_barcode/` samples the left/center line sensors and the right barcode sensor.
- `buddy4_imu/` reads MPU-6050-compatible I2C registers and reports shock, turn rate, and an approximate hump height.
- `buddy5_obstacle/` performs centered range checks, coarse/fine servo scans, width estimation, and clearance selection.
- `gpio_irq_router.c` multiplexes the RP2040's single GPIO IRQ callback so encoder and echo interrupts coexist.

Subsystem messages are copied into `g_system_event_queue`; commands and telemetry use dedicated queues. Queue sends from periodic sensor paths are non-blocking. A full event queue drops that sample rather than stalling a time-sensitive task.

Buddy 2 samples encoder A-channel rising edges every 20 ms, estimates signed wheel speed and center distance, and regulates each wheel independently. A 500 ms no-tick timeout on a commanded-moving wheel triggers a fault stop. `MOTION_TURN` commands execute encoder-odometry point turns; barcode payloads `L`, `R`, and `U` request left 90°, right 90°, and 180° turns at a junction. Measure and update `MOTOR_TRACK_WIDTH_M` and `MOTOR_ENCODER_TICKS_PER_REV` for the assembled robot before relying on turn accuracy. The PID gains and turn speeds are initial tuning values, not calibrated values.

## Integration assumptions to verify

The default GPIO map in `include/config.h` is a proposed map, not a verified Robo Pico pinout. Check it against the carrier schematic and the actual motor driver, encoder, sensor, and servo modules before building with `ROBOT_SIMULATION=OFF`. Level-shift the ultrasonic module's echo output to 3.3 V, use a suitable separate motor/servo supply, and connect grounds as required by the selected drivers. Keep the wheels raised for initial motor tests and use a physical power cut-off. The IMU driver assumes an MPU-6050-compatible device at I2C address `0x68`, accelerometer range ±2 g, and gyroscope range ±250 degrees/s. The barcode decoder currently assumes an 8-bit pulse-width frame with a long start gap; replace that framing with the track's published barcode format before navigation tests.

RP2040 has no built-in Wi-Fi. Buddy 1 currently supplies heartbeat/JSON telemetry over USB and a thread-safe command-queue API; it does not claim a live MQTT connection. Add an adapter for the team's actual radio (for example, an ESP AT modem or a Pico W/CYW43 board) and MQTT library, then route received, validated commands through `Buddy1Telemetry_SubmitCommand`. Topic names, payload schema, reconnect policy, and credentials belong in that adapter, not in the controller.

The obstacle bypass distances/timings and PID gains are initial tuning values. Validate them at low speed in a clear test area, establish motor polarity and encoder scaling, and retain a physical power cut-off during motor integration. The hump-height equation is an approximate ballistic estimate from integrated vertical acceleration; it is not a calibrated geometric measurement and needs empirical validation on the challenge track.# INF2004_Project