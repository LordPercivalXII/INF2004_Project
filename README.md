# INF2004 autonomous robot car — micro T-Kernel 3.0 on Raspberry Pi Pico W

> This is the micro T-Kernel port of the FreeRTOS robot car. Read
> **`PORTING.md`** (what changed and why) and **`TEAM.md`** (pins,
> priorities, interfaces). Build options added for the car:
> `make SIM=0` drives the real hardware; the default `SIM=1` uses
> simulated sensors, like `ROBOT_SIMULATION=ON` in the original.

The template guide below still applies.

## Ramp-height measurement

The LSM303DLHC supplies pitch; signed wheel travel supplies distance. Start
stationary on level ground and wait for `imu_calibrated:1` and
`height_tracking_valid:1` before moving slowly and straight over the hump.

Brief invalid-tilt intervals preserve the crossing while height is frozen and
marked invalid. Recovery requires valid tilt within **250 ms**, no more than
**20 mm** absolute wheel travel, and at most **6 degrees** endpoint pitch change.
Acceleration magnitude during the gap must remain within **0.65–1.35 g**.
Interpolated travel is limited to **60 mm per crossing/baseline**. Invalid pitch
is never integrated; interpolated travel cannot satisfy the level runout.
These are software approximation limits, not guaranteed sensor accuracy.

A tiny initial downhill increment no longer immediately loses the baseline:
confirmation permits at most **0.5 mm net fall**, with **20 mm / 250 ms** per
unconfirmed out-of-level excursion. Fresh valid level samples renew that
excursion window, so a small dip followed by flat travel or a level pause does
not discard the ground reference. Signed height debt and total path are never
zeroed by this renewal: a later ascent must regain the initial fall before
becoming active. Interpolated level samples cannot renew the window, and the
overall 10 m path limit still applies.
Small encoder increments on a slope accumulate without a per-sample vertical
distance deadband, so slow pushing does not silently lose ascent or descent.
Sustained downhill starts, impacts, clipping, sensor failures, excessive roll,
stale data and implausible wheel movement still reject the measurement.
Acceleration-limit rejections print the failing acceleration components.

Only a completed ascent/descent with consistent return height and 3 cm of valid
level runout updates `hump_valid` and `hump_count`. Interrupted peaks remain
separate diagnostic records with `interrupted_height_valid:0`; they are not
accepted ramp heights. This accelerometer-only tilt sensor cannot guarantee
accurate height during arbitrary acceleration, wheel slip or chassis rocking.

### Diagnosing an incomplete ramp crossing

`IMU_HEIGHT_TRACE_ENABLED=1` enables separate `height_trace` JSON records at
each motion sample (up to 50 Hz), during wheel movement/tracking and for
500 ms afterward. Normal one-second snapshots and heartbeats are unchanged.
The low-priority telemetry task prints the records; acquisition does not wait
for USB output. Trace queue loss is counted in `dropped` and reported explicitly;
motion-event queue loss is reported separately by subsystem 2.

Each record contains motion and IMU timestamps, motion interval, corrected
acceleration in g, pitch/roll, both signed encoder totals and speeds, cumulative
distance, interval travel in mm, live rise/peak in mm, and quality/gap flags.
`null` means a nonfinite or out-of-range diagnostic value, not zero.
This trace is for commissioning, not a more accurate height estimator; set
`IMU_HEIGHT_TRACE_ENABLED=0` when the extra serial output is no longer needed.

For the next check, first roll the car **straight on flat ground for a measured
20 cm**, then cross the ramp slowly and continue at least 3 cm beyond it. Keep
both wheels in contact and do not lift or rock the body by hand. Both encoder
totals should increase by similar amounts on the straight flat segment. With
the configured 66 mm wheels and 630 ticks/revolution, each wheel should gain
approximately 608 ticks over 20 cm; verify the actual hardware geometry before
changing calibration. A very different left/right gain needs investigation of
wheel contact, encoder A/B wiring and direction/counting; do not compensate by
arbitrarily doubling one wheel or relaxing the return-height check.

All application millisecond timestamps now use the same RP2040 hardware timer
as wheel-speed microsecond intervals, rather than serviced kernel ticks.
The full 64-bit timer is read coherently before conversion, so the 32-bit
microsecond rollover does not reset millisecond time. Application millisecond
time still wraps naturally after about 49.7 days. Kernel wake-up scheduling is
unchanged. A genuinely delayed motion interval over 100 ms remains invalid;
it must not be squeezed into a short interval and integrated with current tilt.
Height fusion also checks signed travel against average wheel speed and elapsed
time, allowing 1 mm plus 2 ms worth of speed for rounding. A failure reports
`distance_speed_mismatch`, retaining the interrupted diagnostic peak.

Successful IMU reads with an abnormal sample interval are now distinguished
from I2C/range failures. For intervals **31–100 ms**, calibration and gravity
are preserved, tilt is marked temporarily invalid, and the existing bounded
gap path can recover after a normal valid endpoint. No acceleration or held
pitch is integrated across missing samples. All existing gap distance, pitch,
acceleration and total-travel limits still apply. Intervals below 5 ms or above
100 ms reject height as `imu_sample_interval`, not `sensor`; real read errors,
clipping and impacts still reject immediately. The acceleration-only reference
integrator still aborts on timing gaps rather than double-integrating them.
Trace records include `imu_dt_ms`, `timing_gap` and `acquisition_us`.
Motion arriving with a 61–100 ms old IMU sample also buffers travel until a
fresh endpoint arrives; that stale pitch cannot finalize a bridge. Older IMU
data still rejects as `imu_age`.
The acquisition duration includes preemption while reading, not only I2C bus
time. Comparing it with the sample interval helps locate scheduling/read stalls;
the cause of all long hardware stalls has not yet been established.

USB service now runs at priority 19 and Wi-Fi polling at 21, below the robot's
sensor/control tasks (10–18); telemetry remains at 20. Both services stay pinned
to core 0 for interrupt ownership, but no longer outrank motion or IMU sampling.
USB wake requests are coalesced before each event-processing pass, and unused
input is drained only up to the byte count present at the start of that pass.
Console strings are appended in spans rather than taking nested locks for every
character; complete-call SMP serialization and CRLF conversion are preserved.
These changes remove service-priority starvation paths without increasing any
height-recovery limit. Elimination of the observed 182 ms stall still requires
a hardware crossing test; the serial log alone cannot prove its exact source.

On SMP USB builds, a separate `timing_diag` JSON line reports cumulative maximum
kernel-lock hold/wait times, per-core console-lock hold/wait times (microseconds),
and USB-ring dropped bytes once per heartbeat. These are boot-to-date maxima,
including startup, not the duration of a particular IMU delay; compare before
and after a failed crossing. The console measurement includes only outermost
lock ownership, so nested output does not overwrite its start time. This
diagnostic does not change height acceptance or validate the physical result.

---


A starting point for your project: micro T-Kernel 3.0 running on **both cores**
of the Pico W's RP2040, with a USB serial console and the WiFi driver built in.

You need a **Raspberry Pi Pico W**, a USB cable, and a computer running
macOS, Linux, or Windows with **WSL (Ubuntu)**. On Windows, run every
command below inside the WSL Ubuntu terminal.

---

## 1. Set up the Pico C SDK and the compiler

You need two separate things:

| What | Why | Where it comes from |
|---|---|---|
| **Pico C SDK** | USB and WiFi driver source code | `git clone` (step 1.2) |
| **Arm GNU Toolchain** (`arm-none-eabi-gcc`) | the compiler | a separate download (step 1.3) |

> ⚠️ **The SDK does NOT include the compiler.** You must install the compiler
> on its own and put its `bin` folder on your `PATH` (step 1.4).

**1.1 Install the basic tools: git, make and a host C++ compiler**

```sh
# macOS
xcode-select --install

# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++
```

**1.2 Download the Pico C SDK** (any folder works; this guide uses `~/pico`)

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver
```

The `submodule` line is required: it downloads the USB and WiFi drivers.
(SDK 2.3.0 has also been tested and works.)

**1.3 Install the Arm GNU Toolchain (the compiler)**

- **macOS / Windows (WSL) / Linux:** download from
  <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>.
  Pick the **AArch32 bare-metal target (arm-none-eabi)** package for your
  computer. For WSL, choose the **Linux x86_64** `.tar.xz`.
  Extract it, e.g. to `~/tools/arm-gnu-toolchain-<version>/`.
- **Linux / WSL shortcut:** `sudo apt install -y gcc-arm-none-eabi`
  installs it straight onto your PATH. If you use this, skip the PATH line in 1.4.

Find the folder that contains `arm-none-eabi-gcc`. It is the toolchain's
**`bin`** folder, e.g. `~/tools/arm-gnu-toolchain-15.3.rel1/bin`.

**1.4 Tell your shell where the compiler and SDK are**

Add these two lines to the end of `~/.zshrc` (macOS) or `~/.bashrc` (Linux/WSL),
**changing both paths to match your computer**:

```sh
export PATH="$HOME/tools/arm-gnu-toolchain-15.3.rel1/bin:$PATH"   # folder that contains arm-none-eabi-gcc
export PICO_SDK_PATH="$HOME/pico/pico-sdk"                         # the SDK from step 1.2
```

Open a **new** terminal, then check both:

```sh
arm-none-eabi-gcc --version     # must print a version, not "command not found"
ls $PICO_SDK_PATH               # must list the SDK files
```

If `arm-none-eabi-gcc` is "not found", your `PATH` does not point to the
toolchain's `bin` folder. Fix that before going further.

---

## 2. Clone this repo and build the `.uf2` file

**2.1 Clone your project**

```sh
cd ~/pico
git clone <URL-of-this-repo> my-project
cd my-project
```

**2.2 Build**

```sh
cd build_make
make -j8
```

A successful build ends with a line like
`tools/elf2uf2 mtk3pico_smp1_usb_cdc_wifi.elf mtk3pico_smp1_usb_cdc_wifi.uf2`.
Your firmware is **`build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`**.

If `make` stops with an error, it tells you which one is wrong:
`arm-none-eabi-gcc not found` means fix `PATH`, and
`PICO_SDK_PATH is not set` means fix `PICO_SDK_PATH` (step 1.4).

To build a single-core, UART-only or test version, see section 3.

**2.3 Flash it onto the Pico W**

1. Hold the **BOOTSEL** button and plug the Pico W into USB, then release.
   A drive called **RPI-RP2** appears.
2. Copy the `.uf2` file onto that drive. On WSL, run `explorer.exe .` in
   `build_make` to open the folder in Windows. The Pico reboots by itself.

**2.4 See the output**

Open the Pico's USB serial port at 115200 baud:

```sh
screen /dev/tty.usbmodem* 115200      # macOS   (quit: Ctrl-A then K)
screen /dev/ttyACM0 115200            # Linux
```

On Windows, use PuTTY or the VS Code *Serial Monitor* on the new COM port.
You should see one telemetry JSON line and one heartbeat per second.
(The template's GP19 example LED is gone: GP19 is now the right encoder B.)

**2.5 Write your own code**

Your code lives in **`app_program/`**. Every `.c` file there, and in its
subfolders, is compiled automatically.

- **`app_main.c` → `usermain()` is where the program starts.** It first calls
  every subsystem's `subsystemN_init()` (create kernel objects), then every
  `subsystemN_start()` (start tasks). Read its comments first.
- **One folder per student:** `app_program/subsystem1/` … `subsystem5/`.
  Subsystems 1–4 hold a working example, one task each (LED, producer,
  consumer, monitor). Together they use the main micro T-Kernel APIs and
  pass data between subsystems. Subsystem 5 is an empty template.
  Replace the examples with your own subsystems.
- `subsystemN.h` is that subsystem's public interface: the object IDs and
  data formats the other subsystems may use.
- `app.h` holds the definitions shared by the whole team.
- **Drivers go in your subsystem folder too**, as a `drv_<name>.c` /
  `drv_<name>.h` pair, e.g. `subsystem2/drv_ultrasonic.c`. They are compiled
  automatically. Do not add drivers under `device/`: that folder holds the
  kernel's own serial, I2C and ADC drivers, and adding to it means editing
  kernel and makefile files shared by the whole team.

**Working as a team:** record owners, pins, priorities and the interfaces
between subsystems in **`TEAM.md`**, and record every test in **`testing/`**
(see `testing/README.md`). `make TEST=1` builds a test image that runs all
the subsystem tests at power-up. Short videos of tests and demos are shared
as unlisted YouTube links, listed in **`videos/`**.

Pins: use **GP0–GP22 and GP26–GP28**. GP0/GP1 are free unless your team
builds with `CRASH_REPORT=1` (section 3), which needs them. Never use
GP23/GP24/GP25/GP29: they are wired to the Pico W radio. The Pico W's
on-board LED is also on the radio, so use an external LED instead.

After editing, run `make -j8` again in `build_make` and copy the new `.uf2`.

---

## 3. Build options: `SMP`, `CONSOLE`, `WIFI`, `TEST`, `CRASH_REPORT`

`make` on its own uses the defaults. To build differently, put options
after `make`. They can be combined in any order:

```sh
make -j8                          # defaults: SMP=1 CONSOLE=usb_cdc WIFI=cyw43 TEST=0 CRASH_REPORT=0
make SMP=0 -j8                    # single-core build
make CONSOLE=uart WIFI=none -j8   # no USB console, radio off (no Pico SDK needed)
make TEST=1 -j8                   # firmware with the subsystem self-tests
make CRASH_REPORT=1 -j8           # also see crash reports, on GP0/GP1 (uses those pins)
```

To set several options at once, list them all, separated by spaces. For
example, to track down a crash in the test build while running on one core only:

```sh
make SMP=0 TEST=1 CRASH_REPORT=1 -j8
#  -> build_make/mtk3pico_smp0_usb_cdc_wifi_crash_test.uf2
```

Any option you leave out keeps its default (here `CONSOLE=usb_cdc` and
`WIFI=cyw43`). Options are not remembered: give them on every `make`, as
the next plain `make` goes back to the defaults.

| Option | Values (**default**) | What it does | Image name | C macro your code can test |
|---|---|---|---|---|
| `SMP` | `0`, **`1`** | `1` runs the kernel on both RP2040 cores; `0` uses core 0 only | `smp1` / `smp0` | `TK_SUPPORT_SMP` (1 or 0) |
| `CONSOLE` | **`usb_cdc`**, `uart` | where `tm_printf` output goes. `usb_cdc` = USB cable; `uart` = UART0 only (GP0 TX / GP1 RX, needs a USB-serial adapter) | `_usb_cdc` / `_uart` | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | `cyw43` powers up the Pico W radio and its driver task; `none` leaves the radio off | `_wifi` / *(nothing)* | `TM_WIFI_CYW43` |
| `TEST` | **`0`**, `1` | `1` compiles every `subsystemN_test.c` and runs the tests at power-up | `_test` / *(nothing)* | `APP_TEST` (1 or 0) |
| `CRASH_REPORT` | **`0`**, `1` | `1` also copies the console to UART0 on **GP0 (TX) / GP1 (RX)**, 115200 baud, so you can see the last messages and the kernel's crash report (see below). **Uses GP0/GP1.** `CONSOLE=uart` always does this. | `_crash` / *(nothing)* | `TM_CONSOLE_UART` (1 or 0) |

**Why `CRASH_REPORT` exists.** USB output is not sent the moment you call
`tm_printf`: it waits in a buffer until the kernel's USB task sends it. If the
program crashes, no task runs again, so the last messages and the kernel's
crash report never reach your USB terminal. The terminal just goes quiet.
UART output *is* sent at once, even inside the crash handler. So when you
need to know why the Pico crashed or froze, build with `CRASH_REPORT=1`,
connect a USB-serial adapter (adapter RX → GP0, adapter GND → GND), and read
it at 115200 baud. Normal `tm_printf` output on USB works the same with
either setting. If a subsystem has claimed GP0/GP1 in `TEAM.md`, unplug that
hardware before using `CRASH_REPORT=1`.

The options are part of the image name, e.g.
`mtk3pico_smp1_usb_cdc_wifi_test.uf2`, so you can always tell which build
you are flashing. Switching options needs no `make clean`: the makefile
notices and recompiles everything.

Using an option in C code:

```c
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC;   /* core affinity only exists on the dual-core build */
#endif

#if APP_TEST
    /* code that only exists in the test build */
#endif
```

### Where the options live in the makefile

All five are handled in **`build_make/pico_rp2040.mk`**. To find each piece, search the file for the
names in the left column (e.g. `grep -n "TEST" build_make/pico_rp2040.mk`).
Each option goes through the same steps:

| Step | What to look for | Example for `TEST` |
|---|---|---|
| 1. Default value | `NAME ?= value` near the top of the file | `TEST ?= 0` |
| 2. Allowed values | an `$(error Unknown ...)` check just below it | `$(error Unknown TEST '$(TEST)'; use 0 or 1)` |
| 3. Image-name suffix | `NAME_SUFFIX :=` and the `EXE_FILE :=` line | `TEST_SUFFIX := _test` |
| 4. C macro for the code | a `CFLAGS += -D...` line | `CFLAGS += -DAPP_TEST=$(TEST)` |
| 5. Rebuild on change | the `PROFILE_ID :=` line | `...-test$(TEST)` |
| 6. Extra source files (`CONSOLE`, `WIFI` only) | an `ifeq ($(CONSOLE),usb_cdc)` or `ifeq ($(WIFI),cyw43)` block that adds the USB/radio files and SDK include paths | — |

- **Changing a default:** edit the `?=` line from step 1, e.g.
  `CONSOLE ?= uart`. Every `make` without that option then uses the new value.
- **Adding your own option** (say `DEBUG=1` to turn on extra prints): copy
  how `TEST` is done. Add steps 1 to 5 next to the `TEST` lines, then
  use `#if APP_DEBUG` in your code. **Step 5 matters:** without it, switching
  the option can quietly link files that were compiled with the old setting.
- **Kernel settings** such as the maximum number of tasks, semaphores or
  mailboxes are not make options. They are in `config/config.h` (`CNF_MAX_...`).

The makefile also has `NET=lwip` and `MQTT=1` (TCP/IP and MQTT over WiFi).
They need the SDK's `lib/lwip` submodule and a `config/wifi_secrets.h`
(copy `config/wifi_secrets.example.h`). This guide does not cover them.

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
