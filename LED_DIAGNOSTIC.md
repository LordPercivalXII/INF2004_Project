# LED Diagnostic for Power Bank WiFi/MQTT Debugging

## Problem
When the Pico is powered by a power bank (without USB), the serial console is unavailable because there's no USB host to enumerate the CDC serial device. This makes it impossible to see boot messages or diagnose why MQTT telemetry doesn't work.

The most likely cause of MQTT failure on power bank is **I2C/IMU initialization failure** due to power supply ripple or sag. The controller won't start missions if `b_imu_healthy` is false, even when receiving START commands over MQTT.

## Solution: Diagnostic LED

Added GPIO 25 (onboard Pico LED) diagnostic indicator that shows IMU/I2C health status without requiring a serial console.

### LED Patterns

**BOOT (booting, I2C initializing)**
- Steady 1 Hz blink: 500ms ON, 500ms OFF
- Shows while the robot is starting up and IMU is calibrating

**I2C/IMU FAILED**  
- Pattern: 3 fast blinks (100ms each) + 600ms pause, repeating
- **This means:** I2C bus failed to open, or IMU chip not responding
- **Action:** Check power supply voltage stability; add capacitor to 3.3V rail

**IMU HEALTHY**
- LED on steady (100% duty)
- **This means:** IMU is initialized and calibrated; robot is ready
- MQTT commands will now be processed and mission will start

## How It Works

1. Controller task polls IMU status in its main loop
2. Checks `imu.b_sensor_valid` and `b_imu_healthy` flags
3. Sets LED state accordingly: `BOOT` → `I2C_FAIL` or `IMU_HEALTHY`
4. `diag_led_poll()` updates the LED every iteration

### Code Integration

- **Files added:**
  - `app_program/common/diag_led.h` — declarations
  - `app_program/common/diag_led.c` — implementation
  
- **Files modified:**
  - `app_program/controller/controller.c` — initialize and poll LED in main loop

### What to Do on Power Bank

1. **Flash the updated firmware** to the Pico
2. **Power on from power bank** (no USB connection)
3. **Watch the onboard LED** (GPIO 25):
   - Blinking slowly (~1 Hz)? Still booting, wait 5-10 seconds
   - Fast blink pattern (3x ON-OFF)? **I2C/IMU failed** — see troubleshooting below
   - Steady ON? IMU healthy, ready for MQTT commands

### Troubleshooting I2C Failure

If you see the **fast 3-blink error pattern**:

**Most likely:** Power bank voltage sag or ripple
- Try a **different power bank** or USB cable
- Use a **5V wall charger** (rated 2A+) instead
- Add a **100µF capacitor** across 3.3V and GND on the IMU board (close to the sensor)
- Check that all wires are **soldered securely** (loose connection looks like power failure)

**Next:** Check I2C bus with a multimeter or oscilloscope if available
- SDA (GPIO 4) and SCL (GPIO 5) should idle HIGH (~3.3V)
- If they're LOW or noisy, the bus is stuck or being pulled

**Last resort:** Use USB power (with a USB battery bank that provides stable 5V) or connect to serial over GPIO 0/1 (UART0 is mirrored to the console)

## Files

- [diag_led.h](./app_program/common/diag_led.h) — Header
- [diag_led.c](./app_program/common/diag_led.c) — Implementation  
- [controller.c](./app_program/controller/controller.c) — Integration (includes + poll call)

## Build

The firmware is built automatically:
```bash
cd build_make
make
```

The new `diag_led.c` is picked up by the wildcard in `build_make/mtkernel_3/app_program/subdir.mk`.
