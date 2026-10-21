# Buddy 5 Testing Checklist

## Build checks

- [x] Build the `robot_car` target with `ROBOT_SIMULATION=ON`.
- [x] Confirm the Buddy 5 source compiles without Buddy 5 warnings.
- [x] Confirm `build/robot_car.uf2` is generated.
- [x] Compile the Buddy 5 source with `ROBOT_SIMULATION=OFF` to validate its GPIO ISR and hardware path.
- [x] Build the complete firmware with `ROBOT_SIMULATION=OFF`.

The previously undefined Buddy 3 settings are now defined in `include/config.h`:
`IR_RING_SIZE=64`, `ADC_CHANNEL_IR_LEFT=0` (GP26/ADC0),
`ADC_CHANNEL_IR_CENTER=1` (GP27/ADC1), and `IR_SAMPLE_RATE_HZ=1000`.
The full hardware-mode firmware build now succeeds. Existing warnings remain
in other Buddy code and Pico SDK sources; no Buddy 5 warnings were emitted.

## Simulation checks

The deterministic simulation places an obstacle at the center on every twelfth
scan and returns clear left/right distances. Verify through debugger/event
inspection that:

- [ ] A complete clear scan posts `SYSTEM_EVENT_OBSTACLE` with
      `scan_complete=true` and `detected=false`.
- [ ] The simulated obstacle triggers the fine scan and reports a positive
      width, center distance at 24 cm, and a safe bypass side.
- [ ] `bypass_direction` is `-1` or `+1` for a clear side and `0` when both
      sides are blocked.
- [ ] A detected obstacle with no safe side causes the Vehicle Controller to
      stop and enter `MISSION_FAULT`.
- [ ] Event queue delivery failures are reported rather than silently ignored.

## Hardware checks

Before applying power:

- [ ] TRIG is wired to GP17; ECHO is wired to GP16 through a divider/level
      shifter; servo signal is GP15.
- [ ] HC-SR04 and Pico share ground. HC-SR04 ECHO does not exceed 3.3 V at GP16.
- [ ] Servo power is within the servo rating and is not overloaded from the
      Pico's 3.3 V rail.

With the robot safely immobilized:

- [ ] Confirm servo positions correspond to left (45°), center (90°), and
      right (135°).
- [ ] Confirm TRIG pulse is at least 10 µs and pulses are at least 66 ms apart.
- [ ] Check ECHO pulse width and calculated distance at several measured
      distances from 2 cm to 400 cm; record the error against the ±2 cm target.
- [ ] Confirm a target at or inside 30 cm sets `detected=true`.
- [ ] Confirm the fine scan runs only after a coarse hit and covers the hit
      sector in 5° increments.
- [ ] Confirm left/right clearance selects the larger safe side, and both
      blocked sides yield `bypass_direction=0`.
- [ ] Confirm no-echo timeout is handled as maximum range and does not stall
      other FreeRTOS tasks.
- [ ] Measure the time from the first coarse trigger to the completed
      obstacle/bypass decision; it must be under 3 seconds.
- [ ] Verify the Vehicle Controller receives the event and that Motion Control
      alone drives the motors during the bypass.

Record measured timing, accuracy, hardware revision, and any deviations here
before submitting the physical-test results.
