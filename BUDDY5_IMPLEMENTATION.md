# Buddy 5: Ultrasonic Obstacle Scanning

## Design basis

This implementation follows the Group 19 Design Review and the supplied HC-SR04
datasheet. The older Buddy 5 prompt in this repository described different pins
and requirements; use this document and the Design Review as the current source
of truth.

| Signal | Cytron Robo Pico pin | Connection |
| --- | --- | --- |
| HC-SR04 TRIG | GP17 | 3.3 V GPIO output |
| HC-SR04 ECHO | GP16 | GPIO input through a level shifter or voltage divider to 3.3 V |
| Servo signal | GP15 | PWM output |
| Sensor VCC / GND | 5 V / GND | Common ground with the Pico and servo supply |

The servo supply must meet its voltage/current rating. Do not connect the
HC-SR04's 5 V ECHO output directly to RP2040 GPIO.

## Behavior

`Buddy5Obstacle_CreateTask()` starts the obstacle-scanning task. The task:

1. Sweeps coarse scan points at servo angles 45°, 90°, and 135° (−45°, 0°,
   and +45° relative to center).
2. Waits 50 ms for servo movement, sends a 10 µs TRIG pulse, and captures the
   ECHO rising/falling edges through the shared GPIO IRQ router. The ISR queues
   the measured pulse width; the task waits on that queue with a 30 ms timeout.
3. If any coarse point is at or inside the 30 cm threshold, performs a 5° fine
   scan across the detected region with a 20° margin.
4. Calculates center distance, approximate obstacle width, left/right
   clearance, and a bypass direction; publishes a complete
   `SYSTEM_EVENT_OBSTACLE` through `Robot_PostEvent()`.

The HC-SR04 datasheet recommends more than 60 ms between ranging cycles to
avoid echoes from a previous ping. The implementation uses a 66 ms minimum
trigger interval. This keeps the maximum coarse-plus-fine scan below the
Design Review's 3-second detection/decision limit while reducing cross-talk.
The conservative scan budget is 22 readings (three coarse plus at most 19 fine)
at up to 50 ms servo settling plus 66 ms between pings, or 2.552 seconds.

## Measurement and profile calculations

The ECHO high pulse is the round-trip acoustic travel time. With sound speed
approximated as 343 m/s:

```text
distance_cm = echo_width_us * 0.0343 / 2
```

No echo within the 30 ms timeout is represented as the 400 cm maximum range;
readings are bounded to the HC-SR04's 2–400 cm range. Left and right clearance
are the nearest measured distances in each side's scan sector, with values at
or inside the obstacle threshold represented as zero.

Obstacle width is an estimate: the lateral span of the hit samples is
augmented by the HC-SR04's nominal 15° beam footprint. It is suitable for
relative bypass planning, not precision dimensioning. A direction of `-1`
selects the side with greater clearance on the left, `+1` selects the right,
and `0` means neither side has threshold clearance. The Vehicle Controller
executes the selected maneuver; Buddy 5 does not drive the motors.

## Integration and failure behavior

The shared `Robot_GpioIrqRegister()` dispatcher routes GP16 edges. Its ISR only
timestamps edges and queues pulse widths; it does not poll ECHO or perform
distance calculations. The scanning task runs at sensor priority 3, and the
Vehicle Controller runs at priority 4 as specified by the design review.

If the GPIO route or echo queue cannot be initialized, Buddy 5 logs the failure
and posts `SYSTEM_EVENT_FAULT`. If both bypass sides are blocked, the controller
stops and enters its fault state rather than selecting an unsafe direction.
Failure to deliver an obstacle event due to a full system event queue is logged.

## Build and test

Build the project with the repository's configured Pico SDK and FreeRTOS paths:

```powershell
cmake --build build --target robot_car
```

See [BUDDY5_TESTING_CHECKLIST.md](./BUDDY5_TESTING_CHECKLIST.md) for simulation
and hardware tests, and [BUDDY5_ALIGNMENT_REVIEW.md](./BUDDY5_ALIGNMENT_REVIEW.md)
for requirement traceability. A successful firmware build does not replace
physical verification of the echo voltage divider, distance accuracy, servo
alignment, or end-to-end bypass behavior.