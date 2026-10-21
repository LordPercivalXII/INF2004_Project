# Buddy 5 Requirements Alignment

| Requirement | Design-review intent | Implementation mapping | Status |
| --- | --- | --- | --- |
| FR12 | Sweep left/center/right; capture ultrasonic echo by ISR; calculate obstacle width and clearance | GP15 servo scan at 45°/90°/135°; GP16 shared-router edge ISR queues pulse widths; profile fields populated in `ObstacleEvent` | Implemented; physical sensor measurements pending |
| FR13 | Evaluate left/right clearance and command an offline bypass trajectory | Buddy 5 selects `bypass_direction`; event goes to Vehicle Controller, which issues motion commands | Integrated; physical route validation pending |
| NFR8 | Obstacle detection and bypass decision in under 3 seconds | Coarse three-point scan followed by a bounded 5° fine scan on detection; 66 ms minimum ping interval | Build verified; timing measurement pending |
| NFR9 | 3-point scan at −45°, 0°, +45° | Coarse points are 45°, 90°, 135° servo angles | Implemented |
| Hardware map | TRIG GP17, ECHO GP16, servo GP15 | `include/config.h` pin constants | Implemented; wiring/voltage check pending |
| HC-SR04 interface | 10 µs trigger, 2–400 cm range, avoid echo cross-talk | 10 µs TRIG, 30 ms echo timeout, range bounding, 66 ms ping spacing | Implemented; physical accuracy pending |
| ISR architecture | Central GPIO dispatcher; ISR capture only; task processes pulse widths | `Robot_GpioIrqRegister()` and FreeRTOS echo-width queue | Implemented |
| Safety | Do not choose a blocked bypass side | Direction `0` when both clearances are blocked; controller stops and faults | Implemented |

## Known limits and verification still required

- HC-SR04 timeout cannot distinguish an unobstructed path from a failed,
  disconnected, or poorly reflecting sensor. Validate the sensor and wiring
  before relying on timeout-as-clear behavior.
- Width is an estimate based on coarse/fine hit samples and the nominal beam
  width; validate it against the course obstacles before using it for precise
  navigation.
- The ±2 cm distance objective, servo angle calibration, sub-3-second
  end-to-end timing, and successful physical line reacquisition require a
  hardware run. A firmware build alone does not prove these criteria.
