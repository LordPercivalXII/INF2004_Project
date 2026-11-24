# Buddy 4 — Terrain Analysis Report

**Subsystem:** LSM303DLHC IMU and odometry-assisted terrain estimation  
**Source:** `app_program/subsystem4/`  
**Configuration reviewed:** `app_program/common/robot_config.h`

## 1. Objective

Buddy 4 detects shocks, classifies motion and estimates the height of a hump
or ramp while the robot crosses it. The terrain estimator combines IMU pitch
with signed wheel travel. It is a bounded crossing measurement, not an
absolute-altitude or general-purpose terrain map.

## 2. Sensor and processing pipeline

The LSM303DLHC is sampled at 100 Hz over I2C0:

| Item | Configuration |
|---|---|
| Sensor | LSM303DLHC |
| I2C pins | GP4 SDA, GP5 SCL |
| Calibration | 200 stable samples |
| Filter | 5-sample moving average |
| Shock threshold | 3.0 g |
| Hump trigger | 0.08 g world-vertical acceleration |
| Wheel input | Signed motion distance from Buddy 2 |

The processing sequence is:

1. Calibrate acceleration while the car is stationary and level.
2. Calculate roll and pitch from the filtered acceleration vector.
3. Validate sensor status, timing, saturation, shock and tilt limits.
4. Fuse pitch slope with signed wheel distance:

```text
vertical increment ≈ wheel travel × slope
```

5. Track rise, peak rise, descent and level run-out.
6. Accept a terrain crossing only when the robot returns to level and
   satisfies the required valid run-out.

The controller receives terrain events before other queued events so that a
terrain fault or accepted height result is handled promptly.

## 3. Terrain classification and validity rules

The estimator intentionally rejects ambiguous or unsafe terrain data:

- The car must be calibrated before a crossing can be accepted.
- Invalid tilt may be bridged only for a bounded interval and distance.
- Recovery requires valid tilt within 250 ms, no more than 20 mm travel and no
  more than 6° endpoint pitch change.
- During a tilt gap, acceleration magnitude must remain between 0.65 g and
  1.35 g.
- A crossing is limited to 10 m total path and 1.0 m peak rise.
- A negative initial excursion greater than 50 mm is rejected as a downhill
  start.
- Wheel-speed mismatch, excessive turn rate, stale IMU data, impacts,
  saturation and invalid pitch/roll reject the window.
- A completed crossing requires a consistent return height and at least 3 cm
  of valid level run-out.

A rejected window preserves a diagnostic interrupted peak but does not replace
the last accepted terrain result. This prevents a noisy or interrupted
crossing from being reported as valid.

## 4. Terrain scenarios

| Scenario | Expected handling |
|---|---|
| Flat level floor | No hump accepted; level reference remains stable |
| Gentle hump with level exit | Rise and peak are tracked; result accepted after run-out |
| Short invalid-tilt interval | Crossing is temporarily invalid and may recover within limits |
| Impact or shock | Window is rejected; shock diagnostic is reported |
| Sustained downhill start | Window is rejected as `downhill_start` |
| Excessive wheel slip or turning | Window is rejected as distance, speed or turn mismatch |
| No level return | Peak remains an interrupted diagnostic, not a valid height |

## 5. Verification status

The automated Buddy 4 tests currently cover:

| Test | Status | Scope |
|---|---|---|
| `S4-U01` | PASS | Five-tap moving average |
| `S4-U02` | PASS | 45° roll calculation |
| `S4-U03` | PASS | 30° pitch calculation |
| `S4-U04` | PASS | 12-bit acceleration decode and scale |
| Full S4 host suite | PASS | 169 checks; 0 failures |
| `S4-M01` | NOT RUN | Physical LSM303DLHC boot detection |

The host suite now passes all 169 checks. The repository still does not contain
a saved physical terrain-crossing log, so real-world height accuracy remains
**pending hardware validation**.

## 6. Recommended terrain test procedure

1. Place the robot on flat ground and keep it still and level for the full
   200-sample calibration period.
2. Roll it straight for a measured 20 cm on flat ground.
3. Cross a known-height hump slowly and without lifting, rocking or slipping
   the wheels.
4. Continue at least 3 cm beyond the hump onto level ground.
5. Save the serial output, including calibration, terrain status and any
   rejection reason.
6. Compare the reported peak height with the physical reference height:

```text
height error (%) =
  (reported peak height - reference height) / reference height × 100
```

For the configured 66 mm wheels and 630 ticks/revolution, a 20 cm straight
run should produce approximately 608 ticks per wheel. A large left/right
difference should be investigated as wheel contact, encoder wiring or
calibration error before changing terrain thresholds.

## 7. Limitations

The LSM303DLHC has no gyroscope. Acceleration-based tilt cannot be perfectly
separated from vehicle acceleration, wheel slip or chassis rocking. Magnetic
heading is uncalibrated and must not be used as the turn-control signal.
Consequently, this implementation is suitable for slow, mild humps that
return to the same level, but it cannot guarantee accurate height during
arbitrary acceleration or uneven terrain.

## 8. Conclusion

Buddy 4 provides a conservative, fault-aware terrain-crossing estimator.
Filtering, calibration, shock detection and bounded recovery are implemented.
The host suite passes all 169 checks. The remaining work is physical
validation: run `S4-M01`, cross a measured hump, save the console log and
calculate reported-height error against the physical reference.

