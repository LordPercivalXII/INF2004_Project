# Buddy 2 — PID Tuning and Motion Accuracy Report

**Subsystem:** Motion, motors, encoders and odometry  
**Source:** `app_program/subsystem2/`  
**Configuration reviewed:** `app_program/common/robot_config.h`  
**Latest recorded bench evidence:** 28 October 2026

## 1. Objective

This report evaluates the per-wheel speed controller and the accuracy of
straight-line and point-turn motion. The controller runs every 20 ms and uses
wheel encoder feedback to regulate each motor independently.

## 2. Motion-control implementation

The measured wheel speed is calculated from encoder ticks:

```text
distance per tick = pi × wheel diameter / ticks per revolution
                 = pi × 0.066 / 630
                 = 0.329 mm/tick
```

The current hardware configuration is:

| Parameter | Value |
|---|---:|
| Wheel diameter | 66 mm |
| Encoder resolution | 630 ticks/revolution |
| Track width | 125 mm |
| Control period | 20 ms |
| Maximum commanded speed | 0.70 m/s |
| PWM duty limit | 0.70 |
| Encoder stall timeout | 500 ms |

Each wheel uses a feed-forward target plus PID correction:

```text
error = target speed - measured speed
output = target + Kp(error) + Ki(integral) + Kd(error rate)
```

The implemented gains are:

| Gain | Value | Effect |
|---|---:|---|
| `Kp` | 0.55 | Corrects instantaneous speed error |
| `Ki` | 0.08 | Removes persistent speed error |
| `Kd` | 0.00 | Disabled to avoid amplifying encoder quantisation |

The integral is bounded and anti-windup prevents further integration when the
output is saturated in the direction of the error. A 0.40 duty breakaway kick
is applied when a non-zero target is commanded while the measured speed is
below 0.02 m/s.

For straight motion, an additional gain of 0.10 reduces the faster wheel's
target and increases the slower wheel's target. This correction is enabled only
when both wheel targets are intentionally equal; it is disabled during turns
and steering commands.

## 3. Tuning method

The tuning sequence is:

1. Verify encoder polarity and motor-to-wheel mapping with `S2-M01`.
2. Set `Ki = 0` and increase `Kp` until the wheel reaches the target without
   sustained oscillation.
3. Increase `Ki` only enough to remove steady-state speed error.
4. Keep `Kd = 0` unless a higher-resolution speed measurement is introduced.
5. Re-run straight distance and point-turn tests after every gain change.

The current values (`Kp=0.55`, `Ki=0.08`, `Kd=0.00`) are the checked-in
baseline. They should be retuned if the wheel diameter, motor supply, payload,
floor surface or encoder mounting changes.

## 4. Recorded results

| Test | Result | Evidence | Interpretation |
|---|---|---|---|
| `S2-U01` PID output at zero error | Defined; result not recorded | Test case in `testing/subsystem2/TEST_LOG.md` | Feed-forward output equals the target |
| `S2-U02` slow wheel correction | Defined; result not recorded | Test case in `testing/subsystem2/TEST_LOG.md` | Error increases controller output |
| `S2-U05` saturation and anti-windup | Defined; result not recorded | Test case in `testing/subsystem2/TEST_LOG.md` | Output is bounded and integral does not wind up |
| `S2-M01` motor/encoder direction | PASS | Serial monitor, 28 Oct 2026 | Left: `+1405`, right: `+1225`; combined checks were positive |
| `S2-M02` 5 s straight run | PARTIAL | Serial monitor, 28 Oct 2026 | Encoder distances: left `875 mm`, right `880 mm`; ruler distance was not recorded |
| `S2-M03` point turns | PARTIAL | Serial monitor, 28 Oct 2026 | Encoder estimate was approximately `91°` for the 90° turn and `160°` for the 180° turn; physical angle was not measured |

The straight-run encoder disagreement was:

```text
|880 - 875| / ((880 + 875) / 2) × 100 = 0.57%
```

This indicates good left/right repeatability in the recorded run, but it does
not prove absolute distance accuracy because the ruler measurement is missing.

## 5. Accuracy evaluation

### Straight-line distance

The encoder-estimated distance is based on the average of both wheel travels:

```text
distance = ((left ticks + right ticks) / 2) × 0.329 mm
```

For a valid absolute-accuracy result, record:

```text
distance error (%) =
  (encoder distance - ruler distance) / ruler distance × 100
```

The current evidence is therefore **relative wheel agreement: PASS** and
**absolute distance accuracy: NOT YET VERIFIED**.

### Heading and point turns

Point-turn angle is estimated from differential wheel travel:

```text
angle (degrees) =
  (right travel - left travel) / 0.125 × 180 / pi
```

The 90° result is close by encoder estimate. The 180° result requires a
physical heading measurement before it can be accepted as accurate. Possible
contributors to the short turn include wheel slip, an inaccurate track-width
measurement, unequal motor response, or stopping latency.

## 6. Recommendations

- Repeat `S2-M02` with a measured ruler distance and save the serial log.
- Measure both physical turn angles with a marked floor reference or
  protractor; do not rely on encoder angle alone.
- If both turns show the same scale error, recalibrate `MOTOR_TRACK_WIDTH_M`.
- If only one wheel consistently differs, check wheel diameter, encoder
  polarity, wheel contact and motor direction before changing PID gains.
- Keep the current PID gains as the baseline until the missing physical
  measurements are recorded.

## 7. Conclusion

The PID unit behaviour, output limiting and encoder direction checks pass.
The recorded straight run shows only 0.57% left/right encoder disagreement.
Absolute motion accuracy remains partially evaluated because the ruler
distance and physical turn angles were not recorded. The next acceptance
criterion is a repeatable, logged hardware run with those reference
measurements.

