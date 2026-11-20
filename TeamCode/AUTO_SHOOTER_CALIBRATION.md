# Physics-based quadrant shooter

Select **Biobuzz-Quantum Storm Auto-Shooter-Power** on the Driver Station. This separate TeleOp uses encoder-regulated `setVelocity()` and the calibrated 28-count, 1:1, 6000 RPM motor configuration. The regular **Biobuzz-Quantum Storm Tele-Op** retains manual RPM adjustment. Autonomous shooter behavior is unchanged.

## Field coordinates and initialization

Coordinates are robot-center positions in inches, with `(0, 0)` at field center and bounds `[-72, 72]` on each axis. This mode uses an absolute field frame, including positive X/Y; it does not inherit the previous autonomous mode's pose or its canonical alliance-relative frame.

The supplied hive back target `(-12.75, -6)` is mirrored into all four quadrants. These assumed mirrored centers should be checked against physical field measurements:

| Robot quadrant | Selected hive back center |
| --- | --- |
| X < 0, Y < 0 | (-12.75, -6) |
| X >= 0, Y < 0 | (12.75, -6) |
| X < 0, Y >= 0 | (-12.75, 6) |
| X >= 0, Y >= 0 | (12.75, 6) |

The selected hive shares the robot center's quadrant. For these symmetric centers this is also the closest hive. Exactly on an axis, ties use the positive side. Crossing an axis recomputes the selected hive each loop; the driver must aim at the new hive. The code assumes that hive is tilted up and ready, and does not detect its tilt or aim the robot.

1. Place the robot at one of the mirrored Team 1 start references `(±12.75, ±62.9)`, with its front facing the Y wall and its rear shooter facing into the field.
2. Press INIT. Default start is `(-12.75, -62.9, -90 degrees)`.
3. On gamepad 1, D-pad LEFT/RIGHT selects negative/positive X; DOWN/UP selects negative/positive Y. Both opposite buttons together leave that axis unchanged. The chosen coordinates and heading appear in telemetry. Heading is -90 degrees at negative Y, +90 degrees at positive Y.
4. Keep the robot still while Pinpoint calibrates, then START. The shooter and feeder remain off during INIT. The selector is disabled after START.

This initialization assigns a known pose; it cannot discover an arbitrary starting location. Reinitializing requires returning the robot to the selected start reference. The mode may be driven from that reference to A, B, C or any other location.

## Equation and calibration

Assumptions: launch angle 70 degrees; ball exit 8 inches above ground; target 60 inches above ground; ball exit 8 inches toward the target from robot center. Distances subtract that offset assuming the rear shooter is aimed directly at the selected hive. Robot heading is not used to project an unaligned ball exit. Air resistance and spin are omitted.

For robot `(x, y)` and selected hive `(hx, hy)`:

```
d = max(0, hypot(x - hx, y - hy) - 8)           // inches from ball exit
v = sqrt(386.09 * d^2 /
         (2 * cos(70 deg)^2 * (d * tan(70 deg) - 52)))  // inches/second
rpm = (v - b) / a
multiplier = rpm / 6000
```

The ball-speed relationship is affine: `v = a * rpm + b`. The coefficients are derived from only the two straight measurements:

- A: robot `(-12.75, -62.9)`; shot distance 48.9 inches; 3000 RPM / multiplier 0.50.
- B: robot `(-12.75, -57)`; shot distance 43 inches; 2940 RPM / multiplier 0.49.

```
a = (requiredBallSpeed(48.9) - requiredBallSpeed(43)) / 60
  = 0.068580457043... inches/second per RPM
b = requiredBallSpeed(43) - a * 2940
  = 13.160922336... inches/second
```

The discarded 0.54 side measurement is not part of this fit. The last requested C position is `(-36, -62.9)`:

| Position | Shot distance | Target RPM | Multiplier |
| --- | ---: | ---: | ---: |
| A (-12.75, -62.9) | 48.9000 in | 3000.00 | 0.5000 |
| B (-12.75, -57) | 43.0000 in | 2940.00 | 0.4900 |
| C (-36, -62.9) | 53.4668 in | 3059.20 | 0.5099 |

Mirroring the X and/or Y signs gives the same distances and RPMs. Fractional multipliers are sent directly; they are not rounded to the old manual 0.01 button steps.

The model is calibrated only between 43 and 48.9 inches. Other achievable distances use the equation with an **EXTRAPOLATED** telemetry label. Ball exit speeds were inferred from the ideal flight equation rather than directly measured; friction, compression, spin and slip can vary. Hardware validation is required, especially outside the calibration interval.

## Invalid solutions and controls

A falling crossing requires `d > 2 * 52 / tan(70 degrees)`, approximately **37.8529 inches**. Equality is an apex-only arrival; shorter distances cannot hit the target on the descending branch at this angle. These positions stop the shooter/feed and show `TOO_CLOSE_FOR_FALLING_SHOT`. Nonfinite/out-of-field positions, invalid Pinpoint samples, and required RPM above 6000 also stop output. The equation is not clamped to a different trajectory or replaced with an arbitrary endpoint speed.

Gamepad 1 retains the default driving/intake controls after START. Gamepad 2 A starts the shooter; B stops it. A held through INIT/START must be released and pressed again. D-pad retains paced forward feeding and reverse control, but in this automatic mode feeding requires the shooter to be ON and the shot solution valid. The existing flower controls and D-pad release guard are preserved. Following an invalid solution, a fresh A press after recovery is required to restart. RPM adjustment through BACK + LB/LT is disabled in this entry because the target is calculated each loop. The driver still controls aiming and when to feed; this mode does not add an RPM-at-speed feeding gate.

Telemetry shows pose, quadrant, hive target, ball-exit distance, calculated RPM/multiplier, actual/commanded RPM and validity/extrapolation status. Normal STOP or an exception during the control loop commands zero velocity and stops the other mechanisms.

## Verification

`DistanceShooterCalibrationChecks` verifies A/B/C in all quadrants, mirrored starting references, nearest-hive selection, deterministic axis ties, continuous RPM at axis crossings, invalid inputs and impossible shots. A field grid independently integrates the commanded trajectories to verify arrival at 60 inches while falling. Existing velocity and spindexer regression checks cover motor scale/API and feed cadence. Run the Android build before deploying; these desktop checks cannot validate physical ball flight or field placement.
