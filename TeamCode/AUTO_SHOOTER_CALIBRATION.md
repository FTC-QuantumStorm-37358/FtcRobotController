# Table-based quadrant shooter

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

1. Place the robot at one of the mirrored Team 1 start references `(±12.75, ±63.25)`, with its front facing the Y wall and its rear shooter facing into the field.
2. Press INIT. Default start is `(-12.75, -63.25, -90 degrees)`.
3. On gamepad 1, D-pad LEFT/RIGHT selects negative/positive X; DOWN/UP selects negative/positive Y. Both opposite buttons together leave that axis unchanged. The chosen coordinates and heading appear in telemetry. Heading is -90 degrees at negative Y, +90 degrees at positive Y.
4. Keep the robot still while Pinpoint calibrates, then START. The shooter and feeder remain off during INIT. The selector is disabled after START.

This initialization assigns a known pose; it cannot discover an arbitrary starting location. Reinitializing requires returning the robot to the selected start reference. The mode may be driven from that reference to A, B, C or any other location.

## Distance and RPM table

The RPM points and FTCLib `InterpLUT` interpolation follow Stealth Robotics' [ShotCalculator.java at commit 7259c01](https://github.com/Stealth-Robotics/AutonomousPrime2026/blob/7259c012a7725d9c79e0abc5648350a549c068e2/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/ShotCalculator.java). The original repository's FIRST copyright and redistribution notice is retained in this repository's root `LICENSE`.

| Shooter-to-hive distance (in) | Target RPM |
| ---: | ---: |
| 24 | 2900 |
| 30 | 3000 |
| 40 | 3100 |
| 47 | 3120 |
| 50 | 3200 |
| 60 | 3300 |
| 70 | 3400 |
| 80 | 3500 |

FTCLib 2.1.1 performs cubic interpolation between these points. We preserve its actual curve, including a small dip from approximately 3104.5 RPM at 42 inches to 3102.4 RPM at 44 inches. As in Stealth's code, the lookup distance is clamped to **24.1–79.9 inches**, avoiding the library's excluded endpoints. Shorter or longer distances keep the corresponding endpoint command (approximately 2901.67 or 3499 RPM) and show `CLAMPED_NEAR` or `CLAMPED_FAR`. Actual distance and lookup distance appear separately in telemetry. No extrapolation is performed.

These are **their calibration values**, originally paired with an adjustable hood. Our hood stays fixed at approximately 70 degrees. The table is a starting point for physical testing, not a guarantee of landing in the hive. This replaces the A/B projectile fit in this entry: the old 0.50/0.49 reference multipliers and falling-trajectory rejection are no longer used. There is no hood servo requirement or hood command.

Like Stealth, we rotate the robot-to-shooter offset into field coordinates before measuring the horizontal distance. We retain our own 5.35-inch rear offset (ball center 3.4 in inside the rear edge) and centered field coordinates, rather than their turret offset and alliance coordinate frame. For robot-center `(x, y)`, front heading `h` (counterclockwise from +X), and selected hive `(hx, hy)`:

```
shooterX = x - 5.35 * cos(h)
shooterY = y - 5.35 * sin(h)
distance = hypot(hx - shooterX, hy - shooterY)
targetRPM = table(clamp(distance, 24.1, 79.9))
multiplier = targetRPM / 6000
```

The offset remains correct while the robot is turning; only when the rear faces the hive does this simplify to center distance minus 5.35 inches. Heading is read in degrees from Pinpoint and converted to radians for trigonometry. Target selection remains based on the robot center's quadrant, even if the shooter offset crosses an axis. No moving-shot/flight-time compensation is applied; aim and stop before feeding.

## Invalid solutions and controls

Nonfinite position or heading, out-of-field robot-center coordinates, and invalid Pinpoint samples block shooter/feed output. A clamped lookup is still a valid RPM command and does not block feeding. There is no projectile-height or falling-arrival check with this empirical table.

Gamepad 1 retains the default driving/intake controls after START. Gamepad 2 A starts the shooter; B stops it. A held through INIT/START must be released and pressed again. D-pad retains paced forward feeding and reverse control, but in this automatic mode feeding requires the shooter to be ON and the shot solution valid. The existing flower controls and D-pad release guard are preserved. Following an invalid solution, a fresh A press after recovery is required to restart. RPM adjustment through BACK + LB/LT is disabled in this entry because the target is calculated each loop. The driver still controls aiming and when to feed; this mode does not add an RPM-at-speed feeding gate.

Telemetry shows pose, quadrant, hive target, ball-exit distance, calculated RPM/multiplier, rear shooter position, lookup distance, actual/commanded RPM and validity/clamping status. Normal STOP or an exception during the control loop commands zero velocity and stops the other mechanisms.

## Verification

`TableShooterCalibrationChecks` runs against the real FTCLib library and checks published RPM points, cubic interpolation, endpoint clamping, mirrored A/B/C distances, heading-dependent offsets, axis ties, invalid inputs, and a field/heading sweep. The legacy `DistanceShooterCalibrationChecks` still covers the retained physics helper, which no longer supplies this entry's RPM. Android compilation verifies TeleOp integration. Desktop checks do not validate physical ball flight or odometry placement.
