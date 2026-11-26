# Latest distance-based shooter

Select **Biobuzz-Quantum Storm Auto Power Control-latest 11/25**. This entry consolidates automatic distance targeting, FTCLib PIDF RPM feedback, battery compensation, relative driver trim and Y rear-hive aiming. It sends `setPower()` in `RUN_WITHOUT_ENCODER` while still reading the encoder. See [PIDF and aiming details](SHOOTER_PIDF_TEST.md).

**Biobuzz-Quantum Storm Tele-Op-old** retains the previous table/SDK `setVelocity()` behavior. The separate **Auto-Shooter-Power** entry has been removed. Autonomous also uses the shared distance target and PIDF/battery power control, with its own start-reference calibration; see [autonomous shooter control](AUTONOMOUS_SHOOTER_CONTROL.md).

## Field coordinates and initialization

Coordinates are robot-center positions in inches, with `(0, 0)` at field center and bounds `[-72, 72]` on each axis. This mode does not inherit a previous autonomous pose.

| Robot quadrant | Closest hive back center |
| --- | --- |
| X < 0, Y < 0 | (-12.75, -6) |
| X >= 0, Y < 0 | (12.75, -6) |
| X < 0, Y >= 0 | (-12.75, 6) |
| X >= 0, Y >= 0 | (12.75, 6) |

The hive shares the robot center's quadrant; axis ties select the positive side. This is the nearest of the four symmetric hive centers measured from the robot center. Selection is updated each loop. The code assumes the selected hive is tilted up and ready; it does not detect tilt.

1. Place the robot at a mirrored Team 1 start `(±12.75, ±63.25)`, front facing the Y wall and rear shooter facing into the field.
2. Press INIT. The default is `(-12.75, -63.25, -90 degrees)`.
3. Gamepad 1 D-pad LEFT/RIGHT chooses the X sign; DOWN/UP chooses the Y sign. The start heading is -90 degrees at negative Y, +90 degrees at positive Y. Opposite D-pad buttons together leave that axis unchanged.
4. Keep still while Pinpoint calibrates, then START. Shooter/feed stay OFF during INIT. Drive to other locations after START.

This assigns a known field pose; it cannot discover an arbitrary start. Reinitializing requires returning the robot to the selected reference.

## Distance, table and default calibration

The measured ball exit is 3.4 inches inside the rear edge of the 17.5-inch robot, giving a **5.35-inch rear offset**. For robot center `(x, y)`, front heading `h` (radians, counterclockwise from +X), and hive `(hx, hy)`:

```
shooterX = x - 5.35 * cos(h)
shooterY = y - 5.35 * sin(h)
distance = hypot(hx - shooterX, hy - shooterY)
```

At the default start, the exit is `(-12.75, -57.90)` and the horizontal distance to `(-12.75, -6)` is **51.90 inches**. All mirrored starts have that distance. Pinpoint headings are converted from degrees to radians before this calculation. Distance uses the actual rear exit while turning; when aimed it simplifies to robot-center distance minus 5.35 inches.

The shared table originated in Stealth Robotics' [ShotCalculator.java at commit 7259c01](https://github.com/Stealth-Robotics/AutonomousPrime2026/blob/7259c012a7725d9c79e0abc5648350a549c068e2/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/ShotCalculator.java). The root `LICENSE` retains its FIRST notice. Our current table already contains team changes at 50/60/70/80 inches:

| Distance (in) | Shared table RPM, before latest-entry bias |
| ---: | ---: |
| 24 | 2900 |
| 30 | 3000 |
| 40 | 3100 |
| 47 | 3120 |
| 50 | 2910 |
| 60 | 2935 |
| 70 | 3300 |
| 80 | 3400 |

FTCLib 2.1.1 uses cubic interpolation, not linear interpolation. Distances clamp to **24.1–79.9 inches** to avoid excluded endpoints. The table is not monotonic, particularly the drop from 47 to 50 inches; the latest entry preserves that existing curve. Clamped distances remain usable and show `CLAMPED_NEAR/FAR`. No extrapolation or flight-time compensation is applied.

`AutoShooterTarget` applies a constant RPM bias only in the latest TeleOp:

```
baseRPM(distance) = table(distance) + [2970 - table(51.90)]
baseMultiplier = baseRPM / 6000
finalMultiplier = clamp(baseMultiplier + driverTrim, 0, 1)
targetRPM = 6000 * finalMultiplier
```

This makes the default-location base multiplier **0.495**, preserving the table's RPM differences at other distances. The shared lookup points remain unchanged. Autonomous applies the same bias method at its own 51.55-inch reference; the old TeleOp keeps the original unshifted table. Our fixed hood remains approximately 70 degrees; this is a calibration starting point, not a guarantee of landing in the hive.

## Controls and compensation

Gamepad 2 **A** starts the shooter; **B** stops it and cancels aiming. **Y** turns the rear shooter toward the selected hive without starting the shooter. A/Y held across INIT/START must be released before a new request.

Hold **Back**, then tap **LB** to add **+0.005 / +30 RPM** of trim or **LT** past halfway to add **-0.005 / -30 RPM**. For example, at the default pose: zero trim gives 0.495; one LB gives 0.500; two give 0.505. Moving preserves the trim on top of the newly calculated distance target. B/A stops/restarts and fault recovery preserve it; a new INIT resets it. Trim storage is bounded to +/-1.0; the final target is clamped to 0–6000 RPM.

Shoulders act once per fresh press, including after being held through START. Back must be held when the shoulder is pressed; pressing Back after an already-held shoulder does not step. Simultaneous LB/LT cancels, and START suppresses adjustments. Trim never starts a stopped shooter.

The final requested RPM enters the same feedback loop:

```
currentRPM = motor.getVelocity(DEGREES) / 6
power = PIDF.calculate(currentRPM, targetRPM)
power *= 12 / batteryVoltage
motor.setPower(clamp(power, 0, 1))
```

Trim changes the RPM setpoint, not motor output after regulation. Bounded integral correction remains enabled. Battery voltage changes the power needed to reach the requested RPM, not the desired RPM itself.

Invalid Pinpoint samples, nonfinite/out-of-field poses or PIDF/voltage faults stop shooter/feed. A fresh A after recovery is required. Existing drive, intake, flower, pulsed feed and D-pad release guards remain. Feed requires shooter ON at a nonzero target and a valid solution/controller, and pauses during aiming. There is no measured-RPM-at-speed feeding interlock.

Telemetry shows pose, selected hive, actual/lookup distance, base RPM/multiplier, driver trim, final trimmed target, measured RPM, battery voltage and PIDF/applied power. STOP and exceptions in the control loop zero motor power and stop the other mechanisms.

## Verification

`AutoShooterTargetChecks` covers default calibration, mirrored starts and A/B/C, continuous table targeting, trim persistence/bounds/reset and invalid poses. `ShooterTeleOpTargetChecks` covers actual gamepad/target/output hooks, PIDF/battery wiring and menu names. `TableShooterCalibrationChecks` covers the current team table, cubic interpolation, endpoint clamps, headings, quadrants and field sweep. The Android build and PIDF, aiming, spindexer and SDK velocity checks validate the combined implementation. Physical landing accuracy and gain tuning still require robot testing.
