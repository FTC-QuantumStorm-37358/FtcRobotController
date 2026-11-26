# Latest shooter power control

Driver Station entry: **Biobuzz-Quantum Storm Auto Power Control-latest 11/25**.

This TeleOp combines location-based targeting with **FTCLib 2.1.1 `PIDFController`**, encoder RPM feedback, battery-compensated `setPower()`, and gamepad 2 Y rear-hive aiming. The previous standard TeleOp remains available as **Biobuzz-Quantum Storm Tele-Op-old** with its existing table/SDK velocity behavior. The redundant Auto-Shooter-Power entry is removed. Autonomous preselects the latest entry; its shooting algorithm and table are unchanged.

The latest entry uses its own Pinpoint sample for both aiming and shooter targeting, once per loop. Its A-button/readiness gate uses that same distance solution, rather than the old superclass's uninitialized odometry. Invalid aiming odometry or an invalid shot pose stops the automatic target, shooter and feeder. PIDF motor/voltage faults also stop shooter/feed. Recovery requires a fresh A press; it never starts the shooter automatically.

## Controls

- At the default starting pose, the automatic multiplier is **0.495**, meaning **2970 RPM**, with zero driver trim. INIT leaves the motor off. The distance curve is offset to this reference; see [distance calibration](AUTO_SHOOTER_CALIBRATION.md).
- Gamepad 2 A alone starts the shooter; B stops it and cancels an active aim. Release A after INIT/controller assignment before pressing it to start.
- Tap gamepad 2 Y to turn the rear shooter toward the hive in the robot's quadrant. Y never starts the shooter. Move a gamepad 1 drive stick beyond 0.10, press gamepad 2 B, or hold gamepad 2 START to cancel the turn.
- Hold BACK and tap LB/LT to add/subtract **0.005** (30 RPM) of relative trim. The trim remains added to the automatic distance target as the robot moves, during A/B stops, and across sensor recovery; a new INIT resets it to zero. Trim storage is bounded to +/-1.0 and the final target to 0–6000 RPM. Holding a shoulder does not repeat; LB+LT cancels, START suppresses adjustment, and releasing START/pressing BACK while a shoulder is held does not apply a step. A shoulder held through START must be released first.
- Existing D-pad/flower controls remain. In this entry, feeding requires the shooter ON at a nonzero target, a healthy controller and no active aiming turn. Feeding is also blocked on the aim completion/fault/cancellation loop, then returns to the manual D-pad controls. This is not an at-speed feed interlock.
- Invalid odometry or a controller fault stops shooter/feed. Release A and press it again after fixing the fault to retry.

## Field setup and rear-hive aiming

This entry requires `odometry_1` (goBILDA Pinpoint). Its origin is field center and X/Y are in inches. Before INIT, place the robot at one of the mirrored Team 1 starting references `(±12.75, ±63.25)`, front facing the Y wall and rear shooter facing into the field. Default is `(-12.75, -63.25, -90 degrees)`.

During INIT, gamepad 1 D-pad LEFT/RIGHT selects the negative/positive X starting reference, and DOWN/UP selects negative/positive Y. Heading is -90 degrees at negative Y and +90 at positive Y. Check the displayed pose, keep the robot still during Pinpoint calibration, then START. This assigns a known absolute field pose; it does not locate an arbitrarily placed robot or inherit the prior autonomous pose. Drive to your test location after START.

The shared Pinpoint pod offsets use the measured 17.5-inch robot length and a centered shooter. Looking toward the intake, the forward-measuring pod is 6 inches right of the centerline, so `PINPOINT_X_POD_OFFSET_MM = -152.4`. The sideways-measuring pod is 1 inch from the rear edge, or 7.75 inches behind center, so `PINPOINT_Y_POD_OFFSET_MM = -196.85`. Its 4-inch displacement to the left does not enter the Y offset: Pinpoint needs each wheel's perpendicular distance from the tracking center. Measure at the wheel's ground contact point.

These replace the incorrect +4.125-inch X and -7-inch Y settings. The reported stationary half-turn from `(-36.7, -48.1, -88.2 degrees)` to `(-16.6, -47.5, 91.8 degrees)` implies about 10.05 inches of X-pod offset error, consistent with changing +4.125 inches to -6 inches. That run implies roughly 7.02 inches behind center for the Y pod, while the physical rear-edge measurement gives 7.75 inches. The code uses the physical measurement; repeat the marked-center turn test to verify the wheel contact point and remaining drift. This is a pod-geometry correction shared by modes using `PinpointOdometry`, not a quadrant-specific aim correction. Re-INIT at the known start after deploying; an already accumulated pose error is not repaired by changing offsets.

Gamepad 2 Y performs a one-shot, nonblocking turn using:

```
hiveX = x < 0 ? -12.75 : 12.75
hiveY = y < 0 ? -6 : 6
frontTargetDegrees = wrapDegrees(degrees(atan2(hiveY - y, hiveX - x)) + 180)
headingError = wrapDegrees(frontTargetDegrees - measuredFrontHeading)
```

The robot's rear points along `frontTargetDegrees + 180`. The 5.35-inch rear shooter offset lies on that aim line, so it does not change the final heading. This assumes the shooter is centered on the robot's rear axis. The same-quadrant target is also the nearest among the four mirrored hive centers. Exact axis ties select the positive side; target selection is updated each loop.

Only rotation is commanded; no translation is requested. Heading is CCW-positive while the mecanum turn input is clockwise-positive, so the turn command has the opposite sign from heading error. Proportional gain is 0.020 power/degree, with minimum/maximum turn effort 0.06/0.40. Large turns use up to 40% power; a 10-degree error uses 20%, and a 2-degree error retains the original 6% minimum. These values are in `DriveChain/RearHiveAim.java` and require physical turn-sign/response verification. The turn ends after the estimated heading stays within **1 degree for 0.20 seconds**. This is an odometry tolerance, not a guarantee of perfect physical aim or a camera-based alignment check.

Invalid/nonfinite/out-of-field odometry, a target within the 5.35-inch shooter offset, or a **5-second timeout** stops the turn. Recovery does not restart it; press Y again. Driver override and B/START cancellation win over a simultaneous aim request. A Y held through INIT/START cannot start aiming until it is released. Other mechanisms and the PIDF loop continue updating while aiming; feeding pauses and no shot is fired automatically.

Telemetry adds current X/Y/heading, aiming state, selected hive, target front heading, heading error and clockwise turn power. This function does not check hive tilt or shot reachability; the driver selects when to shoot.

If aiming stops with `INVALID_POSE`, `Aim blocked reason` distinguishes `ODOMETRY_SAMPLE_INVALID`, `NONFINITE_POSE`, `OUTSIDE_FIELD` (center X/Y beyond +/-72 inches), and `INVALID_CLOCK`. `Pinpoint at aim fault` and `Pose at aim fault` retain the first failure's sensor status, validity and last available pose until another Y request. An invalid sample's displayed pose is last-known data, not a fresh measurement. This preserves evidence of transient faults even if current Pinpoint telemetry recovers. Check sensor status/wiring for invalid samples, or the initialized reference and pod measurements for out-of-field coordinates; a fresh Y press is needed after recovery. The validity checks and zero-power response remain enforced.

## Motor control

The REV Hub, not a chip inside the motor, implements the SDK's built-in velocity loop. This entry uses `RUN_WITHOUT_ENCODER` so that loop does not compete with software PIDF. Encoder readings are still available in this mode. The confirmed motor profile remains 28 counts/revolution, 1:1, 6000 RPM.

Each active control update uses:

```java
currentRPM = motor.getVelocity(AngleUnit.DEGREES) / 6.0;
baseRPM = table(distance) + (2970 - table(51.9));
targetRPM = Math.max(0, Math.min(6000, baseRPM + 6000 * driverTrim));
calculatedPower = pidfController.calculate(currentRPM, targetRPM);
calculatedPower *= 12.0 / batteryVoltage;
motor.setPower(Math.max(0, Math.min(1, calculatedPower)));
```

FTCLib's feedforward contribution is **kF × targetRPM**, not a constant kF. Gains must use RPM error and motor-power output units. `getVelocity()` supplies the measured RPM; `PIDFController` calculates the command and does not read hardware itself.

Battery voltage is the minimum usable reading across the Hub voltage sensors on the shared battery. Values outside 6–16.8 V and nonfinite readings are ignored. If none is usable, or RPM/target/calculated output is invalid, the motor receives zero power and feeding stops. Final clipping happens **after** voltage compensation. Overspeed reduces power toward zero rather than reversing the flywheel.

Integral correction addresses persistent RPM shortfall, such as the reported 2880 target / 2700 actual and 3180 target / 3000 actual. With I=0, a repeated proportional calculation can settle below target. The latest entry uses `kI=0.0002` power per RPM-second. Accumulation is enabled only within 400 RPM of target, with the I contribution bounded to +/-0.15 power **before voltage compensation**. Outside that zone the accumulator is clamped to zero, avoiding buildup during spin-up or large target changes. FTCLib's default +/-1 bound is in accumulated-error units; the code sets explicit bounds of +/-750 RPM-seconds for this gain. Reaching the target retains the learned correction; overspeed reduces it. This is feedback correction, not an automatic change of the RPM multiplier or an added 180 RPM to the target.

On STOP, zero target, or fault, the motor receives zero power directly and the PIDF state resets. No sensor read or PID calculation is needed to stop. Saturation clears accumulated integral error. The latest entry never sends `setVelocity()` commands, including during INIT and STOP.

## Initial gains and tuning

`Constants.java` contains these **provisional, not hardware-tuned** values:

| Setting | Value | Meaning |
| --- | ---: | --- |
| `SHOOTER_AUTO_START_MULTIPLIER` | 0.495 | Default-pose target, 2970 RPM |
| `SHOOTER_AUTO_TRIM_INCREMENT` | 0.005 | Relative driver adjustment, 30 RPM per press |
| `SHOOTER_PIDF_P` | 0.0001 | 100 RPM error adds 0.01 power before voltage compensation |
| `SHOOTER_PIDF_I` | 0.0002 | Accumulates persistent RPM error near target |
| `SHOOTER_PIDF_INTEGRAL_ZONE_RPM` | 400 | Clear I outside this error range |
| `SHOOTER_PIDF_MAX_INTEGRAL_POWER` | 0.15 | Maximum absolute I contribution before voltage compensation |
| `SHOOTER_PIDF_D` | 0 | Derivative disabled initially |
| `SHOOTER_PIDF_F` | 1 / 6000 | Nominal 6000 RPM at full power and 12 V |
| `SHOOTER_PIDF_REFERENCE_VOLTS` | 12 | Reference voltage used to define the gains |

At 3000 RPM measured and requested, the provisional feedforward is 0.50 power at 12 V, approximately 0.571 at 10.5 V and 0.429 at 14 V. Positive RPM error adds proportional effort, and near-target error accumulates integral effort. Therefore a multiplier of 0.50 does not imply a fixed 0.50 power command.

Observe steady RPM before feeding and the drop/recovery after a ball. Tune F for the steady motor load, P for recovery, and I for persistent error; D remains zero. The new I gain is a starting value validated with synthetic motor models, not a completed physical tuning result. If I remains at its limit with a persistent error, further F/load calibration is needed; if final power is already 1.0, the controller cannot request more voltage. This software controller is not inherently more accurate than the Hub controller: encoder quality, loop timing, gain tuning and flywheel mechanics determine the result. Wheel RPM also does not directly measure ball exit speed.

Telemetry separates automatic base RPM/multiplier, persistent driver trim, final trimmed RPM/multiplier, distance and lookup clamping. It also shows target/actual RPM, PIDF status, the sampled RPM used by the controller, voltage during active control, raw/compensated/applied power, integral contribution/limit and configured gains. Controller sample values are unavailable until the first active command and retain the last sample while stopped. Regular encoder telemetry remains visible while stopped.

## Validation

`AutoShooterTargetChecks` verifies the 51.9-inch/2970-RPM starting reference in all four quadrants, A/B/C geometry, preservation of the existing table's RPM differences, persistent trim across movement, bounds, reset, and invalid-pose rejection. `ShooterTeleOpTargetChecks` exercises the real TeleOp hooks and gamepad edges, own-solution readiness, precise +/-0.005 trim, movement/fault/STOP persistence, PIDF output wiring, voltage compensation and menu/autonomous preselection names. These checks run on the desktop without starting an OpMode or motors.

`ShooterPidfControlChecks` verifies the real FTCLib controller's encoder units, feedback, 12/V compensation, clipping, STOP/reset and sensor faults, and rejects any `setVelocity()` call. `ShooterPidfSteadyStateChecks` reproduces the reported 180-RPM deficits with I disabled, then checks synthetic recovery across voltages/loads, bounded I, saturation and reset. `RearHiveAimChecks` covers quadrant geometry, turn signs, simulated convergence, cancellation and faults. Table/interpolation, spindexer and retained SDK velocity checks also run against production classes. An Android build verifies packaging. Physical shot calibration, PIDF tuning and robot verification remain required; simulated convergence does not establish hardware performance.
