# New Shooter Power Control test

Driver Station entry: **Biobuzz-Quantum Storm New Shooter Power Control**.

This separate manual TeleOp tests **FTCLib 2.1.1 `PIDFController`** with motor power output. It does not change the controller used by the regular TeleOp, the quadrant Auto-Shooter-Power entry, or autonomous modes. The test now initializes Pinpoint odometry for gamepad 2 Y rear-hive aiming and retains the existing two-driver drive/intake/flower controls. RPM remains a manual target; aiming does not enable distance-based RPM adjustment.

The manual entry explicitly preserves its requested multiplier, enables BACK + LB/LT adjustment, and makes its target available to the A-button start gate. It does not use the normal TeleOp's distance-target readiness: that superclass owns a separate odometry object which this entry does not initialize. Invalid aiming odometry stops the aim assist; PIDF motor/voltage faults still stop the shooter and feeder and require a fresh A press.

## Controls

- Default target multiplier: **0.48**, meaning **2880 RPM**. INIT leaves the motor off.
- Gamepad 2 A alone starts the shooter; B stops it and cancels an active aim. Release A after INIT/controller assignment before pressing it to start.
- Tap gamepad 2 Y to turn the rear shooter toward the hive in the robot's quadrant. Y never starts the shooter. Move a gamepad 1 drive stick beyond 0.10, press gamepad 2 B, or hold gamepad 2 START to cancel the turn.
- Hold BACK and tap LB/LT to increase/decrease the RPM multiplier by 0.01 (60 RPM). The test target range is 0 to 6000 RPM; it never reverses the shooter.
- Existing D-pad/flower controls remain. In this entry, feeding requires the shooter ON at a nonzero target, a healthy controller and no active aiming turn. Feeding is also blocked on the aim completion/fault/cancellation loop, then returns to the manual D-pad controls. This is not an at-speed feed interlock.
- A controller fault stops shooter/feed. Release A and press it again after fixing the fault to retry.

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

The REV Hub, not a chip inside the motor, implements the SDK's built-in velocity loop. This test switches to `RUN_WITHOUT_ENCODER` so that loop does not compete with software PIDF. Encoder readings are still available in this mode. The confirmed motor profile remains 28 counts/revolution, 1:1, 6000 RPM.

Each active control update uses:

```java
currentRPM = motor.getVelocity(AngleUnit.DEGREES) / 6.0;
targetRPM = 6000 * multiplier;
calculatedPower = pidfController.calculate(currentRPM, targetRPM);
calculatedPower *= 12.0 / batteryVoltage;
motor.setPower(Math.max(0, Math.min(1, calculatedPower)));
```

FTCLib's feedforward contribution is **kF × targetRPM**, not a constant kF. Gains must use RPM error and motor-power output units. `getVelocity()` supplies the measured RPM; `PIDFController` calculates the command and does not read hardware itself.

Battery voltage is the minimum usable reading across the Hub voltage sensors on the shared battery. Values outside 6–16.8 V and nonfinite readings are ignored. If none is usable, or RPM/target/calculated output is invalid, the motor receives zero power and feeding stops. Final clipping happens **after** voltage compensation. Overspeed reduces power toward zero rather than reversing the flywheel.

Integral correction addresses persistent RPM shortfall, such as the reported 2880 target / 2700 actual and 3180 target / 3000 actual. With I=0, a repeated proportional calculation can settle below target. The test entry now uses `kI=0.0002` power per RPM-second. Accumulation is enabled only within 400 RPM of target, with the I contribution bounded to +/-0.15 power **before voltage compensation**. Outside that zone the accumulator is clamped to zero, avoiding buildup during spin-up or large target changes. FTCLib's default +/-1 bound is in accumulated-error units; the code sets explicit bounds of +/-750 RPM-seconds for this gain. Reaching the target retains the learned correction; overspeed reduces it. This is feedback correction, not an automatic change of the RPM multiplier or an added 180 RPM to the target.

On STOP, zero target, or fault, the motor receives zero power directly and the PIDF state resets. No sensor read or PID calculation is needed to stop. Saturation clears accumulated integral error. The new test entry never sends `setVelocity()` commands, including during INIT and STOP.

## Initial gains and tuning

`Constants.java` contains these **provisional, not hardware-tuned** values:

| Setting | Value | Meaning |
| --- | ---: | --- |
| `SHOOTER_PIDF_TEST_DEFAULT_MULTIPLIER` | 0.48 | Initial target, 2880 RPM |
| `SHOOTER_PIDF_P` | 0.0001 | 100 RPM error adds 0.01 power before voltage compensation |
| `SHOOTER_PIDF_I` | 0.0002 | Accumulates persistent RPM error near target |
| `SHOOTER_PIDF_INTEGRAL_ZONE_RPM` | 400 | Clear I outside this error range |
| `SHOOTER_PIDF_MAX_INTEGRAL_POWER` | 0.15 | Maximum absolute I contribution before voltage compensation |
| `SHOOTER_PIDF_D` | 0 | Derivative disabled initially |
| `SHOOTER_PIDF_F` | 1 / 6000 | Nominal 6000 RPM at full power and 12 V |
| `SHOOTER_PIDF_REFERENCE_VOLTS` | 12 | Reference voltage used to define the gains |

At 3000 RPM measured and requested, the provisional feedforward is 0.50 power at 12 V, approximately 0.571 at 10.5 V and 0.429 at 14 V. Positive RPM error adds proportional effort, and near-target error accumulates integral effort. Therefore a multiplier of 0.50 does not imply a fixed 0.50 power command.

Observe steady RPM before feeding and the drop/recovery after a ball. Tune F for the steady motor load, P for recovery, and I for persistent error; D remains zero. The new I gain is a starting value validated with synthetic motor models, not a completed physical tuning result. If I remains at its limit with a persistent error, further F/load calibration is needed; if final power is already 1.0, the controller cannot request more voltage. This software controller is not inherently more accurate than the Hub controller: encoder quality, loop timing, gain tuning and flywheel mechanics determine the result. Wheel RPM also does not directly measure ball exit speed.

Telemetry shows target/actual RPM, PIDF status, the sampled RPM used by the controller, voltage during active control, raw/compensated/applied power, integral contribution/limit and configured gains. Controller sample values are unavailable until the first active command and retain the last sample while stopped. Regular encoder telemetry remains visible while stopped.

## Validation

`ShooterTeleOpTargetChecks` exercises the real TeleOp target hooks through the base class to verify that the PIDF manual target and A-button readiness survive the normal TeleOp's automatic targeting behavior, speed adjustments persist, and the automatic entries retain their own readiness gates. `ShooterPidfControlChecks` runs the actual FTCLib controller with instrumented FTC motor/voltage interfaces. It verifies INIT/STOP power, encoder units, the 2880 RPM default and 3000 RPM test target, feedback under load/overspeed, 12/V compensation, clipping order, integral reset, restart and invalid/missing sensors. It rejects any use of the Hub `setVelocity()` API by this controller. `RearHiveAimChecks` verifies rear-facing geometry in all quadrants, shortest-angle wraparound, motor turn signs, simulated closed-loop convergence, settling dwell, cancellation, invalid poses, quadrant changes and timeout. `ShooterPidfSteadyStateChecks` runs the actual FTCLib/controller against synthetic motor models: it reproduces both reported 180 RPM deficits with I disabled, verifies recovery at 10.5/12/14 V and another load, and checks I bounds, spin-up suppression, saturation, STOP and fault reset. Test-only reflection advances FTCLib's internal clock without sleeping or replacing its control algorithm. These simulated results do not establish physical convergence time. A successful Android build verifies that the dependency and new entry package together. Physical tuning and robot testing are still required.
