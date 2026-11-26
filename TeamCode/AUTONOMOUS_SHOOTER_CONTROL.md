# Autonomous shooter control

Team One, Team Two, the Team One backup, and the generic FTC autonomous adapter now use the same **FTCLib PIDF encoder-RPM feedback and battery-compensated motor power** as the latest TeleOp. Autonomous selects the target from distance to the same-quadrant hive each shooting loop. There is no gamepad trim in autonomous.

## Default target and geometry

The normal autonomous reference is `FieldLayout.hive[0]`: robot center **(-12.75, -62.9 inches), front heading -90 degrees**. The rear ball exit is 5.35 inches behind robot center, so its distance to hive center (-12.75, -6) is **51.55 inches**. Team Two starts at its opposite wall shot, with the same distance. Both alliances and all mirrored quadrants receive **0.495 × 6000 = 2970 RPM** at this reference distance.

TeleOp retains its separate (-12.75, -63.25, -90 degrees) reference and 51.90-inch distance. This change calibrates autonomous to its actual route coordinates without moving any autonomous start or waypoint.

`AutonomousShooter` uses the shared `AutoShooterTarget` with the autonomous reference. For heading h in radians and the selected hive (hx, hy):

```
shooterX = robotX - 5.35 * cos(h)
shooterY = robotY - 5.35 * sin(h)
distance = hypot(hx - shooterX, hy - shooterY)
baseRPM = 6000 * configuredReferenceMultiplier + table(distance) - table(referenceDistance)
targetRPM = clamp(baseRPM, 0, 6000)
```

The default configured reference multiplier is **0.495**. Explicit autonomous reference overrides still work; zero disables the shooter. The hive is (±12.75, ±6) in the robot center's quadrant, with positive-side axis ties. Autonomous's canonical role coordinates may be rotated 180 degrees from the physical alliance field frame; symmetric mirrored hives preserve this distance calculation.

The existing table points, cubic interpolation, rear offset and 24.1–79.9-inch lookup clamping are shared with TeleOp. The existing nonmonotonic table shape is retained. This does not estimate ball slip, change the fixed hood, or guarantee a successful shot.

## Feedback loop and feeding

Both hardware adapters use `Mechanisms/Shooter`, which holds one `ShooterPidfControl` instance across updates:

```
currentRPM = motor.getVelocity(DEGREES) / 6
rawPower = PIDF.calculate(currentRPM, targetRPM)
compensatedPower = rawPower * 12 / batteryVoltage
motor.setPower(clamp(compensatedPower, 0, 1))
```

The controller runs on every active output loop, including when the requested RPM stays constant. It reads the current battery voltage each loop. `RUN_WITHOUT_ENCODER` disables the Hub's competing velocity regulator while retaining encoder measurements. The confirmed shooter profile is 28 counts/revolution, 1:1, 6000 RPM. Gains and bounded integral correction are shared with the latest TeleOp; no new gains or blocking spin-up waits are added.

The autonomous state machine still decides whether shooting is authorized and retains its existing aim/tag/inventory checks, timed spin-up, feed windows, spindexer pulse/gap, route, parking and STOP behavior. Invalid RPM, battery, target or computed output commands **zero shooter and feeder power**; exceptions during controller updates attempt both stops. A later healthy loop may resume output if the routine still authorizes shooting. Existing time-based feed attempts continue to use their original timing; a controller fault does not guarantee completion or retry of all balls. There is no new RPM-at-speed feeding interlock.

Existing stationary-shot pose handling is retained: the routine may use its last pose during a transient Pinpoint loss, and `AutonomousShooter` uses the configured reference multiplier if the supplied pose is missing, nonfinite or out of field. This is distinct from TeleOp's fresh-A restart requirement. It does not bypass existing autonomous motion/aim gates. RPM and voltage feedback still apply to a fallback target.

INIT, STOP and zero target command zero motor power and clear PIDF accumulation. Autonomous never sends `setVelocity()` commands. Its legacy `RobotIO.Outputs.shooterPower` field still carries an RPM multiplier, not motor power.

## Telemetry and verification

Autonomous shows target/actual RPM, PIDF sampled RPM, controller state, battery voltage, raw/compensated/applied power, and integral correction. Normal QuantumStorm entries also show fresh shooter-to-hive distance, lookup distance and selected hive. Multiplier displays use three decimals so 0.495 is visible. Shot history records actual feeder pulses, not pulses blocked by a PIDF fault.

`AutonomousShooterChecks` exercises both alliances and Team One, Team Two, backup follow-up and generic routine target paths, including default 2970 RPM, moving targets, spin-up/permission gates, explicit overrides, stationary fallback and STOP. `ShooterPowerChecks` runs the production mechanism and FTCLib controller against instrumented hardware interfaces, checking continuous encoder/voltage updates, feedback response, feed suppression on faults, sensor exceptions, INIT/STOP and rejection of every `setVelocity()` call. Shared PIDF steady-state, TeleOp target/trim, distance, aiming, spindexer and retained old-TeleOp velocity checks cover regression behavior.

The Android build and ten targeted checks passed. Two older checks also fail on the unchanged PR #45 base: `FeedCadenceChecks` expects a 0.50-second gap while the existing production constant is 0.35 seconds; `AutoDebugChecks` fails its stale-detection assertion. Neither underlying implementation is changed here. These are documented separately from the shooter validation. No robot deployment or physical shot/gain tuning was performed.
