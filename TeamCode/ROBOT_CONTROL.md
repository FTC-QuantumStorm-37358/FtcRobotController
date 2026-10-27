# Standalone BIOBUZZ robot control

All robot controller sources are in this module's normal Java source tree:

- `src/main/java/org/firstinspires/ftc/teamcode/biobuzz/`: state machine, field poses, navigation, Hive-tag interpretation, sensor/output interfaces and controller lifecycle.
- Its `ftc/` subpackage: FTC hardware integration, calibration configuration, SDK lifecycle and safe shutdown/Pinpoint helpers.
- `quantumstorm/Auto/`: the four normal Red/Blue Team One/Two hardware entries.

No simulator folder, browser, local server, desktop JVM bridge or machine-specific source link is required to build or run the robot app. Package names remain unchanged, so existing imports keep working. Simulator-only recovery experiments and desktop code are not compiled into TeamCode.

## Normal autonomous roles

Team One fires its preload cycle, waits with outputs off until 26 seconds, then drives directly to parking. Team Two assumes its partner caused the first tilt, fires preloads, collects Garden pollen, fires again, allows settling time, waits, then starts parking at 26 seconds. Hardware entries do not enable extra post-routine collection.

These four entries use odometry and do not require Limelight or a camera tilt servo. Unknown or zero inventory does not skip the bounded timed firing cycle. Counts are not fabricated; timed feeding/pickup is an attempt, not confirmed scoring or collection. Physical travel and firing still require calibration.

## Before running

Check configured motor/servo names and directions, Pinpoint wiring and pod offsets/directions, actual drive axes, start pose and shooter power. Place the robot at the INIT-displayed role start and keep it still during calibration. Current local offsets are -170 / -150 mm and shooter default is +0.55; verify these on your robot.

Normal QuantumStorm entries initialize live shooter/feed/intake hardware. Clear the mechanism and firing path before START. They retain manual STOP, interruption checks, guarded odometry reads, collision/progress limits and an unconditional 30-second stop. Parking latches outputs off within two inches without final heading correction. No code can guarantee arrival from an arbitrary or miscalibrated physical position.

The separate Red/Blue shared diagnostic entries use `ftc/FtcHardwareConfig.java`, which defaults to `calibrated=false`. Measure and configure all required hardware/field values before enabling it. Optional vision needs correct pipeline/tag/camera calibration; the four normal camera-free modes do not confirm Hive tilts.

## TeleOp shooter controls

On gamepad 2, tap **A** to start the single shooter and **B** to stop it. A second A press does not toggle it off; B wins when both are pressed.

Hold **Start**, then tap **LB** to increase signed power or **LT** to decrease it by `SHOOTER_POWER_INCREMENT` (0.05). Each press changes power once; holding a shoulder button does not repeat. Pressing Start after a shoulder button is already held does not change power: release and press the shoulder again. Simultaneous LB/LT requests cancel. Telemetry shows whether adjustment is locked and the previous/current power.

Release Start before using A/B for the shooter. Start+A/B is reserved for Driver Station controller assignment, and a fresh A press is required afterward. The Logitech F310 Mode button changes controller behavior internally and is not used as the modifier. Shooter 2 remains removed, so X/Y and RB/RT do not control a second shooter. D-pad up/down retains the existing feeder controls.

## Build

Open the repository root in Android Studio, sync Gradle and build TeamCode normally. From a terminal with an appropriate JDK/Android SDK:

```sh
./gradlew :TeamCode:compileDebugJavaWithJavac
```

The local simulator, if separately present, reads the SDK-free core from TeamCode. The dependency is simulator-to-TeamCode, never TeamCode-to-simulator.
