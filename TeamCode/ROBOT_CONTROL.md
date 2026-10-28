# Standalone BIOBUZZ robot control

The robot app builds entirely from TeamCode. SDK-free controllers, field geometry, navigation and sensor/output contracts are in `src/main/java/org/firstinspires/ftc/teamcode/biobuzz/`; the `ftc/` subpackage contains SDK adapters. Only **Team 1 Auto** and **Team 2 Auto** are enabled under `quantumstorm/Auto/`. No simulator directory, browser or laptop connection is required to build or run the robot app.

## Two modes, no alliance setup

Select **Team 1 Auto** or **Team 2 Auto**, place the robot at the matching physical role start on either alliance, press INIT, keep it still for odometry setup, then START. There is no gamepad input, color picker or confirmation step. Legacy Red/Blue entries and autonomous diagnostics retain their source but are `@Disabled`; TeleOp is unchanged.

Both halves use one canonical route frame anchored by the role start, not absolute field-color coordinates. Rotating the physical setup 180 degrees produces the same body-relative driving commands, Garden/Flower pickup, wall clearance and parking half. Do not point the robot the opposite way to compensate: preserve the existing role's physical front/rear orientation. The numeric local route reference shown on Blue is not the absolute Blue field pose.

Team Two recognizes a single visible Hive tag family automatically at its wall shooting pose. It must see at least two distinct raised far-Cell tags with calibrated heights, centered within 10 degrees of the camera axis at the shot pose. Off-axis tags from the adjacent Hive do not identify the target. If both families have centered raised pairs, the view is ambiguous and it does not lock/shoot. The rearward camera aim must be calibrated to center the intended Hive. Once locked for the run, the other family is ignored; Blue tag IDs are aliased to the same canonical role geometry without changing measured angles, heights or frame timestamps. Existing fresh/stable tag, four-pollen, odometry and parking checks remain enforced. This is not a general guarantee of Hive identity: correct physical start and camera aiming are still required. No calibration flag is enabled automatically.

The simulator may retain Red/Blue test cases to verify symmetry; they are not additional enabled robot OpModes.

## Current autonomous roles

- **Team One:** shoot four preload feed attempts immediately, drive to the alliance Garden, collect yellow pollen using live Limelight targets, retain the collected load and park. Normal Garden pickup does not wait for Hive-tag confirmation. The older spilled-pollen recovery path remains an explicit opt-in controller test, not the normal routine.
- **Team Two:** drive from its side-wall start to the opposite-wall shooting pose, verify four type-confirmed pollen and fresh raised own-Hive far-Cell tags, shoot, trigger the nearest end-wall Flower, clear its outlet and collect released pollen with Limelight. Return to the same wall and shoot the second batch only after measuring four pollen and rechecking the tags. No Garden pickup is used by this role.
- **Safety:** parking overrides pickup/feeding by 26 seconds, or earlier when return-time reserve requires. All outputs stop at 30 seconds. Parking accepts a two-inch position tolerance, preserves arrival heading and latches outputs OFF without spinning to correct the final heading. Odometry errors, manual STOP and interruption still stop safely; physical arrival cannot be guaranteed when obstructed or miscalibrated.

The Flower trigger is deliberate contact: at its clearance pose, Team Two applies 12% strafe for no more than 0.4 seconds, then clears the post and approaches the rolling pollen from the open lane. Calibrate this contact and field geometry on the real robot before use.

## Corrected field setup

## TeleOp shooter controls

On gamepad 2, tap **A** to start the single shooter and **B** to stop it. A second A press does not toggle it off; B wins when both are pressed.

Hold **Start**, then tap **LB** to increase signed power or **LT** to decrease it by `SHOOTER_POWER_INCREMENT` (0.05). Each press changes power once; holding a shoulder button does not repeat. Pressing Start after a shoulder button is already held does not change power: release and press the shoulder again. Simultaneous LB/LT requests cancel. Telemetry shows whether adjustment is locked and the previous/current power.

Release Start before using A/B for the shooter. Start+A/B is reserved for Driver Station controller assignment, and a fresh A press is required afterward. The Logitech F310 Mode button changes controller behavior internally and is not used as the modifier. Shooter 2 remains removed, so X/Y and RB/RT do not control a second shooter. D-pad up/down retains the existing feeder controls.

## Build
All positions below are robot-center coordinates in inches, origin at field center, with counterclockwise headings in degrees. The corrected initial Hive tilt places Team One on the opposite wall from the earlier layout. Its front faces the wall and its rear-facing shooter points toward the Hive.

| Pose | Red | Blue |
| --- | --- | --- |
| Team One start / preload shot | (-12.75, -62.9, -90) | (12.75, 62.9, 90) |
| Team Two side-wall start | (-62.9, 6, -90) | (62.9, -6, 90) |
| Team Two shot, both batches | (-12.75, 62.9, 90) | (12.75, -62.9, -90) |
| Team One parking | (-59, 21) | (59, -21) |
| Team Two parking | (-59, 51) | (59, -51) |

Team One takes the parking half nearest its corrected start; Team Two takes the other half. These are modeled chassis-center targets overlapping the narrow alliance area, not a guarantee both complete robot footprints fit inside separate half-stripes. Coordinate teammate clearance; there is no live teammate avoidance.

Corrected initial raised clusters are Red 34-37 and Blue 42-45. After the partner's first tilt, Team Two verifies the opposite clusters: Red 30-33 and Blue 38-41. Tags identify a Cell; measured heights, not IDs alone, establish whether it is raised.

## Limelight and verified inventory

- Configure `limelight` (fallback `limelight 3A`), AprilTag pipeline 0 and **yellow-only** color pipeline 1. SDK color results do not include color labels, so a combined yellow/blue pipeline is unsuitable. The tilt servo is optional, but the camera must physically see the required targets.
- Team Two needs rearward Hive visibility at its wall shot and forward floor visibility during collection. A fixed camera looking into the wall cannot satisfy this. Calibrate camera aiming/extrinsics and the `TEAM_TWO_TAG_HEIGHT_AXIS`, origin-height and upper/lower-height constants. Set `TEAM_TWO_HIVE_HEIGHT_CALIBRATED=true` only after measurement; its default false deliberately disables Team Two shooting.
- Team Two requires at least two distinct raised far-Cell tags, three fresh frames spanning 100 ms, and arrival within one inch/two degrees of its wall shot. Permission is rebuilt after each arrival. Lost/stale/wrong/uncalibrated evidence pauses the shooter and feeder; there is no any-tag/time-based fallback.
- Both normal roles read the optional four-slot inventory configuration. Set four `TEAM_TWO_POLLEN_SLOT_SENSOR_NAMES`; each named device must support both normalized color and distance. Calibrate presence/color thresholds and enable `TEAM_TWO_INVENTORY_CALIBRATED` only after testing. These existing setting names are shared by both roles.
- Missing hardware, invalid/out-of-range readings or uncalibrated inventory remains UNKNOWN (-1). Empty-slot sensing must provide reliable finite distance readings. Team Two admits a batch only with four total balls AND four verified pollen; subsequent measured count decreases do not require four balls to remain. Unknown/mixed inventory blocks shooting. No count is invented from intake/feed time or disappearing camera targets.
- Team One still performs its timed preload attempts with unknown inventory. A known full load ends Garden pickup; otherwise camera validity, search bounds and the parking reserve bound the attempt. Stale/missing color frames turn intake/translation off. The optional old spill-recovery test has separate Team One height/observation-turn calibration and is disabled in normal mode.

## Spindexer and TeleOp

`FeedCadence` applies a 0.18-second feed pulse followed by at least 0.5 seconds OFF in QuantumStorm TeleOp, normal autonomous Shooter and the generic FTC adapter. Held controls and rapid taps cannot skip the gap; TeleOp reverse unjamming stays immediate. The autonomous feed window is extended from two to three seconds for paced preloads. No sleep blocks steering, sensing, parking or STOP.

This is **open-loop timing**, not confirmed-shot or angle feedback. Calibrate `SPINDEXER_FEED_PULSE_SEC` to advance exactly one pocket on the physical mechanism. A pulse that advances two pockets cannot be fixed by its subsequent pause. The configured CR servo cannot guarantee a 90-degree maximum without suitable position feedback or different hardware; no such guarantee is implemented.

TeleOp joystick Y is inverted to correct the reported forward/backward reversal. Normal QuantumStorm autonomous now applies the same physical correction with AUTO_FORWARD_POWER_SIGN=-1 at the hardware wheel-output boundary. Only forward/backward is corrected; strafing, turning, field coordinates, measured odometry and motor direction configuration are unchanged. Keep the wall-facing start heading: navigation first backs straight approximately six inches into a collision-checked corridor, holding that heading with no sideways command, before routing/turning toward pickup. The same wall-departure clearance is used when leaving a wall shot to park. An already-reached wall shooting target is not backed away from before verification. Do not flip the initial heading 180 degrees to compensate for a motor-power sign error. The simulator retains its ideal forward-positive mixer. Verify the hardware sign with wheels raised, then at low power; this assumes the robot still matches the earlier TeleOp observation. Current local shooter settings are +0.50 (previous master +0.55); verify actual launch direction, power and hood angle before running. Pinpoint offsets remain -170 / -150 mm; verify wiring, configuration, pod directions and odometry axes before field use.

## Verification and build

Local physics checks of all four roles collected four Garden pollen for Team One and four Flower pollen plus eight total shots for Team Two. Modeled parking completed around 26.5 and 28.2 seconds respectively. These tests use a calibrated synthetic field/camera and a simulated shooting power; they are not physical robot validation or proof that real hardware defaults are calibrated. No simulator files are required by or included with this robot-only update.

Place the robot at the INIT-displayed start and keep it still during Pinpoint calibration. Normal modes initialize live shooter/feed/intake hardware: clear the robot and firing path before START. Separate Red/Blue shared diagnostic entries use `ftc/FtcHardwareConfig.java`, which defaults to `calibrated=false` and needs independent setup.

Build from the repository root with the configured Android SDK/JDK:

```sh
./gradlew :TeamCode:compileDebugJavaWithJavac
```
