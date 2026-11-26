# Standalone BIOBUZZ robot control

The default **Biobuzz-Quantum Storm Auto Power Control-latest 11/25** combines distance-based RPM targeting, FTCLib PIDF feedback, battery compensation and rear-hive aiming. The default-location multiplier is 0.495 (2970 RPM); Back+LB/LT adds relative +/-0.005 trim. **Biobuzz-Quantum Storm Tele-Op-old** preserves the previous controller. See [distance calibration and controls](AUTO_SHOOTER_CALIBRATION.md) and [PIDF, field setup and aiming](SHOOTER_PIDF_TEST.md).

The robot app builds entirely from TeamCode. SDK-free controllers, field geometry, navigation and sensor/output contracts are in `src/main/java/org/firstinspires/ftc/teamcode/biobuzz/`; the `ftc/` subpackage contains SDK adapters. Only **Biobuzz-Team 1 Auto** and **Biobuzz-Team 2 Auto** are enabled under `quantumstorm/Auto/`; the default two-driver TeleOp is **Biobuzz-Quantum Storm Auto Power Control-latest 11/25** (`Constants.TELEOP_NAME`, which both autos preselect). No simulator directory, browser or laptop connection is required to build or run the robot app.

## Two modes, no alliance setup

Select **Biobuzz-Team 1 Auto** or **Biobuzz-Team 2 Auto**, place the robot at the matching physical role start on either alliance, press INIT, keep it still for odometry setup, then START. There is no gamepad input, color picker or confirmation step. The legacy Red/Blue autonomous entries have been removed; remaining diagnostics are `@Disabled`.

Both halves use one canonical route frame anchored by the role start, not absolute field-color coordinates. Rotating the physical setup 180 degrees produces the same body-relative driving commands, Garden/Flower pickup, wall clearance and parking half. Do not point the robot the opposite way to compensate: preserve the existing role's physical front/rear orientation. The numeric local route reference shown on Blue is not the absolute Blue field pose.

Team Two recognizes a single visible Hive tag family automatically at its wall shooting pose. It must see at least two distinct raised far-Cell tags with calibrated heights, centered within 10 degrees of the camera axis at the shot pose. Off-axis tags from the adjacent Hive do not identify the target. If both families have centered raised pairs, the view is ambiguous and it does not lock/shoot. The rearward camera aim must be calibrated to center the intended Hive. Once locked for the run, the other family is ignored; Blue tag IDs are aliased to the same canonical role geometry without changing measured angles, heights or frame timestamps. Existing fresh/stable tag, four-pollen, odometry and parking checks remain enforced. This is not a general guarantee of Hive identity: correct physical start and camera aiming are still required. No calibration flag is enabled automatically.

The simulator may retain Red/Blue test cases to verify symmetry; they are not additional enabled robot OpModes.

## Current autonomous roles

- **Team One:** align to a fresh AprilTag before four preload feed attempts, drive to the alliance Garden, collect yellow pollen using live Limelight targets, retain the collected load and park. Normal Garden pickup does not wait for Hive-tag confirmation. The older spilled-pollen recovery path remains an explicit opt-in controller test, not the normal routine.
- **Team Two:** drive from its side-wall start to the opposite-wall shooting pose, verify four type-confirmed pollen and fresh raised own-Hive far-Cell tags, shoot, trigger the nearest end-wall Flower, clear its outlet and collect released pollen with Limelight. Return to the same wall and shoot the second batch only after measuring four pollen and rechecking the tags. No Garden pickup is used by this role.
- **Safety:** parking overrides pickup/feeding by 26 seconds, or earlier when return-time reserve requires. All outputs stop at 30 seconds. Parking accepts a two-inch position tolerance, preserves arrival heading and latches outputs OFF without spinning to correct the final heading. Odometry errors, manual STOP and interruption still stop safely; physical arrival cannot be guaranteed when obstructed or miscalibrated.

The Flower trigger is deliberate contact: at its clearance pose, Team Two applies 12% strafe for no more than 0.4 seconds, then clears the post and approaches the rolling pollen from the open lane. Calibrate this contact and field geometry on the real robot before use.

## Corrected field setup

## TeleOp shooter controls

Select **Biobuzz-Quantum Storm Auto Power Control-latest 11/25**. The shooter target follows distance to the hive in the robot's quadrant, using the actual rear ball-exit position. At the default start `(-12.75, -63.25, -90 degrees)`, the horizontal shooter-to-hive distance is **51.9 inches**, and the base target is **0.495 × 6000 = 2970 RPM**. The previous **Biobuzz-Quantum Storm Tele-Op-old** retains its table/SDK velocity controller. The redundant Auto-Shooter-Power entry is removed.

The latest entry reads encoder RPM, calculates FTCLib PIDF output (including bounded integral correction), multiplies by **12 / battery voltage**, and sends bounded `setPower()` in `RUN_WITHOUT_ENCODER`. Encoder readings remain active. The motor is goBILDA **5203-2402-0001, 1:1, 6000 RPM, 28 counts/revolution**. Keep its power and encoder on **Control Hub port 1**, configured as `shooter`. The shooter-specific motor profile corrects the SDK generic gearbox scale; other motors are unchanged.

Gamepad 2 **A** starts and **B** stops. **Y** aims the rear shooter at the closest quadrant hive; gamepad 1 drive sticks or gamepad 2 B/START cancel aiming. Hold **Back**, then tap **LB/LT** to add/subtract **0.005 (30 RPM)** of persistent trim on top of the automatic target. This remains a relative RPM correction while moving; PIDF and battery compensation continue controlling motor power. At the start, one LB changes 0.495 to 0.500. Trim resets on a new INIT, persists across stops/restarts, and never starts a stopped shooter. Final RPM is bounded to 0–6000.

Each fresh shoulder press applies one step; holding does not repeat. Back pressed after an already-held shoulder does not adjust. LB+LT cancels and START suppresses adjustments. A and shoulder buttons held through INIT/START must be released before a new command. Release START before A/B shooter controls; controller assignment cannot start the shooter.

Telemetry separates distance-based RPM/multiplier, driver trim, final target, actual RPM, PIDF power and battery compensation. Invalid pose or controller data stops shooter/feed and requires a fresh A after recovery. Existing intake, flower and spindexer controls remain, including the flower-modifier release guard. Feeding pauses during aiming and requires a running shooter and valid target/controller; there is no measured-at-speed interlock.

Autonomous shooting still uses its existing SDK velocity controller and shared table; only its TeleOp preselection changes. The latest TeleOp's 2970-RPM reference bias and relative trim do not alter autonomous targets. See [distance calibration](AUTO_SHOOTER_CALIBRATION.md) and [PIDF/aiming details](SHOOTER_PIDF_TEST.md) for initialization, formulas and gain settings.

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

- Configure `limelight` (fallback `limelight 3A`): AprilTags in slot **1**, yellow-only pickup in slot **0**, blue in slot **2**, and red in slot **3**. No green pipeline is used. SDK color results do not include color labels, so each slot must detect its named color only.
- `LimeServo` commands neutral at servo position **0.5** for AprilTags/shooting. Color detection commands **120 degrees clockwise** from neutral (position **0.9** with the configured 300-degree travel). This is a commanded angle, not measured position feedback; verify mounting, direction and clearance on hardware.
- Autonomous stationary idle states scan **red then blue**, switching every **3 seconds**, without steering. Active Garden forward-intake and Flower intake strokes cycle **yellow -> blue -> red** every 3 seconds at the +120-degree camera position. Yellow is collected; `collectBlue=true` collects blue, and `collectRed=false` stops forward movement/intake and turns away from visible red. These controller settings do not identify an alliance or change type-verified pollen counts. Color steering requires fresh observations and valid odometry, is capped at 20% power toward desired colors or 15% away, and does not run during transit or backing/repositioning. One color slot is visible at a time: this is sampled avoidance, not continuous red-obstacle detection or a guarantee of collision avoidance.
- **Normal Team One override (not the backup):** Garden forward pickup stays on yellow slot **0** at +120 degrees, rather than cycling colors. During **10 through 18 seconds after START**, a fresh yellow observation keeps `Leave authorized=false` and maintains forward collection without back/right retry strokes. A fresh frame reporting no yellow immediately latches `Leave authorized=true`, stops collection and enters the regular stationary wait before the existing 26-second parking route. Missing, disconnected, stale (over 250 ms), or switching camera frames do not prove absence: motion/intake pause until a fresh observation returns. Outside this window, the existing Garden pass/full-inventory logic applies. Forward/strafe power requests are multiplied by **1.25**, bounded to [-1, 1]; turning, shooter, feeder and intake power are unchanged. This is a motor-command increase, not a guarantee of 1.25x measured speed; verify travel and clearance on hardware.
- Shooting and tag verification return to the same **INIT-commanded neutral position (0.5)**, not an unknown unpowered physical position. Parking overrides color scanning/steering and intake, commands the camera to stay **+120 degrees (0.9)** with the yellow pipeline, and keeps the existing route, 26-second park command and 30-second hard stop. Servo positions remain open-loop commands, not measured camera-angle feedback.
- Camera telemetry is limited to `tv`, `tx`, `ty`, shooting authorized (YES/NO), AprilTags seen, colors seen, turn suggestion, and commanded servo NEUTRAL/NOT NEUTRAL. Shooting authorization reflects the controller commanding shooter or feeder power, including spin-up. During shooting alignment, `tx` and the turn suggestion use the selected Hive cluster's center, not the Limelight aggregate target. Otherwise they use the camera's aggregate `tx`; suggestions alone never command movement.
- During shooting alignment only, Team One uses its four near-Cell IDs; Team Two and the Team One backup follow-up use the four far-Cell IDs. The center is the average image-plane horizontal position of the distinct visible tags in that group, converted back to a `tx` angle. At least two distinct expected tags are required to aim or shoot; fewer tags, partial visibility that misses this minimum, or two visible families in robot-relative modes blocks alignment. Two or more visible tags steer toward their visible center; each fresh frame recomputes it. This does not estimate unseen tag locations or prove the full four-tag center. Wrong-Cell IDs are ignored, and missing/duplicate/nonfinite expected-tag evidence cannot fall back to aggregate `tx`.
- Both enabled autonomous roles and the Team One backup require fresh valid AprilTag `tv`, at least two distinct expected tags, and cluster-center `tx` within **2 degrees** for three distinct frames spanning at least **100 ms** before shooting. Off-center `tx` commands an in-place clockwise-positive mecanum turn capped at **25% power**, with valid fresh odometry required for turning. The initial proportional gain is **0.02 power per degree**; verify its response and the motor/camera turn sign at low power, then tune `ShootingAim`'s named constants on the robot rather than assuming these values fit the hardware. No automatic Hive-cluster turning runs during pickup, transit or idle scans. A lost, stale, wrong-pipeline or nonfinite target stops shooter/feed immediately; alignment restarts the spin-up delay. Existing Hive, inventory, route and STOP/parking checks remain in force. Camera frames must be no older than 250 ms, and pipeline transitions discard results for 200 ms. Camera neutral must face the intended Hive/shooter direction.
- The hardware adapter sends this turn through the same robot-centric mecanum mixer as TeleOp. Forward-power calibration does not invert the aiming turn. Limelight results with `tv=0` cannot authorize turning or feeding even if they contain tag entries.
- Team Two needs rearward Hive visibility at its wall shot and forward floor visibility during collection. A fixed camera looking into the wall cannot satisfy this. Calibrate camera aiming/extrinsics and the `TEAM_TWO_TAG_HEIGHT_AXIS`, origin-height and upper/lower-height constants. Set `TEAM_TWO_HIVE_HEIGHT_CALIBRATED=true` only after measurement; its default false deliberately disables Team Two shooting.
- Team Two requires at least two distinct raised far-Cell tags, three fresh frames spanning 100 ms, and arrival within one inch/two degrees of its wall shot. Permission is rebuilt after each arrival. Lost/stale/wrong/uncalibrated evidence pauses the shooter and feeder; there is no any-tag/time-based fallback.
- Both normal roles read the optional four-slot inventory configuration. Set four `TEAM_TWO_POLLEN_SLOT_SENSOR_NAMES`; each named device must support both normalized color and distance. Calibrate presence/color thresholds and enable `TEAM_TWO_INVENTORY_CALIBRATED` only after testing. These existing setting names are shared by both roles.
- Missing hardware, invalid/out-of-range readings or uncalibrated inventory remains UNKNOWN (-1). Empty-slot sensing must provide reliable finite distance readings. Team Two admits a batch only with four total balls AND four verified pollen; subsequent measured count decreases do not require four balls to remain. Unknown/mixed inventory blocks shooting. No count is invented from intake/feed time or disappearing camera targets.
- Team One still performs its timed preload attempts with unknown inventory. A known full load ends Garden pickup; otherwise camera validity, search bounds and the parking reserve bound the attempt. Stale/missing color frames turn intake/translation off. The optional old spill-recovery test has separate Team One height/observation-turn calibration and is disabled in normal mode.

## Spindexer and TeleOp

Select **Biobuzz-Quantum Storm Auto Power Control-latest 11/25** for the controls below.

| Gamepad 1 control | Assignment |
| --- | --- |
| Left stick up/down | Forward/backward, with the robot's corrected longitudinal mapping |
| Left stick left/right | Strafe left/right |
| Right stick left/right | Turn left/right |
| Tap A without Back or Start | Intake forward ON; stays on until another command |
| B | Intake OFF; overrides A and all modifier combinations |
| Hold Back, then tap A, with Start released | Intake reverse ON; stays on after releasing the buttons |
| Start + A | Driver Station gamepad 1 assignment; no intake direction command |
| Tap X | Flower arm toggle: down if up, up if down |

A is no longer a toggle. Tapping A alone again leaves forward intake on. Hold Back before pressing A to reverse; changing Back while A remains held does not change direction. Start suppresses all A direction commands, even with Back held, and releasing Start while A remains held does not activate the intake. Release A before issuing a new direction command. B always stops the intake, including while Start or Back is held. Assign the controller before starting TeleOp, then release A. An A held across the TeleOp start is ignored until released and pressed again. On the Logitech F310, use the Back button: Mode changes stick/D-pad behavior internally and is not available as a normal FTC button.

| Gamepad 2 control | Assignment |
| --- | --- |
| Tap A, with Start released | Shooter ON |
| B, with Start released | Shooter OFF; overrides A |
| Hold Back, then tap LB | Add +0.005 (+30 RPM) relative trim to automatic target |
| Hold Back, then tap LT past halfway | Add -0.005 (-30 RPM) relative trim to automatic target |
| Tap Y | Aim rear shooter at closest quadrant hive |
| Hold D-pad Up | Spindexer forward, using timed pulses |
| Hold D-pad Down | Spindexer reverse continuously for unjamming |
| Release D-pad Up/Down | Spindexer STOP |
| Back + D-pad Up / Down | Flower arm up / down |
| LB + D-pad Up / Down | Nudge flower arm +2° / −2° |

Flower-arm combinations reserve the D-pad until **both Up and Down are released**. Releasing Back or LB first does not run the spindexer. Release the D-pad, then press it again without either modifier to feed or reverse.

Gamepad 2 A/B are ignored while Start is held for controller assignment. Trim changes once per new shoulder press, with Back held and Start released. LB and LT together cancel. Default-location multiplier is 0.495 (2970 RPM), with relative trim initially zero and a final target range of 0–6000 RPM. Adjustment does not start a stopped shooter. Shooter 2 remains removed. The F310 Mode button changes controller behavior internally.

`FeedCadence` applies a 0.18-second spindexer pulse followed by at least 0.5 seconds OFF in QuantumStorm TeleOp, normal autonomous Shooter and the generic FTC adapter. Held controls and rapid taps cannot skip the gap; TeleOp reverse unjamming stays immediate. The autonomous feed window is extended from two to three seconds for paced preloads. No sleep blocks steering, sensing, parking or STOP.

This is **open-loop timing**, not confirmed-shot or angle feedback. Calibrate `SPINDEXER_FEED_PULSE_SEC` to advance exactly one pocket on the physical mechanism. A pulse that advances two pockets cannot be fixed by its subsequent pause. The configured CR servo cannot guarantee a 90-degree maximum without suitable position feedback or different hardware; no such guarantee is implemented.

TeleOp joystick Y is inverted to correct the reported forward/backward reversal. Normal QuantumStorm autonomous now applies the same physical correction with AUTO_FORWARD_POWER_SIGN=-1 at the hardware wheel-output boundary. Only forward/backward is corrected; strafing, turning, field coordinates, measured odometry and motor direction configuration are unchanged. Keep the wall-facing start heading: navigation first backs straight approximately six inches into a collision-checked corridor, holding that heading with no sideways command, before routing/turning toward pickup. The same wall-departure clearance is used when leaving a wall shot to park. An already-reached wall shooting target is not backed away from before verification. Do not flip the initial heading 180 degrees to compensate for a motor-power sign error. The simulator retains its ideal forward-positive mixer. Verify the hardware sign with wheels raised, then at low power; this assumes the robot still matches the earlier TeleOp observation. The latest TeleOp targets 2970 RPM at its default start with zero trim; autonomous retains its own calibration. Verify launch direction, RPM and hood angle on hardware. Pinpoint offsets are -152.4 / -196.85 mm: the forward pod is 6 inches right of center, and the sideways pod is 7.75 inches behind center (1 inch from the rear edge of the 17.5-inch robot). The sideways pod's 4-inch left displacement is not the Y offset. Verify wheel contact-point measurements and repeat a stationary-center turn after re-INIT; see [odometry calibration details](SHOOTER_PIDF_TEST.md#field-setup-and-rear-hive-aiming).

## Verification and build

Local physics checks of all four roles collected four Garden pollen for Team One and four Flower pollen plus eight total shots for Team Two. Modeled parking completed around 26.5 and 28.2 seconds respectively. These tests use a calibrated synthetic field/camera and a simulated shooting power; they are not physical robot validation or proof that real hardware defaults are calibrated. No simulator files are required by or included with this robot-only update.

Place the robot at the INIT-displayed start and keep it still during Pinpoint calibration. Normal modes initialize live shooter/feed/intake hardware: clear the robot and firing path before START. Separate Red/Blue shared diagnostic entries use `ftc/FtcHardwareConfig.java`, which defaults to `calibrated=false` and needs independent setup.

Build from the repository root with the configured Android SDK/JDK:

```sh
./gradlew :TeamCode:compileDebugJavaWithJavac
```
