# Shared Java robot control

## Limelight pollen and nectar experiment (simulator only)

Select **Limelight Pollen + Nectar Test** in the webpage's Autonomous mode menu. Selecting it enables synthetic vision; turn that checkbox off to test lost detections. This fifth entry runs `shared/.../LimelightRecoveryAuto.java`, not an FTC hardware OpMode. The four uploaded robot entries and their 26-second parking wait remain unchanged.

The experiment requires four measured preloads and a fresh, stable view of the alliance's raised Hive tags before feeding. It shoots the four preloads, moves 18 inches off the fence for a clear tag view with intake OFF, and requires fresh multi-frame tag-height evidence that the opposite Cell is up. A timer, four shots, empty inventory, or opponent tags do not confirm a tilt. It then backs another eight inches inward for camera clearance, without a forced half-turn, and changes from tag pipeline 0 to pollen/alliance-nectar color pipeline 1.

After collecting a batch, the robot makes room to rotate and surveys the full camera view, preferring the other eligible color before returning to its best observed bearing. These clearance moves do not specify ball pickup locations. Both alliance physics regressions recover three actual Hive-spilled balls (two alliance nectar and one pollen) and finish parking before 30 seconds; a fourth pickup is not guaranteed.

Color recovery selects the largest visible yellow pollen or alliance-colored nectar target by image-area percentage, ignoring opponent-colored nectar and steers from its horizontal angle, adapting the selection/strafe strategy in the imported `PollenAndAprilTagAuto.java`. Forward cruise now uses 0.70 power for small/far targets, 0.50 at mid-range and 0.25 near the target, versus the previous 0.45/0.28/0.12. Steering and search are faster too; collision, freshness and deadline checks remain active. The controller gets only camera angles, apparent area, color, timestamps and measured robot pose/inventory: no ball world coordinates, IDs identifying a particular physics ball, or Hive/Garden/Flower provenance. The collision map can authorize a short side clearance around a frame leg while a target remains visible. There are no fixed Garden/Flower pickup destinations. Each measured pickup resumes the search instead of ending it. Collection continues until measured inventory reaches four balls or the return-travel budget requires parking. Recovered pollen and nectar stay loaded; no second firing cycle is added. Recovery ends by 26 seconds, or earlier when the conservative return-travel budget requires it. The 30-second hard stop remains. Lost/stale targets cancel translation/intake; a disconnected camera cancels all recovery outputs. STOP, invalid odometry and collision guards remain active.

This is a synthetic control/physics test, **not a test of real Limelight image recognition, device wiring, the SDK calls, or direct execution of `V1code.java` / `AprilTagFireGate.java`**. Hive tilt uses shared `HiveVision` height confirmation, not V1's camera-orientation calibration or its any-tag-after-10-seconds fallback. The virtual Hive camera faces the rear shooter and the ground camera faces the front intake; a real build needs an equivalent calibrated camera arrangement before using this behavior. Existing Vision files are preserved. Their previously reported `a.java` class-name conflict and missing `LIMELIGHT_COLOR_PIPELINE` still need resolving before an Android upload.

Run `node checks/limelight-recovery-check.mjs` from `simulator/`. It verifies native decision/safety fixtures and both alliances in Cannon, including actual Hive-spilled pollen AND alliance nectar pickups, multiple measured balls and four-ball capacity (not fabricated counts or Garden/Flower pickups). Other balls are not relabeled or teleported for the demo.

## Simulator partner roles (current webpage)

Team One shoots its four preloads and parks without collecting. After its required shots, simulator Team Two can attempt one side Flower pickup only with free inventory space and a conservative approach/pickup/parking budget. The attempt lasts at most three seconds after arrival, ends by 25 seconds, and never adds another shot. Full/late robots skip it; collision/route failures return to parking. A pickup is not guaranteed: the tests measure actual pollen collected, including partial loads. The extra pollen stays loaded when parked. The removed dedicated short-routine files and test remain recoverable from Git history.

The four main webpage modes now select separate simulator roles via `teamRole`:

- **Red/Blue Team One Auto:** start by the alliance fence with the intake/front facing the fence and the rear-facing shooter pointing toward the Hive, shoot four preloaded pollen, then drive directly to parking without an intermediate Hive waypoint or collection.
- **Red/Blue Team Two Auto:** start at browser X/Z `(-62.9,-6)` / `(62.9,6)`, halfway along the wall between its Loading Zone and side Flower, inset to fit the chassis. Its partner's first tilt is a stated solo-simulation initial condition: four physical partner pollen are placed into the initial raised Cell and physics advances until the first tilt settles. These are not counted as this robot's shots. The robot shoots four preloads at the far face, collects the Garden's four pollen, shoots those four at the far face, allows 3.5 seconds of bounded flight/settling time, then in the simulator tries one nearby Flower pickup if time and capacity allow before parking. The actual robot omits that final pickup. This timer is not sensor confirmation; regression checks require a real second tilt before parking starts.

Each Garden contains exactly **four pollen**, without stacked extras. Each alliance’s five unused nectar are visible, inactive stock outside the fence in its matching alliance area, not in the Loading Zone or eligible for pickup, vision targets or scoring. The three nectar already preloaded into each Hive stay there. Base practice stock is 28 pollen and 16 nectar; Team Two setup adds its partner's four scored pollen explicitly. This is the requested practice setup, not a claim about official full-field stock. Both roles retain manual stop, collision checks, a 26-second parking fallback and a 30-second hard stop. Only the four main team selections remain; direct legacy Full-controller tests remain available.

The four QuantumStorm FTC entries now select these same shared partner roles, with matching role-specific starting poses and live shooter/feed/intake outputs. The webpage supplies simulated sensors; the robot supplies Pinpoint pose and reports inventory unknown. No browser or laptop is needed on the robot.

The browser and the robot compile the same files in `shared/src/main/java`. Change autonomous decisions there once. The browser has no JavaScript autonomous state machine.

| Folder | Responsibility |
|---|---|
| `shared/` | `AutoStateMachine`, collision-aware `Navigator`, `FieldLayout`, and `RobotIO` sensor/actuator contract; Java 8, no FTC SDK or browser imports |
| `desktop/` | Local JVM runner and JSON transport; never deployed to the robot |
| `ftc/` | FTC SDK hardware adapter, hardware calibration, and red/blue OpMode lifecycle; never loaded in the browser |
| `../dist/` | Field rendering, physics, synthetic sensor measurements and simulated actuator behavior |
| `../bridge/` | Local HTTP/WebSocket server and Java compilation/process management |

## Run and edit

Install Node.js and **JDK 17 or newer**. `java -version` should work; point `JAVA_HOME` to your JDK if needed. A JRE without its compiler module is insufficient. No npm packages need installing.

```sh
npm start
```

Open `http://localhost:8000`. Start autonomous with the UI button. Restart the server after changing Java sources; it compiles them on startup. Refresh the browser after changing HTML/JavaScript. A compile error disables autonomous but leaves manual simulation available.

Choose a mode in **Autonomous mode / Java file**, then click **Start autonomous**. The dropdown has four partner-role routines, with the mapped OpMode filename shown underneath. Changing selection resets the field to that alliance start; starting also gives a fresh field/four preloads while preserving shooter settings. Team One/Two use their separate role starts described above. Selections use simulator role overrides of the shared controller. The browser does not execute FTC hardware wrapper files or arbitrary Java files. The menu is locked during a run/timed match; stop or reset before switching. Blue choices switch aiming, robot label, scoring and opponent-nectar checks to blue. The optional synthetic vision preview is not required.

Java uses simulation time in the browser and a monotonic clock on the robot. The browser advances physics in 20 ms steps only after the matching Java response. Rendering is independent; a slow browser can run simulation slower than real time. Pending responses, dropped connections, manual input, page hiding and Escape cannot leave an old drive command running. Independent browser runs use fresh Java processes.

## FTC repository integration

### Current QuantumStorm hardware mode: live simulator roles

The actual robot sets waitUntilParkTime=true: after completing its required routine it holds drive, shooter, feeder and intake OFF, without extra pickup, until match elapsed time reaches 26 seconds. Then it drives directly to the parking target and latches OFF on arrival; the 30-second hard stop remains. This restores timed parking on hardware only; the webpage's role timing is unchanged.

Parking completion is position-only and latched: the QuantumStorm adapter accepts a measured position within 2 inches of the parking target, immediately zeros drive/shooter/feeder/intake, and exits the routine without a final heading correction. Later sensor drift cannot resume movement; an explicit new START resets completion. The simulator retains a 0.65-inch position tolerance. These thresholds do not repair incorrect odometry; physical braking distance still depends on the robot. Existing drive motors retain BRAKE zero-power behavior.

Hardware exception: the four QuantumStorm entries set collectAfterRoutine=false. Team One shoots its preloads and goes directly to parking; Team Two keeps its required Garden pickup and second shot batch, then parks after its settling pause. Neither adds the final optional Flower collection. The webpage still enables optional collection. Parking fallback at 26 seconds, STOP and the 30-second cutoff are unchanged.

Red/Blue Team One select shared Java TEAM_ONE; Red/Blue Team Two select TEAM_TWO. Both run camera-free with movementOnly=false. The required shot sequence, route, parking budget and deadlines come from the same controller as the browser. Team One disables optional collection by default in both environments; robot Team Two also disables its final optional collection. Team Two assumes the partner has already completed the first Hive tilt; no sensor confirms this. It does not wait for a partner signal.

The shooter adapter drives a single motor using the current signed TeleOp calibration (SHOOTER_1_POWER), the feeder CR servo (FORWARD direction, matching TeleOp), and the existing intake motor/servos. Hardware names shooter, shooter_servo and intake devices must match the Control Hub configuration. Mechanisms initialize OFF but run automatically after START. Both TeleOp and autonomous require only the single shooter motor.

No ball sensor is configured: inventory remains -1. Shot attempts run for one-second spin-up plus two-second feeding (spin-up can overlap approach). Garden pickup runs on approach and up to one second at arrival; Flower pickup runs at most three seconds. These are bounded attempts, not invented inventory or confirmed scoring. Pickup timing can therefore differ from the simulator's measured-count completion. The optional attempt ends by 25 seconds, parking takes over by 26, and every actuator stops at 30 seconds or STOP. Invalid odometry stops the live routine safely; it never switches to blind driving.

The separate BIOBUZZ Shared Java Red/Blue diagnostic entries still use FtcHardwareConfig and its calibration gate/movement-only default; choose the QuantumStorm Red/Blue Team One/Two entries for the webpage roles. The legacy FULL movement-only regression remains available in desktop tests.

This folder is bundled inside the FTC repository at `simulator/`. `TeamCode/build.gradle` registers `robot/shared/src/main/java` and `robot/ftc/src/main/java` directly in its Android `sourceSets`. No separate checkout, machine-specific source link, or Java copy is needed. Open the whole repository root in Android Studio and sync Gradle after pulling changes. Opening only TeamCode, copying only AutoStateMachine.java, or an unsuccessful sync can leave `biobuzz` imports unresolved. Missing required shared files now produce a clear Gradle error rather than a cascade of missing symbols. The standalone `shared-sources.gradle` helper remains available for separate simulator checkouts.

Android Studio and the desktop runner compile the same shared sources. Build/deploy the FTC project normally. **Red/Blue Team One/Two Auto** now wrap the shared controller in camera-free mode with the existing QuantumStorm hardware and simulator alliance start. **BIOBUZZ Shared Java Red/Blue** use the configurable FTC adapter; its calibration gate remains in place.

## Camera-free routine

After the full three-shot routine, spare time may be used for one short sweep of the known Flower pollen outlet. Admission requires free inventory space (or unknown inventory), no route fault, collision-clear approach/sweep/park corridors and a conservative travel-time budget. This is a fixed search for leftovers, not camera-based detection or a guaranteed pickup. The extra attempt ends by 25 seconds or measured full inventory; the global 26-second park trigger interrupts all collection and the 30-second hard stop remains. No extra shooting cycle is added. Late or full routines skip this optional sweep.

### STOP responsiveness

The controller checks STOP before/after sensor reads and before sending outputs. Camera-free mode follows a small set of alliance-mirrored simulator corridors and turn-clearance points, checking their segments against field geometry without runtime grid/A* search or a planning clock. Red/Blue Team One/Two telemetry explicitly shows `Fixed simulator route; no route search`. Vision-enabled mode retains resumable dynamic planning with a 20 ms/4096-work-unit slice budget. The global 26/30-second deadlines always win. Both robot adapters validate raw Pinpoint readings, shutdown avoids duplicate stop batches, and INIT reports/logs the hardware setup stage. SDK stuck detection and failsafe remain enabled: blocked I2C/USB calls still require hardware/log investigation. No SDK watchdog timeout is increased and no warning is hidden.

Red/Blue Team One/Two do not gate the sequence on AprilTags or indexer occupancy. Every camera-free shot stage spins up and runs the complete `timedFeedSeconds`/`AUTO_FEED_TIME_SEC` feed cycle, even with a measured zero, unknown inventory, or an unchanged full count. Counts are never overwritten and actual shots are never inferred from time. Camera-free Garden arrival accepts up to three inches and eight degrees of approach error; partial pickups advance on a bounded timer. Shooting and Flower contact retain precise navigation targets. Movement timeouts use three seconds without measured progress. Odometry faults, blocked paths, manual STOP and the 26/30-second guards remain enforced; slower hardware can still run out of scoring time. Fixed-mode feeding is a bounded timed attempt, not a count-based jam detector; vision-enabled mode retains its measured-count jam checks.

Real-robot Team One shoots its preloads backward into the Hive and drives directly to parking without an intermediate Hive waypoint or collection. Team Two shoots preloads at the far face, collects Garden pollen, shoots again, allows settling time, optionally collects nearby Flower pollen and parks. Java start coordinates (inches, heading CCW from +X) are Red One (-12.75, 62.9, 90 degrees), Blue One (12.75, -62.9, -90 degrees), Red Two (-62.9, 6, -90 degrees), Blue Two (62.9, -6, 90 degrees). INIT displays the required pose. Place the robot there and calibrate Pinpoint offsets/directions, actual chassis geometry, drive speed and shooter power. Shared FieldLayout supplies these poses; the old zero-valued Constants poses are not used by these entries. Simulation success does not guarantee real arrival times, aiming, tilts or collection.

Without vision, Team One uses the initial face and Team Two assumes its partner already changed the Hive to its far face. The controller cannot confirm/react to empty or missed shots, a different initial Hive orientation or another robot tipping it. QuantumStorm reports inventory unknown (`-1`): feeding uses shared `timedFeedSeconds` (2 seconds), Garden intake runs on approach and up to one second at arrival, and Flower release/sweep uses shared `timedPickupSeconds` (3 seconds). These are timed attempts, not confirmed collections/shots. Configured ball sensors can complete pickup early but do not skip or shorten camera-free feeding. Valid/fresh Pinpoint, collision routing, manual STOP and hard deadlines remain required.

The webpage defaults to camera-free Red Team One. Direct desktop/client calls without `cameraFree:true` retain legacy vision-enabled diagnostic behavior. Physical regression checks require all 12 full-routine shots, both pickups and parking before 30 seconds, with no planning/detection wait. The fixed route requires the displayed simulator start and calibrated field geometry, chassis footprint and odometry. If a fixed corridor or turn is blocked, it reports the calibration/route fault rather than blindly driving through it. Simulation success is not a guarantee of hardware aiming, timing or parking from every start.

## Configure actual hardware before running

`ftc/.../FtcHardwareConfig.java` deliberately starts with `calibrated=false`. Simulator waypoints and unmeasured pod offsets are not verified hardware calibration. Unknown inventory is handled only as timed attempts, not evidence of a full/empty indexer.

Configure:

- Measured starts, shooting, far-face, garden and flower poses in the shared coordinate frame, and the real robot footprint/field geometry if different from the simulated starter robot. Shared coordinates are inches, heading radians CCW from +X, origin at field center. Browser coordinates convert as `X=x`, `Y=-z`, `heading=theta+pi/2`. Confirm this frame against your Pinpoint setup.
- Actual tag IDs assigned to the near/far cell of each hive. `FieldLayout` defaults and the simulated cluster placement are provisional; verify against the field. Tag angles are degrees, right/clockwise positive. The controller centers the mean of visible tags in the intended cluster; if a single-tag aim requires an offset, add that calibration in the shared controller or Limelight point-of-interest pipeline and test both adapters.
- Motor/servo directions, pod directions and measured offsets, camera up/down servo positions, pipelines and shooting power. Hardware names and directions begin with the QuantumStorm configuration. Drive motors use the same Java-normalized wheel powers in both adapters; the flywheel uses motor power, not guaranteed RPM feedback.

`FtcHardwareConfig.useVision=false` is the hardware default: neither Limelight nor tilt servo is looked up, and camera-height calibration is not required. Set `useVision=true` to restore camera-based aiming and tip detection; then Limelight and calibrated tag heights are required, with an optional tilt servo. A null or empty `cameraTilt` explicitly disables that servo.
- A sensor-backed ball count: configure four occupied-slot digital input names or override `createBallCounter()` for your actual indexer sensors. Empty/full counts are never inferred from time. Camera-free mode with no configured slots reports unknown inventory and uses timed attempts; vision-enabled mode still requires measured counts.

Then set `calibrated=true`. Hardware initialization and Pinpoint calibration happen in INIT while the robot is still. Motors and mechanisms stop in the OpMode's `finally` block. No WebSocket connection, browser or laptop is required when the controller runs on the robot.

## Sensor behavior and limits

### Alliance filtering and Hive-tip detection

`HiveVision` is used by the shared controller and QuantumStorm camera wrapper.
Only the selected alliance's Cell clusters can authorize aiming or feeding:
Red rear 30-33 / audience 34-37; Blue audience 38-41 / rear 42-45
(BIOBUZZ TU03, page 76). "Normal" follows the simulator's initial upward Cell;
it is not an assumption about the physical match setup.

An ID identifies a Cell, not its tipped state. Detection uses measured tag-plane
height above the tiles. Nominal simulator bands are 52.02 inches up and 36.57
inches down, with 1.5-inch tolerance. At least two tags from a cluster and three
distinct fresh frames spanning 100 ms must agree. Height drift over 0.20 inches
during confirmation, conflicting Cell observations, missing 3D pose, stale frames,
or the wrong pipeline produce an unknown state. Feeding pauses when state/aim is
unknown, and a confirmed change moves to the opposite shooting pose in either
direction. A partial remaining load can continue after that reposition.

The simulator supplies tag heights from visible tag geometry, preserving its
camera delay and field of view. No JavaScript autonomous decision was added.
The FTC adapter reads each fiducial's target pose in robot space. Configure the
Limelight camera pose/extrinsics for the actual tilt-up position; verify the
vertical axis and robot-origin height against the floor. Measure the up/down
bands and noise tolerance in `FieldLayout`. Then set
`FtcHardwareConfig.hiveHeightCalibrated=true`; keep `calibrated=false` until all
other hardware checks are complete. QuantumStorm uses the equivalent
`Constants.LIMELIGHT_HIVE_HEIGHT_CALIBRATED`, `LIMELIGHT_TAG_HEIGHT_AXIS`, and
`LIMELIGHT_ROBOT_ORIGIN_HEIGHT_IN` settings. These calibration flags default false
so an unconfigured 3D pose cannot authorize firing. No custom Python pipeline is
required. Both hardware wrappers wait 200 ms after a tilt/pipeline change before
accepting a pose.

`HiveVisionChecks` covers both alliances, repeated/replayed frames, missing pose,
motion and contradictory measurements, and normal-to-flipped-to-normal changes.

The browser synthesizes Pinpoint pose and measured inventory. Its virtual Hive preview faces the rear shooter while the ground-ball preview faces the front intake; this is a synthetic preview model, not a claim about an installed camera. It has a robot-mounted origin, up/down camera pitch, finite field of view/range, tag-facing checks, conservative support/flower occlusion, 30 FPS, 60 ms delay and a pipeline-switch delay. Optional seeded angle noise and dropped detections are available in `SimSensors` constructor options. The UI camera checkbox tests missing detections.

In vision-enabled mode the robot adapter reads actual Pinpoint and Limelight data, checks validity/pipeline and individual tags, and accounts for latency. Java rejects stale pose/vision and ignores the other alliance's tags. Camera-free mode deliberately uses odometry and fixed shooting poses without vision confirmation.

Synthetic results test control behavior. They do not execute Limelight image recognition, reproduce every occluder, estimate raw encoder dynamics, or guarantee real-world trajectory accuracy. Validate camera calibration, wheel speed and trajectories on your robot. Ground-ball tracking currently uses the first result of the configured ball pipeline.

## Protocol and checks

The local browser connects to `/robot` over WebSocket. Messages carry `protocol:1`, a fresh `runId`, incrementing `seq`, and `time` in seconds. `start` supplies alliance/power and sensors; `step` supplies sensors exactly 20 ms later; `stop` clears actuators. Replies contain actuator powers, pipeline and telemetry. One session is allowed at a time; stale/out-of-order messages stop the controller. The server binds to loopback and rejects cross-origin browser connections.

```sh
npm run java:build
npm run check
```

Checks run the real JVM against cannon-es for both alliances, live firing/refill, tag loss, delayed pipelines, protocol/reconnect/manual stop, and the existing game regressions. The physical routine check verifies four Garden pickups, four Flower pickups, all 12 shots in order, a second completed Hive tilt before parking, independent AprilTag confirmation, and zero further pickups. The robot waits stationary after confirmation, begins parking at 26 seconds, and arrives before 30 seconds at both default alliance starts with .395 shooter power and the 66.5-degree hood. Parking preserves the current heading and targets an inset position overlapping the alliance Loading Zone. Faults, changed shooter calibration, blocked routes or a distant starting pose can still prevent completion. An unconditional actuator stop at 30 seconds remains in place.

The shared and FTC adapter sources were additionally compiled against the team's FTC SDK 12.0.0 Hardware and RobotCore artifacts. This is a compile check, not a Control Hub hardware test.
