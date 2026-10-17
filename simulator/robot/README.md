# Shared Java robot control

## Simulator partner roles (current webpage)

The four main webpage modes now select separate simulator roles via `teamRole`:

- **Red/Blue Team One Auto:** start in front of that alliance's Hive, shoot four preloaded pollen, clear the nearby Flower, then park immediately. No Garden/Flower collection.
- **Red/Blue Team Two Auto:** start at browser X/Z `(-62.9,-6)` / `(62.9,6)`, halfway along the wall between its Loading Zone and side Flower, inset to fit the chassis. Its partner's first tilt is a stated solo-simulation initial condition: four physical partner pollen are placed into the initial raised Cell and physics advances until the first tilt settles. These are not counted as this robot's shots. The robot shoots four preloads at the far face, collects the Garden's four pollen, shoots those four at the far face, allows 3.5 seconds of bounded flight/settling time, then parks. This timer is not sensor confirmation; regression checks require a real second tilt before parking starts.

Each Garden contains exactly **four pollen**, without stacked extras. Each alliance’s five unused nectar are visible, inactive stock outside the fence in its matching alliance area, not in the Loading Zone or eligible for pickup, vision targets or scoring. The three nectar already preloaded into each Hive stay there. Base practice stock is 28 pollen and 16 nectar; Team Two setup adds its partner's four scored pollen explicitly. This is the requested practice setup, not a claim about official full-field stock. Both roles retain manual stop, collision checks, a 26-second parking fallback and a 30-second hard stop. The four legacy Single Tilt selections and direct legacy Full-controller tests remain available separately.

These changes are **simulator selections only**: uploaded FTC OpMode wrappers and autonomous-only PR #10 are unchanged. The webpage's Java filename identifies the corresponding robot entry, not a promise that its uploaded hardware mode uses this simulator role. All routine decisions live in shared Java (`TEAM_ONE` / `TEAM_TWO`); JavaScript only selects the role, initializes the scene and supplies physics/transport. Test with `node checks/team-role-check.mjs` from `simulator/`.

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

Choose a mode in **Autonomous mode / Java file**, then click **Start autonomous**. The dropdown has four partner-role routines and four legacy Single Tilt routines, with the mapped OpMode filename shown underneath. Changing selection resets the field to that alliance start; starting also gives a fresh field/four preloads while preserving shooter settings. Team One/Two use their separate role starts described above. Main selections use simulator role overrides of the shared controller; legacy selections retain the older configuration. The browser does not execute FTC hardware wrapper files or arbitrary Java files. The menu is locked during a run/timed match; stop or reset before switching. Blue choices switch aiming, robot label, scoring and opponent-nectar checks to blue. The optional synthetic vision preview is not required.

Java uses simulation time in the browser and a monotonic clock on the robot. The browser advances physics in 20 ms steps only after the matching Java response. Rendering is independent; a slow browser can run simulation slower than real time. Pending responses, dropped connections, manual input, page hiding and Escape cannot leave an old drive command running. Independent browser runs use fresh Java processes.

## FTC repository integration

### Current hardware mode: movement-only rehearsal

All eight QuantumStorm autonomous entries now run the shared controller with movement-only enabled. The SDK BIOBUZZ adapter also defaults `FtcHardwareConfig.movementOnly=true` (with `useVision=false`). The browser stays in normal scoring mode. Robot outputs suppress shooter, feeder, intake and camera-pipeline commands; both hardware adapters independently block scoring mechanisms as a second safeguard. Existing mechanism devices are still initialized and stopped, so their configured hardware names must remain valid. TeleOp is unchanged.

The robot retains the simulator's pauses instead of skipping the shooting/pickup states: nominal one-second spin-up plus two-second feed dwell, 0.14-second Garden dwell and 2.34-second Flower release/sweep dwell. These are dry-run schedule constants, not invented ball counts. Full/unknown inventory cannot skip the route or prolong pickup in movement-only mode. Chassis movement during the Flower sweep and optional leftover sweep remains; no real collection or scoring is expected. Full routines still start parking at 26 seconds; Single Tilt rehearses opening pause → Garden → park. STOP, odometry validation, stalled-motion checks and the 30-second hard deadline remain active.

This matches nominal simulator sequencing and dwell timing, not a guarantee of identical physical arrival times. Real travel time depends on motor speed, odometry, starting pose and field calibration. Do not replace feedback with blind timed driving to force the schedule. The desktop test protocol accepts `movementOnly:true` alongside `cameraFree:true` for regression checks; normal webpage runs omit it.

In movement-only mode, a transient invalid/stale odometry sample or sensor-read exception enters `SAFETY_HOLD`: the loop and match timer continue, but every actuator stays OFF. Valid odometry resumes the interrupted state with its remaining pretend-shoot/pickup dwell preserved. Parking state takes priority at 26 seconds even during a hold, but driving to park still requires valid odometry. At 30 seconds or manual STOP, the loop ends. A blocked fixed corridor gets one collision-checked, resumable replanning attempt per leg; it never blindly drives through an obstacle. Unrecoverable route faults may hold safely until the parking trigger. Clock faults, initialization/output errors and blocking hardware calls are not bypassed. Driver Station `State` and `Last error` show these separately from the routine description.

This folder is bundled inside the FTC repository at `simulator/`. `TeamCode/build.gradle` registers `robot/shared/src/main/java` and `robot/ftc/src/main/java` directly in its Android `sourceSets`. No separate checkout, machine-specific source link, or Java copy is needed. Open the whole repository root in Android Studio and sync Gradle after pulling changes. Opening only TeamCode, copying only AutoStateMachine.java, or an unsuccessful sync can leave `biobuzz` imports unresolved. Missing required shared files now produce a clear Gradle error rather than a cascade of missing symbols. The standalone `shared-sources.gradle` helper remains available for separate simulator checkouts.

Android Studio and the desktop runner compile the same shared sources. Build/deploy the FTC project normally. **Red/Blue Team One/Two Auto** now wrap the shared controller in camera-free mode with the existing QuantumStorm hardware and simulator alliance start. **BIOBUZZ Shared Java Red/Blue** use the configurable FTC adapter; its calibration gate remains in place.

## Camera-free routine

### Additional Single Tilt OpModes

Four additional Driver Station entries are available: **Red Team One Single Tilt**, **Red Team Two Single Tilt**, **Blue Team One Single Tilt**, and **Blue Team Two Single Tilt**. Each has its own annotated Java OpMode, with shared lifecycle/controller logic. They assume the starting Hive cell is UP, run only the opening timed four-preload shot attempt, collect Garden pollen with a bounded intake attempt, then park immediately after pickup rather than waiting until 26 seconds. They do not return for another shot cycle or wait for a camera-confirmed tilt; collected pollen stays loaded. The alliance partner is expected to handle the second tilt. Existing fixed-route starts/calibration, no-Limelight operation, empty-indexer shot attempts, STOP checks and 30-second hard stop still apply. A tilt/four actual shots cannot be guaranteed without balls and calibrated shooting. The desktop test protocol accepts `singleTilt:true` with `cameraFree:true`; physical tests cover all four entries with loaded and empty indexers.

After the full three-shot routine, spare time may be used for one short sweep of the known Flower pollen outlet. Admission requires free inventory space (or unknown inventory), no route fault, collision-clear approach/sweep/park corridors and a conservative travel-time budget. This is a fixed search for leftovers, not camera-based detection or a guaranteed pickup. The extra attempt ends by 25 seconds or measured full inventory; the global 26-second park trigger interrupts all collection and the 30-second hard stop remains. No extra shooting cycle is added. Late routines and Single Tilt skip this optional sweep.

### STOP responsiveness

The controller checks STOP before/after sensor reads and before sending outputs. Camera-free mode follows a small set of alliance-mirrored simulator corridors and turn-clearance points, checking their segments against field geometry without runtime grid/A* search or a planning clock. Red/Blue Team One/Two telemetry explicitly shows `Fixed simulator route; no route search`. Vision-enabled mode retains resumable dynamic planning with a 20 ms/4096-work-unit slice budget. The global 26/30-second deadlines always win. Both robot adapters validate raw Pinpoint readings, shutdown avoids duplicate stop batches, and INIT reports/logs the hardware setup stage. SDK stuck detection and failsafe remain enabled: blocked I2C/USB calls still require hardware/log investigation. No SDK watchdog timeout is increased and no warning is hidden.

Red/Blue Team One/Two do not gate the sequence on AprilTags or indexer occupancy. Every camera-free shot stage spins up and runs the complete `timedFeedSeconds`/`AUTO_FEED_TIME_SEC` feed cycle, even with a measured zero, unknown inventory, or an unchanged full count. Counts are never overwritten and actual shots are never inferred from time. Camera-free Garden arrival accepts up to three inches and eight degrees of approach error; partial pickups advance on a bounded timer. Shooting and Flower contact retain precise navigation targets. Movement timeouts use three seconds without measured progress. Odometry faults, blocked paths, manual STOP and the 26/30-second guards remain enforced; slower hardware can still run out of scoring time. Fixed-mode feeding is a bounded timed attempt, not a count-based jam detector; vision-enabled mode retains its measured-count jam checks.

Real-robot autonomy defaults to: fixed-pose preload shot → Garden → scripted far-side shot → Flower → scripted far-side shot → stationary wait → park starting at 26 seconds. All outputs stop at 30 seconds even if parking is incomplete. Navigation, waypoints, starts and obstacles come from the simulator's shared `FieldLayout`. Team One/Two now share their alliance's simulator start: Red X=-12.75, Y=63.945 inches, heading=-90 degrees; Blue X=12.75, Y=-63.945, heading=90 degrees. INIT telemetry displays it. These are NOT the old side-wall starts. Place the robot there and calibrate odometry, actual footprint/clearance, trajectories and shooter power before testing. The simulated chassis is asymmetric; larger chassis require measured inset poses. Existing hardware shooter power is preserved, not newly calibrated.

Without vision, the first batch is assumed to change the Hive to its far face. The controller cannot confirm/react to empty or missed shots, a different initial Hive orientation or another robot tipping it. QuantumStorm reports inventory unknown (`-1`): feeding uses `AUTO_FEED_TIME_SEC`, Garden intake runs on approach and up to one second at arrival, and Flower release/sweep uses `AUTO_PICKUP_TIMEOUT_SEC`. These are timed attempts, not confirmed collections/shots. Configured ball sensors can complete pickup early but do not skip or shorten camera-free feeding. Valid/fresh Pinpoint, collision routing, manual STOP and hard deadlines remain required.

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

The browser synthesizes Pinpoint pose and measured inventory. Its virtual Limelight has a robot-mounted origin, up/down camera pitch, finite field of view/range, tag-facing checks, conservative support/flower occlusion, 30 FPS, 60 ms delay and a pipeline-switch delay. Optional seeded angle noise and dropped detections are available in `SimSensors` constructor options. The UI camera checkbox tests missing detections.

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
