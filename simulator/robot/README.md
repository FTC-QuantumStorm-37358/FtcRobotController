# Shared Java robot control

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

Java uses simulation time in the browser and a monotonic clock on the robot. The browser advances physics in 20 ms steps only after the matching Java response. Rendering is independent; a slow browser can run simulation slower than real time. Pending responses, dropped connections, manual input, page hiding and Escape cannot leave an old drive command running. Independent browser runs use fresh Java processes.

## FTC repository integration

This folder is bundled inside the FTC repository at `simulator/`. `TeamCode/build.gradle` directly includes `robot/shared/src/main/java` and `robot/ftc/src/main/java` using `shared-sources.gradle`. No separate checkout, machine-specific source link, or Java copy is needed.

Android Studio and the desktop runner compile the same shared sources. Build/deploy the FTC project normally. On the Driver Station select **BIOBUZZ Shared Java Red** or **BIOBUZZ Shared Java Blue**. Existing QuantumStorm OpModes remain separate. The source-link helper is only for users who intentionally maintain a separate simulator checkout.

## Configure actual hardware before running

`ftc/.../FtcHardwareConfig.java` deliberately starts with `calibrated=false`. The original team repository has zero-valued field waypoints, unmeasured pod offsets and a `SoftwareIndexer` that assumes ball counts. These are not sufficient to run the shared controller on hardware.

Configure:

- Measured starts, shooting, far-face, garden and flower poses in the shared coordinate frame, and the real robot footprint/field geometry if different from the simulated starter robot. Shared coordinates are inches, heading radians CCW from +X, origin at field center. Browser coordinates convert as `X=x`, `Y=-z`, `heading=theta+pi/2`. Confirm this frame against your Pinpoint setup.
- Actual tag IDs assigned to the near/far cell of each hive. `FieldLayout` defaults and the simulated cluster placement are provisional; verify against the field. Tag angles are degrees, right/clockwise positive. The controller centers the mean of visible tags in the intended cluster; if a single-tag aim requires an offset, add that calibration in the shared controller or Limelight point-of-interest pipeline and test both adapters.
- Motor/servo directions, pod directions and measured offsets, camera up/down servo positions, pipelines and shooting power. Hardware names and directions begin with the QuantumStorm configuration. Drive motors use the same Java-normalized wheel powers in both adapters; the flywheel uses motor power, not guaranteed RPM feedback.
- A sensor-backed ball count: configure four occupied-slot digital input names or override `createBallCounter()` for your actual indexer sensors. Empty/full counts are never inferred from time. Add appropriate filtering to your hardware counter if its readings chatter.

Then set `calibrated=true`. Hardware initialization and Pinpoint calibration happen in INIT while the robot is still. Motors and mechanisms stop in the OpMode's `finally` block. No WebSocket connection, browser or laptop is required when the controller runs on the robot.

## Sensor behavior and limits

The browser synthesizes Pinpoint pose and measured inventory. Its virtual Limelight has a robot-mounted origin, up/down camera pitch, finite field of view/range, tag-facing checks, conservative support/flower occlusion, 30 FPS, 60 ms delay and a pipeline-switch delay. Optional seeded angle noise and dropped detections are available in `SimSensors` constructor options. The UI camera checkbox tests missing detections.

The robot adapter reads actual Pinpoint and Limelight data, checks result validity/pipeline, returns individual tag IDs/angles, and accounts for result staleness and capture/processing latency. Java rejects stale pose/vision, ignores the other alliance's tags and never shoots using perfect simulator coordinates as an aiming shortcut.

Synthetic results test control behavior. They do not execute Limelight image recognition, reproduce every occluder, estimate raw encoder dynamics, or guarantee real-world trajectory accuracy. Validate camera calibration, wheel speed and trajectories on your robot. Ground-ball tracking currently uses the first result of the configured ball pipeline.

## Protocol and checks

The local browser connects to `/robot` over WebSocket. Messages carry `protocol:1`, a fresh `runId`, incrementing `seq`, and `time` in seconds. `start` supplies alliance/power and sensors; `step` supplies sensors exactly 20 ms later; `stop` clears actuators. Replies contain actuator powers, pipeline and telemetry. One session is allowed at a time; stale/out-of-order messages stop the controller. The server binds to loopback and rejects cross-origin browser connections.

```sh
npm run java:build
npm run check
```

Checks run the real JVM against cannon-es for both alliances, live firing/refill, tag loss, delayed pipelines, protocol/reconnect/manual stop, and the existing game regressions. The full 30-second run may end after one shooting/refill cycle because travel time and retries consume the remaining time; the tests do not claim it completes every pickup source.

The shared and FTC adapter sources were additionally compiled against the team's FTC SDK 12.0.0 Hardware and RobotCore artifacts. This is a compile check, not a Control Hub hardware test.
