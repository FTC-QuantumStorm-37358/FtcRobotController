# FTC BIOBUZZ Simulator

## Run locally

Requires Node.js and JDK 17 or newer. Set `JAVA_HOME` if Java is not on your path; the server can also use Android Studio's bundled JDK on macOS. From the FTC repository:

```sh
cd simulator
npm start
```

Open **http://localhost:8000/**. No `npm install` is needed. Stop with Ctrl+C. The server binds to this computer only. Restart after Java edits; refresh after webpage edits.

## Play

| Control | Action |
|---|---|
| W / S | Forward / backward |
| A / D | Move sideways |
| Q / E | Rotate |
| Space or Shoot | Shoot the next loaded ball |
| Touch Q W E / A S D | Drive on phones; Shoot sits at the right edge |
| Drag / pinch or scroll | Orbit / zoom |
| Follow camera | Follow the robot and look toward its front |

Start with four pollen; maximum inventory is four. Touch flowers to release pollen and collect loose balls with the front intake. Side/rear contact pushes balls. Tap an inventory slot to select the next shot. Aim shooter, adjust speed/angle, or use Find the best arc. Reset practice restarts the field. This practice robot is red.

Compatible browser-standard gamepads use the HTML Gamepad API: left stick or D-pad moves, right stick rotates, **A shoots**, **X lowers shooting speed**, **Y raises shooting speed**. Hold X/Y for continuous adjustment.

## Autonomous and rules

Start autonomous runs the shared Java controller through the local server. Stop autonomous, Escape, manual driving, leaving the page or a disconnected bridge stops it. Synthetic Limelight enabled can test detection loss. Cyan lines show the route reported by Java. The requested sequence is shoot → Garden → shoot → Flower → shoot → park. At 26 seconds Java interrupts any remaining scoring, stops the shooter/feeder/intake, and starts driving to the alliance Loading Zone. Parking keeps the current heading to avoid wasting time turning. Completed or aborted scoring waits until 26 seconds to begin parking; sensor faults still stop immediately. At 30 seconds all actuators stop even if parking is incomplete. Restart the local server after changing Java code.

The default .395-power / 66.5-degree simulation now collects all four Flower pollen, fires the final batch, and confirms another stable Hive tilt from AprilTags before waiting for parking. A corner approach keeps the Flower outlet clear, and an oblique shooting waypoint avoids the long fence return. There is no extra-pickup phase: all mechanisms and drive stay off after the final confirmed tilt until 26 seconds. Both default alliance checks fire 12 balls, complete the second tilt at about 22.66 seconds, and finish parking at about 29.78 seconds. These are simulator results, not hardware calibration guarantees.

Start timed match resets the field and begins Java AUTO, followed by the 8-second transition and 120-second TELEOP. The score panel shows the red practice estimate. Minor/major fouls award 5/20 points to blue. Measurable violations are monitored; intent, human behavior and opponent interactions use Referee assessment. Warnings appear silently in the corner for five seconds and stay in penalty history. There is no audio. See [rule coverage and limitations](docs/penalty-review.md).

## FTC robot adapter

TeamCode is already configured to compile `robot/shared/` and `robot/ftc/` from this folder. Both the browser runner and robot use the same Java controller; no autonomous decisions are translated into JavaScript. Desktop/bridge files are not deployed to the robot.

Before enabling **BIOBUZZ Shared Java Red/Blue**, configure and validate `FtcHardwareConfig`: motor names/directions, odometry, camera, field poses, and measured ball-count sensors. `calibrated=false` prevents running unconfigured hardware. See [robot setup](robot/README.md). Existing QuantumStorm OpModes remain separate.

```sh
npm run check
```

Checks cover the real Java/WebSocket bridge, simulation physics, controls, sensors, scores and penalties. They do not replace a Control Hub hardware test. Vendored dependency licenses are kept in `dist/vendor/`.
