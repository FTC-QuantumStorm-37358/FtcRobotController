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

Start autonomous runs the shared Java controller through the local server. Stop autonomous, Escape, manual driving, leaving the page or a disconnected bridge stops it. Synthetic Limelight enabled can test detection loss. Cyan lines show the route reported by Java. The requested sequence is shoot → Garden → shoot → Flower → shoot → park. Parking has a 26-second deadline: Java reserves route-dependent travel time and can interrupt scoring or skip a pickup to reach the Loading Zone. At 26 seconds all actuators stop, even if a fault prevents arrival. Restart the local server after changing Java code.

Start timed match resets the field and begins Java AUTO, followed by the 8-second transition and 120-second TELEOP. The score panel shows the red practice estimate. Minor/major fouls award 5/20 points to blue. Measurable violations are monitored; intent, human behavior and opponent interactions use Referee assessment. Warnings appear silently in the corner for five seconds and stay in penalty history. There is no audio. See [rule coverage and limitations](docs/penalty-review.md).

## FTC robot adapter

TeamCode is already configured to compile `robot/shared/` and `robot/ftc/` from this folder. Both the browser runner and robot use the same Java controller; no autonomous decisions are translated into JavaScript. Desktop/bridge files are not deployed to the robot.

Before enabling **BIOBUZZ Shared Java Red/Blue**, configure and validate `FtcHardwareConfig`: motor names/directions, odometry, camera, field poses, and measured ball-count sensors. `calibrated=false` prevents running unconfigured hardware. See [robot setup](robot/README.md). Existing QuantumStorm OpModes remain separate.

```sh
npm run check
```

Checks cover the real Java/WebSocket bridge, simulation physics, controls, sensors, scores and penalties. They do not replace a Control Hub hardware test. Vendored dependency licenses are kept in `dist/vendor/`.
