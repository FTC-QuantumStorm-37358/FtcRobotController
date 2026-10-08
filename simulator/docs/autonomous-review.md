# BIOBUZZ autonomous review

This review uses the supplied **BIOBUZZ Competition Manual V1**, not later rule updates. It compares the local robot repository with the local simulator. The browser controller is a simulator extension; the robot's Java files have not been modified.

## Why the old button could look inactive

The playground has a real pose. Its default robot center is **X = -12.75 in, Z = -63.945 in, heading = 180 degrees**. X/Z are horizontal inches from field center; positive Z points toward the audience-side fence in the model; heading 0 points toward negative Z. Zero is a valid coordinate, not an indication of missing data. Hardware encoders are not needed to read this simulator pose.

In browser verification, the old button did move the robot. However, the old AutoTest adaptation moved only 12.5 inches forward, paused, reversed back toward the starting fence, then stopped at the boundary after roughly three seconds. It ended nearly where it began. On a narrow screen the button was below the field, so movement could happen outside the visible portion of the page. Changing focus also cancelled that previous routine. These observations explain ways movement can be missed; they do not prove which occurred on the user's click.

The replacement shows live pose, destination, distance remaining, elapsed time, fired count and loading-zone overlap. Starting it displays the whole field. A second AUTO button and compact progress display sit in the field view. Visible autonomous operation continues when keyboard focus leaves the page; hiding the page stops it. Manual driving cancels practice AUTO.

## What the Java actually contains

- `TeamCode/.../quantumstorm/StateMachine.java` declares INIT, MOVING, READY_TO_SHOOT, READY_TO_PICKUP, ERROR and PARK. It advances through the first three transitions unconditionally. There is no drivetrain instance, motor command, position measurement, target coordinate, shooting or intake action. READY_TO_PICKUP never exits; ERROR and PARK are empty. It has no completion condition or autonomous elapsed-time check.
- `TeamCode/.../quantumstorm/Tests/AutoTest.java` commands only `left_front_drive` and `right_front_drive`. Its actual sleeps are 1 second forward at 0.5 power, 0.5 second stationary and 2 seconds backward at -0.5 power. Its forward comment incorrectly says 2 seconds. It is a motor smoke test rather than a BIOBUZZ route, and it does not operate the two rear motors of the four-motor mecanum drivetrain.
- `Routing.java` is empty.
- `DriveChain/MecanumDrive.java` provides four-motor robot-relative motion and a stop method, but no localization or waypoint follower. StateMachine does not call it.
- `Apriltags.java` reports tag IDs and image angles. It does not fuse them into robot field position or drive to a target. The separate disabled Limelight example displays botpose but is not connected to StateMachine.

## Manual requirements versus implementation

| Manual reference | Requirement or objective | Java gap | Local simulator replacement / remaining gap |
|---|---|---|---|
| §10.3.4, G304, pp. 85, 104 | Start touching perimeter, wholly on own alliance side, outside loading zone, with exactly four pollen; stationary after INIT | No start validation or seeded field pose | Reads actual current pose; rejects invalid pose or mismatched alliance. Default fence pose has four pollen. Starting elsewhere remains a practice scenario, not a certified legal match setup. |
| §10.4, G305, G401, G403, pp. 85, 105–106 | 30-second AUTO, no driver input except start/stop/safety, motionless transition | No timer-based mission completion or transition handling | Stops driving/firing no later than 30 seconds using a monotonic clock. This remains a practice page: manual takeover cancels AUTO; there is no full 8-second transition / TELEOP match clock. |
| §10.5.4–5, pp. 90–91 | LEAVE: no longer contact perimeter (3 AUTO points). PARK: at least partly overlap loading zone (5 AUTO points) | No achievement detection, parking target or navigation | Uses actual chassis corners to check wall clearance and loading-zone overlap. Drives to own loading zone and stops away from perimeter. Does not claim official scored points. |
| §10.5.1, G417, pp. 87, 111–112 | Tip only by launching into upward-facing hive cell; completed stable-state tip is worth 20 points in AUTO | No aiming, shooting, feedback for hive state or tip detection | Reads live raised cell, turns the fixed front shooter, finds a descending arc for the queue head, fires original loaded balls, and waits while hive is tipping. Existing ball/cell collision physics determines actual retention and tipping; a predicted arc or four fired balls does not guarantee a tip. |
| G407–409, pp. 108–109 | At most four controlled balls; do not control opponent nectar; do not catch freshly spilled hive balls | No ball count, type or pickup interlocks | Existing four-slot capacity is preserved. Own-side route and opponent-nectar firing rejection added. A comprehensive opponent-ball collection interlock and fresh-spill no-catch enforcement are still absent from the simulator. |
| G402, p. 106 | Do not disrupt opponent AUTO | No alliance-aware route or obstacle handling | Whole chassis stays on selected alliance's half; routes check walls, supports and flowers. Other robots are not simulated, so partner/opponent collision avoidance is still missing. Staying on own half is a conservative strategy, not a blanket rule that crossing is always prohibited. |
| G418, p. 112 | Retrieve only pollen from flower bottom; enter balls only through flower top | READY_TO_PICKUP does nothing; no mechanism sequencing | Existing front-only pickup and bottom flower release remain. Automatic flower approach/retrieval and repeat shooting cycles are not implemented. |
| §10.5.2, G410, pp. 88, 110 | Manual says flower scoring begins only in final minute; G410 specifically prohibits nectar entry earlier | No match-phase gates or scoring strategy | New AUTO only shoots at hive and parks. It does not score into flowers. A complete match controller would need explicit phase gating and scoring accounting. |

## Position-guided browser routine

1. Read current `game.pose` and the selected alliance; never replace a missing/invalid pose with zeros.
2. Navigate to the selected rendered hive's X position and a shooting station 52 inches from field center along Z: red near (-12.75, -52), blue near (12.75, 52). These stations are simulator strategy choices, not coordinates mandated by the manual.
3. Use actual heading to transform field-position error into forward/strafe input. Plan a direct path or a four-inch-grid detour around field obstacles. Sample each route edge at no more than half-inch spacing and check the chassis footprint throughout. The final movement step is also validated.
4. Turn toward the live raised cell, re-evaluate the queue-head arc and attempt up to the originally loaded count of shots. Skip unavailable shots and reserve time for parking. Keep the shooter fixed to the robot.
5. Navigate to a safe partial-overlap pose in the loading zone read from the same data used to render its floor marking: red near (-56, -36), blue near (56, 36). Confirm actual chassis/zone intersection rather than equating PARK with zero motor power.
6. Stop on completion, user stop, manual takeover, hidden page, invalid route, persistent blockage or 30-second deadline.

The controller uses exact simulated pose. A physical robot still needs a pose source (encoder odometry and heading, or calibrated external localization), a defined coordinate transform and starting pose, four-motor commands, realistic acceleration and speed calibration, mechanism control, sensor-based completion guards, timeouts and stop-on-error behavior. A waypoint defined in simulated inches is not directly a calibrated hardware motor command.

## Field-setup discrepancies to address separately

Manual §10.3.1 / Figure 10-2, pp. 83–84, specifies forty pollen as **16 in flowers, 8 in gardens, 16 on four robots**, and sixteen nectar as **6 in raised hive cells plus 10 outside the field in alliance areas**.

The current single-robot physics setup keeps forty pollen by placing **4 on this robot, 16 in flowers and 20 in gardens**. It puts the other ten nectar in loading zones inside the field. This is a practice redistribution, not the official V1 starting layout. A competition-accurate four-robot setup would need three additional robots/preload sets, four pollen in each garden, outside-field nectar staging and controlled human entry. This change preserves the established practice setup rather than silently altering existing balls and tests.

The simulator also lacks a complete official score/penalty system, driver-station lifecycle, partner coordination, dynamic robot obstacles, referee judgment, hardware sensor noise/slip and validation of every construction rule. Its approximate mass-based hive tip model is not proof that a real hive will tip from the same sequence.

## Verification

Checks execute the new controller against actual GamePhysics at 30 and 60 frames per second for red and blue starts. They check every moved footprint, own-side constraints, four actual launch calls with descending arc predictions, wall clearance and loading-zone PARK. Additional starts require a support detour and heading change. Cancellation, restart, invalid pose, wrong alliance, heading wrap, unreachable destination and 30-second cutoff are covered. All existing simulator checks must continue to pass. Browser verification separately checks the buttons, live coordinates, shots and final park display.
