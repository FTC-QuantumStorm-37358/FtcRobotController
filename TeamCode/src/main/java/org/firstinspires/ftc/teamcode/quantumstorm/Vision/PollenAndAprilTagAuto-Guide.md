# Pollen and AprilTag Autonomous Guide

This guide describes `PollenAndAprilTagAuto.java`, an FTC autonomous OpMode that uses a webcam, a positional servo, color detection, and AprilTag detection.

## What the robot does

The OpMode reads the servo's commanded position while it runs. It assumes the servo's full command range represents 0 to 180 degrees:

- **70° through 120°:** AprilTag mode. The robot slowly rotates while searching. When it sees a listed red or blue tag with a measured range greater than 24 inches, it stops, displays **“Under fire spot!”**, and requests the OpMode to stop.
- **All other angles:** ball/color mode. A red object makes the robot back up and turn away. Otherwise it approaches the largest blue object; if no blue object is visible, it approaches yellow pollen. If no blue or yellow target is visible, it rotates to search.

The 24-inch cutoff is set by `FIRE_SPOT_FAR_RANGE_IN`. Change it in the Java file to use another distance. The AprilTag far-distance behavior only triggers when the camera supplies a tag pose/range.

## Tag IDs and alliance labels

| Label | Tag IDs |
| --- | --- |
| Blue Audience Side | 37, 38, 39, 40, 41 |
| Blue Scoring Side | 42, 43, 44, 45 |
| Red Cell, opposite Audience Side | 30, 31, 32, 33 |
| Red Cell, Audience Side | 34, 35, 36, 37 |

The code classifies blue IDs as 37–45 and red IDs as 30–37. Since ID 37 is in both lists provided, it is reported as **Blue/Red overlap**. IDs outside 30–45 are reported as other and do not trigger the far-tag stop.

## Robot Configuration

Configure these devices on the Driver Station Robot Configuration:

1. Use the Limelight 3A camera and give its Robot Configuration device the exact name **`limelight 3A`**. The name is case-sensitive and must match `WEBCAM_NAME` in the code.
2. Add the camera-tilt positional servo with the exact name **`limelight_tilt_servo`**. This matches `Constants.LIMELIGHT_TILT_SERVO`.
3. Configure the drive motors using the names expected by `MecanumDrive` in this project.

The OpMode reads the servo position; it does not move the servo itself. Another part of the program must set the servo position. FTC's standard `Servo.getPosition()` returns the last commanded value, not a measurement of the servo's physical angle. The 70°–120° rule converts that command using `degrees = position * 180`, so adjust the rule if the servo's usable travel does not match that assumption.

## Starting the OpMode

Select **“Auto: Find Blue/Yellow, Avoid Red”** in the Driver Hub autonomous list. Test with the robot raised or in a clear, supervised area first because the OpMode can drive and rotate by itself.

Source: [`PollenAndAprilTagAuto.java`](PollenAndAprilTagAuto.java)
