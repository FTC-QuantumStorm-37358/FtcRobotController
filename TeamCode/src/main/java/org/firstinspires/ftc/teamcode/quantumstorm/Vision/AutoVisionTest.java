package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;

import java.util.List;

/**
 * Configurable during INIT via Gamepad 1:
 * - B / X: Toggle Red or Blue Alliance
 * - Dpad Up / Down: Set initial servo angle (Up = Hive AprilTags, Down = Balls)
 *
 * When aimed DOWN: Searches for alliance balls (or yellow), approaches them,
 * and automatically tilts the camera UP once close to a ball.
 * When aimed UP: Searches for alliance AprilTags and centers the robot to fire at the Hive.
 */
@Autonomous(name = "Auto Vision Test", group = "Vision")
public class AutoVisionTest extends LinearOpMode {

    // Default alliance setting (changeable with Gamepad 1 during Init)
    private boolean red = false;

    private static final String WEBCAM_NAME = "limelight 3A";

    // Servo preset positions (0.0 to 1.0 corresponds to 0 to 180 degrees)
    // Tune these two numbers so DOWN looks at the floor and UP looks at the Hive tags!
    private static final double SERVO_DOWN_BALL_POS = 0.20; // ~36 degrees (aimed down at balls)
    private static final double SERVO_UP_TAG_POS = 0.55;    // ~99 degrees (aimed up at Hive AprilTags)

    // Any servo angle >= 70 degrees is treated as "Aimed Up" (AprilTag Hive Mode)
    private static final double APRILTAG_MODE_MIN_SERVO_DEGREES = 70.0;

    private static final int FRAME_WIDTH = 320;
    private static final int FRAME_HEIGHT = 240;
    private static final int MIN_BLOB_AREA = 80;
    private static final int MAX_BLOB_AREA = FRAME_WIDTH * FRAME_HEIGHT;

    // Ball-chasing speeds and thresholds
    private static final double MAX_FORWARD_POWER = 0.22;
    private static final double MAX_STRAFE_POWER = 0.18;
    private static final double SEARCH_TURN_POWER = 0.14;
    private static final double AVOID_BACKUP_POWER = 0.16;
    private static final double AVOID_TURN_POWER = 0.16;
    private static final int TARGET_STOP_AREA = 3500;
    private static final int CENTER_TOLERANCE_PIXELS = 18;

    // Hive AprilTag centering speeds and thresholds
    private static final double TAG_SEARCH_TURN_POWER = 0.12;
    private static final double TARGET_FIRE_RANGE_IN = 36.0;    // Desired firing distance from Hive (inches)
    private static final double RANGE_TOLERANCE_IN = 3.0;       // Allowed distance error (+/- inches)
    private static final double BEARING_TOLERANCE_DEG = 2.5;    // Allowed aiming angle error (+/- degrees)
    private static final double TAG_FORWARD_GAIN = 0.015;       // Speed scaling for distance adjustment
    private static final double TAG_TURN_GAIN = 0.012;          // Speed scaling for aiming rotation
    private static final double MAX_TAG_ALIGN_TURN = 0.18;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        DcMotor intake = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        Servo cameraServo = hardwareMap.get(Servo.class, Constants.LIMELIGHT_TILT_SERVO);

        ColorBlobLocatorProcessor yellowLocator = createColorLocator(ColorRange.YELLOW);
        ColorBlobLocatorProcessor blueLocator = createColorLocator(ColorRange.BLUE);
        ColorBlobLocatorProcessor redLocator = createColorLocator(ColorRange.RED);
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(FRAME_WIDTH, FRAME_HEIGHT))
                .addProcessors(yellowLocator, blueLocator, redLocator, aprilTagProcessor)
                .build();

        // Start aimed down at balls by default during Init
        double currentServoCommand = SERVO_DOWN_BALL_POS;
        cameraServo.setPosition(currentServoCommand);

        // --- INTERACTIVE INIT LOOP (Runs before you press Start) ---
        while (!isStarted() && !isStopRequested()) {
            // Toggle Alliance with B (Red) and X (Blue)
            if (gamepad1.b) {
                red = true;
            } else if (gamepad1.x) {
                red = false;
            }

            // Toggle starting camera angle with D-Pad Up (Hive Tags) or D-Pad Down (Balls)
            if (gamepad1.dpad_up) {
                currentServoCommand = SERVO_UP_TAG_POS;
                cameraServo.setPosition(currentServoCommand);
            } else if (gamepad1.dpad_down) {
                currentServoCommand = SERVO_DOWN_BALL_POS;
                cameraServo.setPosition(currentServoCommand);
            }

            double initDegrees = cameraServo.getPosition() * 180.0;
            boolean initTagMode = initDegrees >= APRILTAG_MODE_MIN_SERVO_DEGREES;

            telemetry.addLine("=== INIT CONFIGURATION (Use Gamepad 1) ===");
            telemetry.addData("1. Alliance (Press B for Red, X for Blue)", red ? "RED" : "BLUE");
            telemetry.addData("2. Start Mode (D-Pad UP/DOWN)", initTagMode
                    ? "AIMED UP (Search AprilTags & Align to Hive)"
                    : "AIMED DOWN (Search for Balls)");
            telemetry.addData("Servo Position", "%.2f (~%.0f deg)", cameraServo.getPosition(), initDegrees);
            telemetry.addLine("Press START when ready.");
            telemetry.update();
            sleep(20);
        }

        if (!opModeIsActive()) {
            intake.setPower(0);
            drive.stop();
            portal.close();
            return;
        }

        String targetColorName = red ? "RED" : "BLUE";
        String avoidColorName = red ? "BLUE" : "RED";

        try {
            while (opModeIsActive()) {
                double servoPosition = cameraServo.getPosition();
                double servoDegrees = servoPosition * 180.0;
                // Aimed UP (>= 70 deg) = AprilTag Hive Mode; Aimed DOWN (< 70 deg) = Ball Mode
                boolean aimedUpAtHive = servoDegrees >= APRILTAG_MODE_MIN_SERVO_DEGREES;

                List<ColorBlobLocatorProcessor.Blob> yellowBlobs = filteredBlobs(yellowLocator);
                List<ColorBlobLocatorProcessor.Blob> blueBlobs = filteredBlobs(blueLocator);
                List<ColorBlobLocatorProcessor.Blob> redBlobs = filteredBlobs(redLocator);

                List<ColorBlobLocatorProcessor.Blob> targetColorBlobs = red ? redBlobs : blueBlobs;
                List<ColorBlobLocatorProcessor.Blob> hazardColorBlobs = red ? blueBlobs : redBlobs;
                List<AprilTagDetection> tags = aprilTagProcessor.getDetections();

                if (aimedUpAtHive) {
                    intake.setPower(0);
                    // =========================================================
                    // AIMED UP: SEARCH FOR APRILTAGS & CENTER TO FIRE AT HIVE
                    // =========================================================
                    AprilTagSingleDetection hiveTag = null;
                    for (AprilTagDetection detection : tags) {
                        if (detection instanceof AprilTagSingleDetection) {
                            AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
                            if (isMyAllianceTag(tag.id) && tag.ftcPose != null) {
                                hiveTag = tag;
                                break;
                            }
                        }
                    }

                    if (hiveTag == null) {
                        // No alliance Hive tag in view yet — spin to search
                        drive.drive(0, 0, TAG_SEARCH_TURN_POWER);
                        telemetry.addData("Action", "Aimed UP: Searching for %s Hive AprilTag", targetColorName);
                    } else {
                        // Calculate how far off-center (bearing) and off-distance (range) we are
                        double bearingError = hiveTag.ftcPose.bearing; // 0 deg is dead center
                        double rangeError = hiveTag.ftcPose.range - TARGET_FIRE_RANGE_IN;

                        boolean centeredAngle = Math.abs(bearingError) <= BEARING_TOLERANCE_DEG;
                        boolean atFiringDistance = Math.abs(rangeError) <= RANGE_TOLERANCE_IN;

                        if (centeredAngle && atFiringDistance) {
                            drive.stop();
                            telemetry.addData("Action", "LOCKED ON HIVE (Tag %d)! Ready to fire!", hiveTag.id);
                            // If you want the OpMode to end once aligned, uncomment the next line:
                            // requestOpModeStop();
                        } else {
                            // Proportional turn to center the tag, and forward/back to hit firing range
                            double turn = centeredAngle ? 0 : clamp(
                                    -bearingError * TAG_TURN_GAIN,
                                    -MAX_TAG_ALIGN_TURN, MAX_TAG_ALIGN_TURN);
                            double forward = atFiringDistance ? 0 : clamp(
                                    rangeError * TAG_FORWARD_GAIN,
                                    -MAX_FORWARD_POWER, MAX_FORWARD_POWER);

                            drive.drive(forward, 0, turn);
                            telemetry.addData("Action", "Centering on Hive Tag %d (Range: %.1f in, Bearing: %.1f deg)",
                                    hiveTag.id, hiveTag.ftcPose.range, hiveTag.ftcPose.bearing);
                        }
                    }

                } else {
                    // =========================================================
                    // AIMED DOWN: SEARCH FOR BALLS & AVOID OPPOSING COLOR
                    // =========================================================
                    if (!hazardColorBlobs.isEmpty()) {
                        intake.setPower(0);
                        ColorBlobLocatorProcessor.Blob hazard = largest(hazardColorBlobs);
                        double hazardX = hazard.getBoxFit().center.x;
                        double turnAway = hazardX >= FRAME_WIDTH / 2.0
                                ? -AVOID_TURN_POWER : AVOID_TURN_POWER;
                        drive.drive(-AVOID_BACKUP_POWER, 0, turnAway);
                        telemetry.addData("Action", "%s SEEN: backing up and turning away", avoidColorName);
                    } else {
                        ColorBlobLocatorProcessor.Blob target = !targetColorBlobs.isEmpty()
                                ? largest(targetColorBlobs)
                                : (!yellowBlobs.isEmpty() ? largest(yellowBlobs) : null);

                        if (target == null) {
                            intake.setPower(0);
                            drive.drive(0, 0, SEARCH_TURN_POWER);
                            telemetry.addData("Action", "Aimed DOWN: Searching for %s / yellow balls",
                                    targetColorName.toLowerCase());
                        } else {
                            RotatedRect box = target.getBoxFit();
                            double horizontalError = box.center.x - FRAME_WIDTH / 2.0;
                            int area = largestArea(target);

                            if (area >= TARGET_STOP_AREA) {
                                // Run the intake only once the ball is close enough to collect.
                                intake.setPower(Constants.INTAKE_POWER);
                                // Ball reached! Stop driving and angle servo UP to align with the Hive
                                drive.stop();
                                cameraServo.setPosition(SERVO_UP_TAG_POS);
                                telemetry.addData("Action", "Ball reached! Tilting servo UP to find Hive...");
                                sleep(300); // Brief pause to let the servo physically tilt up
                            } else {
                                intake.setPower(0);
                                double strafe = Math.abs(horizontalError) <= CENTER_TOLERANCE_PIXELS
                                        ? 0
                                        : clamp(horizontalError / (FRAME_WIDTH / 2.0) * MAX_STRAFE_POWER,
                                                -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
                                drive.drive(MAX_FORWARD_POWER, strafe, 0);
                                telemetry.addData("Action", "Moving toward %s ball",
                                        !targetColorBlobs.isEmpty() ? targetColorName.toLowerCase() : "yellow pollen");
                            }
                            telemetry.addData("Ball center", "x=%d y=%d area=%d px",
                                    (int) box.center.x, (int) box.center.y, area);
                        }
                    }
                }

                // --- TELEMETRY STATUS ---
                telemetry.addData("Alliance", targetColorName);
                telemetry.addData("Vision Mode", aimedUpAtHive ? "AIMED UP (Hive AprilTags)" : "AIMED DOWN (Balls)");
                telemetry.addData("Camera Servo", "%.2f (~%.0f deg)", servoPosition, servoDegrees);
                telemetry.addData("Blobs (Y / B / R)", "%d / %d / %d",
                        yellowBlobs.size(), blueBlobs.size(), redBlobs.size());
                telemetry.addData("AprilTags Seen", tags.size());
                telemetry.update();
                sleep(50);
            }
        } finally {
            intake.setPower(0);
            drive.stop();
            portal.close();
        }
    }

    /** Checks if the detected AprilTag belongs to our currently selected alliance. */
    private boolean isMyAllianceTag(int id) {
        if (red) {
            return id >= 30 && id <= 37; // Red Hive tag range
        } else {
            return id >= 37 && id <= 45; // Blue Hive tag range
        }
    }

    private ColorBlobLocatorProcessor createColorLocator(ColorRange range) {
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(range)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBlurSize(5)
                .build();
    }

    private List<ColorBlobLocatorProcessor.Blob> filteredBlobs(ColorBlobLocatorProcessor processor) {
        List<ColorBlobLocatorProcessor.Blob> blobs = processor.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                MIN_BLOB_AREA, MAX_BLOB_AREA, blobs);
        return blobs;
    }

    private ColorBlobLocatorProcessor.Blob largest(List<ColorBlobLocatorProcessor.Blob> blobs) {
        ColorBlobLocatorProcessor.Blob best = blobs.get(0);
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            if (blob.getContourArea() > best.getContourArea()) {
                best = blob;
            }
        }
        return best;
    }

    private int largestArea(ColorBlobLocatorProcessor.Blob blob) {
        return blob.getContourArea();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}