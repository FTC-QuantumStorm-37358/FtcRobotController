package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.List;

/**
 * Approaches the largest visible blue or yellow object while reporting other detections.
 * Revised for the REV Driver Hub screen layout and Camera Stream preview.
 * Configure a USB webcam as a Webcam device named "limelight 3a".
 */
@Autonomous(name = "Approach Blue or Yellow", group = "Vision")
public class PollenAndAprilTagDetection extends LinearOpMode {

    // These names/settings must match the Robot Configuration and tune movement behavior.
    private static final String WEBCAM_NAME = "limelight 3a";
    private static final int MIN_BLOB_AREA = 80;
    private static final int MAX_BLOB_AREA = 640 * 480;
    private static final double APPROACH_POWER = 0.12;
    private static final double STRAFE_GAIN = 0.0008;
    private static final double MAX_STRAFE_POWER = 0.12;
    private static final int CENTER_TOLERANCE_PIXELS = 25;
    private static final double STOP_BLOB_AREA = 12000;

    // @Override marks this as the runOpMode method required by LinearOpMode.
    @Override
    // public = FTC can access it; void = it returns no value; () = it takes no inputs.
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        // Intake is one motor configured in Robot Configuration as "intake".
        DcMotor intake = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        // Each call creates a detector configured for the color in parentheses.
        ColorBlobLocatorProcessor yellowLocator = createColorLocator(ColorRange.YELLOW);
        ColorBlobLocatorProcessor blueLocator = createColorLocator(ColorRange.BLUE);
        ColorBlobLocatorProcessor redLocator = createColorLocator(ColorRange.RED);

        // Builder sets up the AprilTag detector; build() creates it.
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        // VisionPortal connects the named camera to the detectors and starts image processing.
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(640, 480))
                .addProcessors(yellowLocator, blueLocator, redLocator, aprilTagProcessor)
                .build();

        // Speed up Driver Hub screen refreshes (100 ms = 10 updates per second).
        telemetry.setMsTransmissionInterval(100);

        // Add status messages for the Driver Hub screen, then display them.
        telemetry.addLine("Will approach the largest visible blue or yellow object.");
        telemetry.addLine("Drive power is limited; test in a clear area.");
        telemetry.addLine("Driver Hub: Tap 3-dot menu -> Camera Stream to preview.");
        telemetry.addLine("Press Start to begin.");
        telemetry.update();

        // Wait here until the driver presses Start on the Driver Hub.
        waitForStart();

        // Repeat this block while the OpMode is running.
        while (opModeIsActive()) {
            List<ColorBlobLocatorProcessor.Blob> yellowBlobs = yellowLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> blueBlobs = blueLocator.getBlobs();

            // getBlobs() gets color patches; reportColorBlobs() displays them.
            reportColorBlobs("Yellow pollen", yellowBlobs);
            reportColorBlobs("Blue objects", blueBlobs);
            reportColorBlobs("Red objects", redLocator.getBlobs());

            // Choose the largest blue or yellow blob as the approach target.
            ColorBlobLocatorProcessor.Blob target = getLargestBlob(blueBlobs);
            String targetColor = "Blue";
            ColorBlobLocatorProcessor.Blob yellowTarget = getLargestBlob(yellowBlobs);
            if (yellowTarget != null && (target == null
                    || yellowTarget.getContourArea() > target.getContourArea())) {
                target = yellowTarget;
                targetColor = "Yellow";
            }

            if (target == null) {
                // Stop rather than moving blindly when neither target color is visible.
                drive.stop();
                intake.setPower(0);
                telemetry.addLine("No blue or yellow target; stopped.");
            } else {
                RotatedRect targetBox = target.getBoxFit();
                // Positive error means the target appears to the right of the image center.
                double horizontalError = targetBox.center.x - 320.0;
                double targetArea = target.getContourArea();

                // A bigger image area usually means the object is closer; stop at this threshold.
                if (targetArea >= STOP_BLOB_AREA) {
                    drive.stop();
                    intake.setPower(Constants.INTAKE_POWER);
                    telemetry.addData("Approach", "%s target close; stopped", targetColor);
                } else {
                    // Strafe toward the target's horizontal position, but cap turn power.
                    double strafe = Math.abs(horizontalError) <= CENTER_TOLERANCE_PIXELS
                            ? 0.0
                            : clamp(horizontalError * STRAFE_GAIN,
                            -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
                    drive.drive(APPROACH_POWER, strafe, 0);
                    intake.setPower(Constants.INTAKE_POWER);
                    telemetry.addData("Approach", "Moving toward %s", targetColor);
                    telemetry.addData("Target center / area", "x=%.0f px, %.0f px²",
                            targetBox.center.x, targetArea);
                }
            }

            // Get the AprilTags visible in the current camera frame.
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            // Show how many tag detections were found.
            telemetry.addData("AprilTags", detections.size());
            // Check each detected tag one at a time.
            for (AprilTagDetection detection : detections) {
                if (!(detection instanceof AprilTagSingleDetection)) {
                    telemetry.addLine("AprilTag cluster visible");
                    continue;
                }
                AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
                // Metadata and pose provide the tag name and its measured position.
                if (tag.metadata != null && tag.ftcPose != null) {
                    telemetry.addData("Tag " + tag.id,
                            "%s | range %.1f in | bearing %.1f deg",
                            tag.metadata.name,
                            tag.ftcPose.range,
                            tag.ftcPose.bearing);
                } else {
                    // The camera sees the tag, but could not calculate its 3D pose.
                    telemetry.addData("Tag " + tag.id,
                            "visible at (%.0f, %.0f) (pose unavailable)",
                            tag.center.x,
                            tag.center.y);
                }
            }

            telemetry.update();
            // Pause briefly so the loop does not run continuously at full speed.
            sleep(50);
        }

        // Release the camera when the OpMode stops.
        intake.setPower(0);
        drive.stop();
        visionPortal.close();
    }

    // colorRange is the color this new detector should look for.
    private ColorBlobLocatorProcessor createColorLocator(ColorRange colorRange) {
        // Set the color to detect and turn on outlines in the camera preview.
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorRange)
                // Scan the full camera frame (required by the FTC SDK).
                .setRoi(ImageRegion.entireFrame())
                // Only use the outside edge of each detected color patch.
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                // Blur a little to reduce small noisy variations in the image.
                .setBlurSize(5)
                .build();
    }

    // label is the telemetry name; rawBlobs is the list returned by the color detector.
    private void reportColorBlobs(String label, List<ColorBlobLocatorProcessor.Blob> rawBlobs) {
        // Make a modifiable copy of the list so filtering does not cause an error.
        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>(rawBlobs);

        // Ignore color patches that are too small or too large.
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                MIN_BLOB_AREA,
                MAX_BLOB_AREA,
                blobs);

        // Show the number of blobs on the Driver Hub.
        telemetry.addData(label, "%d detected", blobs.size());
        // Show details for at most 2 blobs so everything fits on the Driver Hub screen.
        int shown = Math.min(blobs.size(), 2);
        // Repeat once for each blob chosen to display.
        for (int i = 0; i < shown; i++) {
            // Fit a rotated rectangle around this blob to estimate its center.
            RotatedRect box = blobs.get(i).getBoxFit();
            telemetry.addData(label + " " + (i + 1), "center (%d, %d), area %d px",
                    (int) box.center.x,
                    (int) box.center.y,
                    (int) blobs.get(i).getContourArea());
        }
    }

    private ColorBlobLocatorProcessor.Blob getLargestBlob(
            List<ColorBlobLocatorProcessor.Blob> rawBlobs) {
        // Filter a copy, then return the largest remaining patch (or null if none remain).
        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>(rawBlobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                MIN_BLOB_AREA,
                MAX_BLOB_AREA,
                blobs);

        ColorBlobLocatorProcessor.Blob largest = null;
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            if (largest == null || blob.getContourArea() > largest.getContourArea()) {
                largest = blob;
            }
        }
        return largest;
    }

    // Keep a calculated motor power within the given minimum and maximum.
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /*
     * Coding Translations
     * @Autonomous = makes this program selectable as an Autonomous OpMode.
     * runOpMode() = the instructions FTC runs for this OpMode.
     * createColorLocator(ColorRange.BLUE) = creates a blue detector.
     * ImageRegion.entireFrame() = tells the color detector to scan the whole camera view.
     * new AprilTagProcessor.Builder().build() = creates an AprilTag detector.
     * VisionPortal.Builder() = connects the camera to image processors.
     * setMsTransmissionInterval(100) = updates the Driver Hub screen every 100 milliseconds.
     * waitForStart() = waits for the driver to press Start.
     * opModeIsActive() = checks whether the OpMode is still running.
     * locator.getBlobs() = gets matching color patches from the camera.
     * reportColorBlobs(label, blobs) = shows the blob count and details.
     * getLargestBlob(blobs) = finds the biggest filtered color patch.
     * drive.drive(forward, strafe, turn) = sends movement commands to the drive motors.
     * intake.setPower(...) = turns the single intake motor on or off.
     * targetBox.center.x = the target's horizontal pixel position in the camera image.
     * STOP_BLOB_AREA = image size at which the robot stops approaching.
     * clamp(value, min, max) = keeps a number within safe limits.
     * List = a collection of items; here, detected blobs or tags.
     * new ArrayList<>(rawBlobs) = makes a copy of the list that can be filtered safely.
     * blobs.size() = how many blobs are in the collection.
     * Math.min(a, b) = returns the smaller number.
     * for (... : ...) = repeats once for each item in a collection.
     * setDrawContours(true) = draws outlines around detected patches in the Camera Stream.
     * aprilTagProcessor.getDetections() = gets the AprilTags seen by the camera.
     * tag.ftcPose.range / bearing = distance in inches and angle in degrees to the AprilTag.
     * telemetry.addData(...) = shows information on the Driver Hub screen.
     * visionPortal.close() = releases the camera when the program ends.
     */
}
