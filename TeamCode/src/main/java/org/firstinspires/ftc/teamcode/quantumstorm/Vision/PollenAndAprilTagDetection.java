package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;

import java.util.List;

/**
 * Finds yellow pollen, blue and red objects, and AprilTags at the same time.
 * Configure a webcam in the Robot Configuration with the name "Webcam 1".
 */
@TeleOp(name = "Vision: Pollen, Colors & AprilTags", group = "Vision")
public class PollenAndAprilTagDetection extends LinearOpMode {

    // These constants store settings that are reused below.
    private static final String WEBCAM_NAME = "Webcam 1";
    private static final int MIN_BLOB_AREA = 80;
    private static final int MAX_BLOB_AREA = 20000;

    // @Override marks this as the runOpMode method required by LinearOpMode.
    @Override
    // public = FTC can access it; void = it returns no value; () = it takes no inputs.
    public void runOpMode() {
        // Each call creates a detector configured for the color in parentheses.
        ColorBlobLocatorProcessor yellowLocator = createColorLocator(ColorRange.YELLOW);
        ColorBlobLocatorProcessor blueLocator = createColorLocator(ColorRange.BLUE);
        ColorBlobLocatorProcessor redLocator = createColorLocator(ColorRange.RED);

        // Builder sets up the AprilTag detector; build() creates it.
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // VisionPortal connects the named camera to the detectors and starts image processing.
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(320, 240))
                .addProcessors(yellowLocator, blueLocator, redLocator, aprilTagProcessor)
                .build();

        // Add status messages for the Driver Station screen, then display them.
        telemetry.addLine("Detecting yellow pollen, blue/red objects, and AprilTags.");
        telemetry.addLine("Camera preview is available from the Driver Station.");
        telemetry.addLine("Press Start to begin telemetry.");
        telemetry.update();

        // Wait here until the driver presses Start.
        waitForStart();

        // Repeat this block while the OpMode is running.
        while (opModeIsActive()) {
            // getBlobs() gets color patches; reportColorBlobs() displays them.
            reportColorBlobs("Yellow pollen", yellowLocator.getBlobs());
            reportColorBlobs("Blue objects", blueLocator.getBlobs());
            reportColorBlobs("Red objects", redLocator.getBlobs());

            // Get the AprilTags visible in the current camera frame.
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            // Show how many tag detections were found.
            telemetry.addData("AprilTags", detections.size());
            // Check each detected tag one at a time.
            for (AprilTagDetection detection : detections) {
                // Check that this detection is one tag, rather than a cluster of tags.
                if (detection instanceof AprilTagSingleDetection) {
                    // Cast lets this code use the single tag's ID and metadata.
                    AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
                    // Metadata and pose provide the tag name and its measured position.
                    if (tag.metadata != null && tag.ftcPose != null) {
                        telemetry.addData("Tag " + tag.id,
                                "%s | range %.1f in | bearing %.1f deg",
                                tag.metadata.name,
                                tag.ftcPose.range,
                                tag.ftcPose.bearing);
                    } else {
                        // The camera sees the tag, but could not calculate its pose.
                        telemetry.addData("Tag " + tag.id, "visible (pose unavailable)");
                    }
                } else {
                    // A cluster is a group detection, not one individual tag.
                    telemetry.addLine("AprilTag cluster visible");
                }
            }

            telemetry.update();
            // Pause briefly so the loop does not run continuously at full speed.
            sleep(50);
        }

        // Release the camera when the OpMode stops.
        visionPortal.close();
    }

    // colorRange is the color this new detector should look for.
    private ColorBlobLocatorProcessor createColorLocator(ColorRange colorRange) {
        // Set the color to detect and turn on outlines in the camera preview.
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorRange)
                // Only use the outside edge of each detected color patch.
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                // Blur a little to reduce small noisy variations in the image.
                .setBlurSize(5)
                .build();
    }

    // label is the telemetry name; blobs is the list returned by the color detector.
    private void reportColorBlobs(String label, List<ColorBlobLocatorProcessor.Blob> blobs) {
        // Ignore color patches that are too small or too large.
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                MIN_BLOB_AREA,
                MAX_BLOB_AREA,
                blobs);

        // Show the number of blobs on the Driver Station.
        telemetry.addData(label, "%d detected", blobs.size());
        // Show details for at most three blobs, so telemetry stays readable.
        int shown = Math.min(blobs.size(), 3);
        // Repeat once for each blob chosen to display.
        for (int i = 0; i < shown; i++) {
            // Fit a rotated rectangle around this blob to estimate its center.
            RotatedRect box = blobs.get(i).getBoxFit();
            telemetry.addData(label + " " + (i + 1), "center (%d, %d), area %d px",
                    (int) box.center.x,
                    (int) box.center.y,
                    blobs.get(i).getContourArea());
        }
    }

    /*
     * Coding Translations
     * @TeleOp = makes this program selectable as a TeleOp on the Driver Station.
     * runOpMode() = the instructions FTC runs for this OpMode.
     * createColorLocator(ColorRange.BLUE) = creates a blue detector.
     * new AprilTagProcessor.Builder().build() = creates an AprilTag detector.
     * VisionPortal.Builder() = connects the camera to image processors.
     * waitForStart() = waits for the driver to press Start.
     * opModeIsActive() = checks whether the OpMode is still running.
     * locator.getBlobs() = gets matching color patches from the camera.
     * reportColorBlobs(label, blobs) = shows the blob count and details.
     * List = a collection of items; here, detected blobs or tags.
     * blobs.size() = how many blobs are in the collection.
     * Math.min(a, b) = returns the smaller number.
     * for (... : ...) = repeats once for each item in a collection.
     * instanceof = checks what kind of object this detection is.
     * setDrawContours(true) = draws outlines around detected patches.
     * aprilTagProcessor.getDetections() = gets the AprilTags seen by the camera.
     * telemetry.addData(...) = shows information on the Driver Station.
     * visionPortal.close() = releases the camera when the program ends.
     */
}
