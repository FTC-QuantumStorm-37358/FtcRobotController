package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;

/** Searches for the selected nectar color and yellow pollen. */
@Autonomous(name = "Search for Nectar and Pollen", group = "Vision")
public class Colordetection extends LinearOpMode {

    private static final String WEBCAM_NAME = "limelight 3A";
    private static final int FRAME_WIDTH = 320;
    private static final int FRAME_HEIGHT = 240;
    private static final int MIN_BLOB_AREA = 80;
    private static final int MAX_BLOB_AREA = 20000;
    private static final boolean BLUE = true;

    @Override
    public void runOpMode() {
        WebcamName camera = hardwareMap.tryGet(WebcamName.class, WEBCAM_NAME);
        if (camera == null) {
            telemetry.addData("Camera", "Not found; check Robot Configuration name '%s'.",
                    WEBCAM_NAME);
            telemetry.update();
            waitForStart();
            return;
        }

        ColorBlobLocatorProcessor nectarLocator = createColorLocator(
                BLUE ? ColorRange.BLUE : ColorRange.RED);
        ColorBlobLocatorProcessor pollenLocator = createColorLocator(ColorRange.YELLOW);
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .setCameraResolution(new Size(FRAME_WIDTH, FRAME_HEIGHT))
                .addProcessors(nectarLocator, pollenLocator)
                .build();

        telemetry.addData("Nectar color", BLUE ? "Blue" : "Red");
        telemetry.addLine("Press Start to search for nectar and pollen.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                List<ColorBlobLocatorProcessor.Blob> nectarBlobs = filteredBlobs(nectarLocator);
                List<ColorBlobLocatorProcessor.Blob> pollenBlobs = filteredBlobs(pollenLocator);

                telemetry.addData("Nectar", nectarBlobs.isEmpty()
                        ? "Searching for " + (BLUE ? "blue" : "red")
                        : "Nectar seen");
                telemetry.addData("Pollen", pollenBlobs.isEmpty()
                        ? "Searching for yellow"
                        : "Pollen seen");
                telemetry.addData("Nectar blobs", nectarBlobs.size());
                telemetry.addData("Pollen blobs", pollenBlobs.size());
                telemetry.update();
                sleep(50);
            }
        } finally {
            visionPortal.close();
        }
    }

    private ColorBlobLocatorProcessor createColorLocator(ColorRange colorRange) {
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBlurSize(5)
                .build();
    }

    private List<ColorBlobLocatorProcessor.Blob> filteredBlobs(
            ColorBlobLocatorProcessor processor) {
        List<ColorBlobLocatorProcessor.Blob> blobs = processor.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                MIN_BLOB_AREA,
                MAX_BLOB_AREA,
                blobs);
        return blobs;
    }
}
