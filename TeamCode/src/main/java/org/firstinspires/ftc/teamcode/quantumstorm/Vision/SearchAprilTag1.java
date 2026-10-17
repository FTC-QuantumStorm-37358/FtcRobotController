package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/** Detects AprilTag 1 and reports it without controlling any motors or servos. */
@Autonomous(name = "Detect AprilTag 1", group = "Vision")
public class SearchAprilTag1 extends LinearOpMode {

    private static final String WEBCAM_NAME = "limelight 3A";
    private static final int TARGET_TAG_ID = 1;

    @Override
    public void runOpMode() {
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(320, 240))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("Camera ready. Press Start to look for AprilTag 1.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                boolean targetDetected = false;
                for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                    if (detection instanceof AprilTagSingleDetection
                            && ((AprilTagSingleDetection) detection).id == TARGET_TAG_ID) {
                        targetDetected = true;
                        break;
                    }
                }

                telemetry.addData("AprilTag 1", targetDetected
                        ? "Target detected"
                        : "Not detected");
                telemetry.update();
                sleep(50);
            }
        } finally {
            portal.close();
        }
    }
}
