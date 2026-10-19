package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

/** Reports every AprilTag detected by the Limelight's 36h11 pipeline. */
@Autonomous(name = "Search AprilTags (36h11)", group = "Vision")
public class SearchAprilTag1 extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight 3a";
    private static final int APRILTAG_PIPELINE = 0;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, LIMELIGHT_NAME);
        if (limelight == null) {
            telemetry.addData("Limelight", "Not found; check Robot Configuration name '%s'.",
                    LIMELIGHT_NAME);
            telemetry.update();
            return;
        }

        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Pipeline 0 must be configured for AprilTag family 36h11.");
        telemetry.addLine("Press Start to report visible tag IDs.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                telemetry.addData("Limelight connection", limelight.isConnected()
                        ? "Connected"
                        : "Not connected");
                LLResult result = limelight.getLatestResult();

                if (!limelight.isConnected()) {
                    telemetry.addLine("Check the Limelight power, network, and Robot Configuration.");
                } else if (result == null) {
                    telemetry.addLine("Waiting for camera results.");
                } else {
                    int activePipeline = result.getPipelineIndex();
                    telemetry.addData("Pipeline", activePipeline);
                    telemetry.addData("Result valid", result.isValid());

                    if (activePipeline != APRILTAG_PIPELINE) {
                        telemetry.addLine("Waiting for pipeline 0; set it to AprilTag 36h11.");
                    } else if (!result.isValid()) {
                        telemetry.addLine("No valid camera result yet.");
                    } else {
                        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                        if (tags == null || tags.isEmpty()) {
                            telemetry.addLine("No tags found. Check tag visibility and pipeline 0 settings.");
                        } else {
                            telemetry.addData("36h11 AprilTags", tags.size());
                            for (LLResultTypes.FiducialResult tag : tags) {
                                telemetry.addData("Tag ID " + tag.getFiducialId(),
                                        "horizontal %.1f° | vertical %.1f°",
                                        tag.getTargetXDegrees(), tag.getTargetYDegrees());
                            }
                        }
                    }
                }

                telemetry.update();
                sleep(50);
            }
        } finally {
            limelight.stop();
        }
    }
}
