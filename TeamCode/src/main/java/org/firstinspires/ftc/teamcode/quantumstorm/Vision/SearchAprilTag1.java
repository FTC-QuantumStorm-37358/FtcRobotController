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
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Limelight ready; pipeline 0 must be configured for AprilTag 36h11.");
        telemetry.addLine("Press Start to report visible tag IDs.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                List<LLResultTypes.FiducialResult> tags = result != null
                        && result.isValid()
                        && result.getPipelineIndex() == APRILTAG_PIPELINE
                        ? result.getFiducialResults()
                        : null;

                if (tags == null || tags.isEmpty()) {
                    telemetry.addLine("No 36h11 AprilTags detected.");
                } else {
                    telemetry.addData("36h11 AprilTags", tags.size());
                    for (LLResultTypes.FiducialResult tag : tags) {
                        telemetry.addData("Tag ID " + tag.getFiducialId(),
                                "horizontal %.1f° | vertical %.1f°",
                                tag.getTargetXDegrees(), tag.getTargetYDegrees());
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
