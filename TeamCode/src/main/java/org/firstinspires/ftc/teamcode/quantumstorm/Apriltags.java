package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name = "Detect AprilTags Only")
public class Apriltags extends LinearOpMode {
    private static final int APRILTAG_PIPELINE = 0; // Change to your AprilTag pipeline number.

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("AprilTag detector ready. Press Start.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                telemetry.addLine("No valid camera result");
            } else {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                if (tags == null || tags.isEmpty()) {
                    telemetry.addLine("No AprilTags detected");
                } else {
                    telemetry.addData("Tags found", tags.size());

                    for (LLResultTypes.FiducialResult tag : tags) {
                        telemetry.addData(
                                "Tag ID " + tag.getFiducialId(),
                                "Family: %s | Left/right: %.1f° | Up/down: %.1f°",
                                tag.getFamily(),
                                tag.getTargetXDegrees(),
                                tag.getTargetYDegrees()
                        );
                    }
                }
            }

            telemetry.update();
            sleep(20);
        }

        limelight.stop();
    }
}