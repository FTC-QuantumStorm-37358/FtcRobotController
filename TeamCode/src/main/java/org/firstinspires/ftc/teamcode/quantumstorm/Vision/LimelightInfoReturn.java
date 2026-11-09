package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

import java.util.List;

@TeleOp(name = "Limelight: AprilTag tv", group = "Vision")
public class LimelightInfoReturn extends LinearOpMode {

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT);
        if (limelight == null) {
            telemetry.addLine("Limelight not found in Robot Configuration.");
            telemetry.update();
            return;
        }

        limelight.pipelineSwitch(Constants.LIMELIGHT_APRILTAG_PIPELINE);
        limelight.start();
        telemetry.addLine("AprilTag pipeline selected. Press PLAY to monitor tv.");
        telemetry.update();

        try {
            waitForStart();
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                boolean seesAprilTag = false;
                int visibleTagCount = 0;

                if (limelight.isConnected() && result != null && result.isValid()
                        && result.getPipelineIndex() == Constants.LIMELIGHT_APRILTAG_PIPELINE) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    visibleTagCount = tags == null ? 0 : tags.size();
                    seesAprilTag = visibleTagCount > 0;
                }

                telemetry.addData("Limelight", limelight.isConnected() ? "Connected" : "Disconnected");
                telemetry.addData("Pipeline", Constants.LIMELIGHT_APRILTAG_PIPELINE + " (AprilTag)");
                telemetry.addData("tv", seesAprilTag ? 1 : 0);
                telemetry.addData("AprilTags detected", visibleTagCount);
                telemetry.update();
                sleep(50);
            }
        } finally {
            limelight.stop();
        }
    }
}
