package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

/** Reports targets returned by Limelight color-threshold pipeline 1. */
@Autonomous(name = "Search for Nectar and Pollen", group = "Vision")
public class Colordetection extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight 3a";
    private static final int COLOR_PIPELINE = 1;
    private static final boolean BLUE = true;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, LIMELIGHT_NAME);
        if (limelight == null) {
            telemetry.addData("Limelight", "Not found; check Robot Configuration name '%s'.",
                    LIMELIGHT_NAME);
            telemetry.update();
            waitForStart();
            return;
        }

        limelight.pipelineSwitch(COLOR_PIPELINE);
        limelight.start();

        telemetry.addData("Limelight", "Waiting for %s color-threshold pipeline %d",
                limelight.isConnected() ? "connected" : "connection", COLOR_PIPELINE);
        telemetry.addData("Selected nectar color", BLUE ? "Blue" : "Red");
        telemetry.addLine("Pipeline 1 must be configured for the selected color.");
        telemetry.addLine("A single color-threshold pipeline cannot distinguish yellow pollen.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                List<LLResultTypes.ColorResult> colorTargets = null;

                telemetry.addData("Limelight connection", limelight.isConnected()
                        ? "Connected" : "Not connected");
                if (result == null) {
                    telemetry.addLine("Waiting for camera results.");
                } else if (result.getPipelineIndex() != COLOR_PIPELINE) {
                    telemetry.addData("Pipeline", "Waiting for pipeline %d",
                            COLOR_PIPELINE);
                } else if (!result.isValid()) {
                    telemetry.addLine("No valid camera result yet.");
                } else {
                    colorTargets = result.getColorResults();
                    int targetCount = colorTargets == null ? 0 : colorTargets.size();
                    telemetry.addData("Configured color target", targetCount > 0
                            ? "Nectar seen" : "Searching for " + (BLUE ? "blue" : "red"));
                    telemetry.addData("Color targets", targetCount);
                }
                telemetry.addData("Pollen", "Not separately detectable with this pipeline");
                telemetry.update();
                sleep(50);
            }
        } finally {
            limelight.stop();
        }
    }
}


 