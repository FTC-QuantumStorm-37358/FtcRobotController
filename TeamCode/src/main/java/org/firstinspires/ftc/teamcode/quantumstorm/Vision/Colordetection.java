package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

/** Scans three single-color pipelines and displays the last color actually detected. */
@Autonomous(name = "Search for Nectar and Pollen", group = "Vision")
public class Colordetection extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight 3a";
    //pipeline 1 = blue, 2 = red, 3 = yellow choose between these to change code for auto
    private static final int BLUE_PIPELINE = 1;
    private static final int RED_PIPELINE = 2;
    private static final int YELLOW_PIPELINE = 3;
    private static final int[] COLOR_PIPELINES = {
            BLUE_PIPELINE, RED_PIPELINE, YELLOW_PIPELINE
    };
    private static final long PIPELINE_SCAN_INTERVAL_MS = 750;

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

        limelight.pipelineSwitch(BLUE_PIPELINE);
        limelight.start();

        telemetry.addData("Limelight", limelight.isConnected()
                ? "Connected" : "Waiting for connection");
        telemetry.addLine("Configure pipeline 1 for blue, 2 for red, and 3 for yellow only.");
        telemetry.addLine("Shows the last detected color, even while scanning other pipelines.");
        telemetry.update();
        waitForStart();

        try {
            int pipelinePosition = 0;
            int activePipeline = COLOR_PIPELINES[pipelinePosition];
            String detectedColor = null;
            long nextPipelineSwitch = System.currentTimeMillis() + PIPELINE_SCAN_INTERVAL_MS;

            while (opModeIsActive()) {
                long now = System.currentTimeMillis();
                if (now >= nextPipelineSwitch) {
                    pipelinePosition = (pipelinePosition + 1) % COLOR_PIPELINES.length;
                    activePipeline = COLOR_PIPELINES[pipelinePosition];
                    limelight.pipelineSwitch(activePipeline);
                    nextPipelineSwitch = now + PIPELINE_SCAN_INTERVAL_MS;
                }

                LLResult result = limelight.getLatestResult();
                if (limelight.isConnected() && result != null && result.isValid()
                        && result.getPipelineIndex() == activePipeline) {
                    List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                    if (colorTargets != null && !colorTargets.isEmpty()) {
                        detectedColor = colorNameForPipeline(activePipeline);
                    }
                }

                telemetry.clearAll();
                if (detectedColor != null) {
                    telemetry.addLine(detectedColor);
                }
                telemetry.update();
                sleep(50);
            }
        } finally {
            limelight.stop();
        }
    }

    private String colorNameForPipeline(int pipeline) {
        if (pipeline == BLUE_PIPELINE) {
            return "Blue";
        }
        if (pipeline == RED_PIPELINE) {
            return "Red";
        }
        return "Yellow";
    }

}


 