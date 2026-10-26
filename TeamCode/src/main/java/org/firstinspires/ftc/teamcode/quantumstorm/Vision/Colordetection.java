package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

/** Searches the blue Limelight color pipeline and displays when blue is detected. */
@Autonomous(name = "Search for Nectar and Pollen", group = "Vision")
public class Colordetection extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight 3a";
    private static final int BLUE_PIPELINE = 1;

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
        telemetry.addLine("Configure pipeline 1 to detect blue only.");
        telemetry.addLine("The OpMode searches pipeline 1 for blue targets.");
        telemetry.update();
        waitForStart();

        try {
            boolean blueSeen = false;
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.getPipelineIndex() == BLUE_PIPELINE) {
                    List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                    blueSeen = colorTargets != null && !colorTargets.isEmpty();
                }

                telemetry.clearAll();
                if (blueSeen) {
                    telemetry.addLine("Blue");
                }
                telemetry.update();
                sleep(50);
            }
        } finally {
            limelight.stop();
        }
    }

}


 