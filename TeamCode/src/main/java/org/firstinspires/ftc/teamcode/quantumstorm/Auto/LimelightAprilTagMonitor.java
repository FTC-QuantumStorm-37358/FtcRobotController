package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Limelight: AprilTag ID Monitor", group = "Vision")
public class LimelightAprilTagMonitor extends LinearOpMode {

    @Override
    public void runOpMode() {
        LimeServo camera = new LimeServo(hardwareMap);
        camera.selectPipeline(LimeServo.Pipeline.APRIL_TAG);

        telemetry.addData("Limelight", camera.isConnected() ? "Connected" : "Waiting for connection");
        telemetry.addLine("AprilTag pipeline selected; press PLAY to monitor tag IDs.");
        telemetry.update();

        try {
            waitForStart();
            while (opModeIsActive()) {
                List<Integer> tagIds = camera.getVisibleAprilTagIds();
                telemetry.clearAll();
                telemetry.addData("Limelight", camera.isConnected() ? "Connected" : "Disconnected");
                telemetry.addData("Pipeline", "AprilTag");
                telemetry.addData("AprilTags detected", tagIds.size());
                telemetry.addData("Visible tag IDs", tagIds.isEmpty() ? "None" : tagIds);
                telemetry.update();
                sleep(50);
            }
        } finally {
            camera.stop();
        }
    }
}
