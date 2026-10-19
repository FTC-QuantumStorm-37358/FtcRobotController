package org.firstinspires.ftc.teamcode.quantumstorm.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;

@Autonomous(name = "Auto Test", group = "Tests")
public class AutoTest extends LinearOpMode {

    private static final double DRIVE_POWER = 0.5;
    private static final long DRIVE_DURATION_MS = 2000;
    private static final long STOP_DURATION_MS = 500;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        try {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            waitForStart();

            if (!opModeIsActive()) {
                return;
            }

            telemetry.addData("Action", "Driving forward");
            telemetry.update();
            drive.drive(DRIVE_POWER, 0, 0);
            sleep(DRIVE_DURATION_MS);
            if (!opModeIsActive()) {
                return;
            }

            telemetry.addData("Action", "Stopping briefly");
            telemetry.update();
            drive.stop();
            sleep(STOP_DURATION_MS);
            if (!opModeIsActive()) {
                return;
            }

            telemetry.addData("Action", "Driving backward");
            telemetry.update();
            drive.drive(-DRIVE_POWER, 0, 0);
            sleep(DRIVE_DURATION_MS);
        } finally {
            drive.stop();
        }
    }
}