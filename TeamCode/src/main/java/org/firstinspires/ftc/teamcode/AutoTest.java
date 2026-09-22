package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTest extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        // Initialize motors based on your robot configuration
        leftDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");

        // Reverse one side so both wheels spin forward together
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the DS press start
        waitForStart();

        // 1. Drive Forward at 0.5 power for 2 seconds
        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);
        sleep(1000);

        // 2. Stop for 0.5 seconds
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        sleep(500);

        // 3. Drive Backward at 0.5 power for 2 seconds
        leftDrive.setPower(-0.5);
        rightDrive.setPower(-0.5);
        sleep(2000);

        // 4. Stop the robot
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }
}