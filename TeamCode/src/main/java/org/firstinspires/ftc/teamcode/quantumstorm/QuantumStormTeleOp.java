package org.firstinspires.ftc.teamcode.quantumstorm;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;

@TeleOp(name = "Quantum Storm Two Driver", group = "Quantum Storm")
public class QuantumStormTeleOp extends LinearOpMode {

    // Drive chain
    private MecanumDrive drive;

    // Mechanism motors
    private DcMotor intake;
    private DcMotor shooter;

    // CR servos
    private CRServo rightIntakeServo;
    private CRServo leftIntakeServo;
    private CRServo shooterServo;

    // Intake:
    //  1 = forward
    //  0 = stopped
    // -1 = reverse
    private int intakeState = 0;

    private boolean shooterRunning = false;

    // For intake toggle buttons
    private boolean previousA = false;
    private boolean previousB = false;

    // For shooter speed
    private double shooterSpeed = Constants.SHOOTER_POWER;

    // For shooter speed buttons
    private boolean previousGamepad2A = false;
    private boolean previousGamepad2B = false;

    @Override
    public void runOpMode() {

        // =========================================================
        // HARDWARE MAP
        // =========================================================

        drive = new MecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);

        shooter = hardwareMap.get(DcMotor.class, Constants.SHOOTER_MOTOR);

        rightIntakeServo = hardwareMap.get(CRServo.class, Constants.RIGHT_INTAKE_SERVO);

        leftIntakeServo = hardwareMap.get(CRServo.class, Constants.LEFT_INTAKE_SERVO);

        shooterServo = hardwareMap.get(CRServo.class, Constants.SHOOTER_SERVO);


        // =========================================================
        // MOTOR DIRECTIONS
        // =========================================================

        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);


        // =========================================================
        // SERVO DIRECTIONS
        // =========================================================

        rightIntakeServo.setDirection(CRServo.Direction.FORWARD);
        leftIntakeServo.setDirection(CRServo.Direction.REVERSE);

        // FLIPPED so:
        // D-pad UP = shooter forward
        // D-pad DOWN = shooter reverse
        shooterServo.setDirection(CRServo.Direction.REVERSE);


        // =========================================================
        // ZERO POWER BEHAVIOR
        // =========================================================

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);


        // =========================================================
        // START EVERYTHING STOPPED
        // =========================================================

        stopAll();


        telemetry.addLine("Quantum Storm Two Driver Ready");
        telemetry.addLine("");

        telemetry.addLine("GAMEPAD 1 - DRIVE + INTAKE");
        telemetry.addLine("Left Stick = Drive / Strafe");
        telemetry.addLine("Right Stick X = Turn");
        telemetry.addLine("A = Intake Forward Toggle");
        telemetry.addLine("B = Intake Reverse Toggle");

        telemetry.addLine("");

        telemetry.addLine("GAMEPAD 2 - SHOOTER");
        telemetry.addLine("A = Shooter Speed Down");
        telemetry.addLine("B = Shooter Speed Up");
        telemetry.addLine("Y = Shooter ON");
        telemetry.addLine("X = Shooter OFF");
        telemetry.addLine("D-Pad UP = Shooter Forward");
        telemetry.addLine("D-Pad DOWN = Shooter Reverse");

        telemetry.update();


        waitForStart();


        // =========================================================
        // MAIN LOOP
        // =========================================================

        while (opModeIsActive()) {

            // =====================================================
            // GAMEPAD 1 - MECANUM DRIVE
            // =====================================================

            drive.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);


            // =====================================================
            // GAMEPAD 1 - INTAKE TOGGLE
            // =====================================================

            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;


            // A = forward toggle
            if (currentA && !previousA) {

                if (intakeState == 1) {

                    intakeState = 0;

                } else {

                    intakeState = 1;
                }
            }


            // B = reverse toggle
            if (currentB && !previousB) {

                if (intakeState == -1) {

                    intakeState = 0;

                } else {

                    intakeState = -1;
                }
            }


            previousA = currentA;
            previousB = currentB;


            // =====================================================
            // INTAKE MOTOR + INTAKE SERVOS
            // =====================================================

            double intakeOutput =
                    intakeState * Constants.INTAKE_POWER;

            intake.setPower(intakeOutput);

            rightIntakeServo.setPower(intakeOutput);
            leftIntakeServo.setPower(intakeOutput);


            // =====================================================
            // GAMEPAD 2 - SHOOTER MOTOR
            // =====================================================

            // Y = shooter ON
            if (gamepad2.y) {
                shooterRunning = true;
            }

            // X = shooter OFF
            if (gamepad2.x) {
                shooterRunning = false;
            }

            boolean currentGamepad2A = gamepad2.a;
            boolean currentGamepad2B = gamepad2.b;


            // A = shooter speed down
            if (currentGamepad2A && !previousGamepad2A) {
                shooterSpeed = shooterSpeed - 0.1;
                if (shooterSpeed < -1.0) {
                    shooterSpeed = -1.0;
                }
            }

            // B = shooter speed up
            if (currentGamepad2B && !previousGamepad2B) {
                shooterSpeed = shooterSpeed + 0.1;
                if (shooterSpeed > 1.0) {
                    shooterSpeed = 1.0;
                }
            }


            previousGamepad2A = currentGamepad2A;
            previousGamepad2B = currentGamepad2B;



            if (shooterRunning) {

                shooter.setPower(shooterSpeed);

            } else {

                shooter.setPower(0);
            }


            // =====================================================
            // GAMEPAD 2 - SHOOTER CR SERVO
            // =====================================================
            //
            // UP = FORWARD
            // DOWN = REVERSE
            //
            // Servo direction is reversed above to match
            // your actual physical mechanism.
            // =====================================================

            if (gamepad2.dpad_up) {

                shooterServo.setPower(Constants.SHOOTER_SERVO_POWER);

            } else if (gamepad2.dpad_down) {

                shooterServo.setPower(-Constants.SHOOTER_SERVO_POWER);

            } else {

                shooterServo.setPower(0);
            }


            // =====================================================
            // TELEMETRY
            // =====================================================

            telemetry.addLine("=== DRIVER 1 ===");

            drive.addTelemetry(telemetry);


            if (intakeState == 1) {

                telemetry.addData(
                        "Intake",
                        "FORWARD");

            } else if (intakeState == -1) {

                telemetry.addData(
                        "Intake",
                        "REVERSE");

            } else {

                telemetry.addData(
                        "Intake",
                        "STOPPED");
            }


            telemetry.addLine("");
            telemetry.addLine("=== DRIVER 2 ===");


            if (shooterRunning) {

                telemetry.addData(
                        "Shooter",
                        "RUNNING @ %.0f%%",
                        shooterSpeed * 100);

            } else {

                telemetry.addData(
                        "Shooter",
                        "STOPPED");
            }


            if (gamepad2.dpad_up) {

                telemetry.addData(
                        "Shooter Servo",
                        "FORWARD");

            } else if (gamepad2.dpad_down) {

                telemetry.addData(
                        "Shooter Servo",
                        "REVERSE");

            } else {

                telemetry.addData(
                        "Shooter Servo",
                        "STOPPED");
            }


            telemetry.update();
        }


        // =========================================================
        // STOP EVERYTHING
        // =========================================================

        stopAll();
    }

    private void stopAll() {

        drive.stop();

        intake.setPower(0);
        shooter.setPower(0);

        rightIntakeServo.setPower(0);
        leftIntakeServo.setPower(0);
        shooterServo.setPower(0);
    }
}
