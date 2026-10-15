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
    private DcMotor shooter2;

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
    private boolean shooter2Running = false;

    // For intake toggle buttons
    private boolean previousA = false;
    private boolean previousB = false;

    // Shooter powers, each adjustable independently
    private double shooterSpeed = Constants.SHOOTER_1_POWER;
    private double shooter2Speed = Constants.SHOOTER_2_POWER;

    // For the shooter on/off toggles
    private boolean previousGamepad2A = false;
    private boolean previousGamepad2B = false;

    // For the shoulder buttons and triggers used as speed steps
    private boolean previousLeftBumper = false;
    private boolean previousLeftTrigger = false;
    private boolean previousRightBumper = false;
    private boolean previousRightTrigger = false;

    // Powers before the most recent change, for tuning telemetry
    private double previousShooterSpeed = Constants.SHOOTER_1_POWER;
    private double previousShooter2Speed = Constants.SHOOTER_2_POWER;

    @Override
    public void runOpMode() {

        // =========================================================
        // HARDWARE MAP
        // =========================================================

        drive = new MecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);

        shooter = hardwareMap.get(DcMotor.class, Constants.SHOOTER_MOTOR);

        shooter2 = hardwareMap.get(DcMotor.class, Constants.SHOOTER_2_MOTOR);

        rightIntakeServo = hardwareMap.get(CRServo.class, Constants.RIGHT_INTAKE_SERVO);

        leftIntakeServo = hardwareMap.get(CRServo.class, Constants.LEFT_INTAKE_SERVO);

        shooterServo = hardwareMap.get(CRServo.class, Constants.SHOOTER_SERVO);


        // =========================================================
        // MOTOR DIRECTIONS
        // =========================================================

        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.FORWARD);


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

        shooter2.setZeroPowerBehavior(
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
        telemetry.addLine("A = Shooter 1 ON/OFF");
        telemetry.addLine("B = Shooter 2 ON/OFF");
        telemetry.addLine("LB = Shooter 1 Power Up");
        telemetry.addLine("LT = Shooter 1 Power Down");
        telemetry.addLine("RB = Shooter 2 Power Up");
        telemetry.addLine("RT = Shooter 2 Power Down");
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

            boolean currentGamepad2A = gamepad2.a;
            boolean currentGamepad2B = gamepad2.b;

            // Triggers are analog, so treat them as pressed past a threshold
            boolean currentLeftBumper = gamepad2.left_bumper;
            boolean currentLeftTrigger =
                    gamepad2.left_trigger > Constants.TRIGGER_THRESHOLD;
            boolean currentRightBumper = gamepad2.right_bumper;
            boolean currentRightTrigger =
                    gamepad2.right_trigger > Constants.TRIGGER_THRESHOLD;


            // A = shooter 1 on/off toggle
            if (currentGamepad2A && !previousGamepad2A) {
                shooterRunning = !shooterRunning;
            }

            // B = shooter 2 on/off toggle
            if (currentGamepad2B && !previousGamepad2B) {
                shooter2Running = !shooter2Running;
            }


            // LB = shooter 1 power up
            if (currentLeftBumper && !previousLeftBumper) {
                previousShooterSpeed = shooterSpeed;
                shooterSpeed = clampShooterPower(
                        shooterSpeed + Constants.SHOOTER_POWER_INCREMENT);
            }

            // LT = shooter 1 power down
            if (currentLeftTrigger && !previousLeftTrigger) {
                previousShooterSpeed = shooterSpeed;
                shooterSpeed = clampShooterPower(
                        shooterSpeed - Constants.SHOOTER_POWER_INCREMENT);
            }

            // RB = shooter 2 power up
            if (currentRightBumper && !previousRightBumper) {
                previousShooter2Speed = shooter2Speed;
                shooter2Speed = clampShooterPower(
                        shooter2Speed + Constants.SHOOTER_POWER_INCREMENT);
            }

            // RT = shooter 2 power down
            if (currentRightTrigger && !previousRightTrigger) {
                previousShooter2Speed = shooter2Speed;
                shooter2Speed = clampShooterPower(
                        shooter2Speed - Constants.SHOOTER_POWER_INCREMENT);
            }


            previousGamepad2A = currentGamepad2A;
            previousGamepad2B = currentGamepad2B;
            previousLeftBumper = currentLeftBumper;
            previousLeftTrigger = currentLeftTrigger;
            previousRightBumper = currentRightBumper;
            previousRightTrigger = currentRightTrigger;


            if (shooterRunning) {

                shooter.setPower(shooterSpeed);

            } else {

                shooter.setPower(0);
            }

            if (shooter2Running) {

                shooter2.setPower(shooter2Speed);

            } else {

                shooter2.setPower(0);
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


            telemetry.addData(
                    "Shooter 1",
                    "%s @ %.0f%%",
                    shooterRunning ? "ON" : "OFF",
                    shooterSpeed * 100);

            // Previous -> current, so a power that works can be written down
            telemetry.addData(
                    "Shooter 1 Power",
                    "%.2f -> %.2f",
                    previousShooterSpeed,
                    shooterSpeed);

            telemetry.addData(
                    "Shooter 2",
                    "%s @ %.0f%%",
                    shooter2Running ? "ON" : "OFF",
                    shooter2Speed * 100);

            telemetry.addData(
                    "Shooter 2 Power",
                    "%.2f -> %.2f",
                    previousShooter2Speed,
                    shooter2Speed);


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
        shooter2.setPower(0);

        rightIntakeServo.setPower(0);
        leftIntakeServo.setPower(0);
        shooterServo.setPower(0);
    }

    /**
     * Keeps an adjustable shooter power inside the configured bounds.
     * Shooter 1 runs negative, so the range spans both signs.
     */
    private double clampShooterPower(double power) {

        if (power < Constants.SHOOTER_POWER_MIN) {
            return Constants.SHOOTER_POWER_MIN;
        }

        if (power > Constants.SHOOTER_POWER_MAX) {
            return Constants.SHOOTER_POWER_MAX;
        }

        return power;
    }
}
