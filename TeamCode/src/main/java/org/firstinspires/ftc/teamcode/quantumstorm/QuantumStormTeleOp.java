package org.firstinspires.ftc.teamcode.quantumstorm;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;

@TeleOp(name = "Quantum Storm Two Driver1", group = "Quantum Storm")
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

    // Adjustable shooter power
    private double shooterSpeed = Constants.SHOOTER_1_POWER;

    // Track even during controller assignment so releasing Start cannot start a shooter.
    private boolean previousGamepad2A = false;

    // For the shoulder buttons and triggers used as speed steps
    private boolean previousLeftBumper = false;
    private boolean previousLeftTrigger = false;

    // Powers before the most recent change, for tuning telemetry
    private double previousShooterSpeed = Constants.SHOOTER_1_POWER;

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

        // D-pad UP = feed; DOWN = reverse feed, matching autonomous.
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
        telemetry.addLine("A = Shooter 1 START, B = Shooter 1 STOP");
        telemetry.addLine("Hold START + tap LB = Shooter 1 Power Up");
        telemetry.addLine("Hold START + tap LT = Shooter 1 Power Down");
        telemetry.addLine("Release START before using shooter START/STOP buttons");
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

            // Triggers are analog, so treat them as pressed past a threshold
            boolean currentLeftBumper = gamepad2.left_bumper;
            boolean currentLeftTrigger =
                    gamepad2.left_trigger > Constants.TRIGGER_THRESHOLD;


            // Start+A/B assigns controllers in the Driver Station, not shooter commands.
            // Require a fresh A press after assignment; STOP wins if both are pressed.
            if (!gamepad2.start) {
                if (gamepad2.a && !previousGamepad2A) shooterRunning = true;
                if (gamepad2.b) shooterRunning = false;
            }


            // Require Start already held when a shoulder button is freshly pressed.
            // Track raw buttons below even while locked: holding LB then pressing Start
            // must not accidentally adjust power. Opposite requests cancel each other.
            if (gamepad2.start && currentLeftBumper && !previousLeftBumper && !currentLeftTrigger) {
                previousShooterSpeed = shooterSpeed;
                shooterSpeed = clampShooterPower(
                        shooterSpeed + Constants.SHOOTER_POWER_INCREMENT);
            }

            // Start + LT = shooter 1 power down
            if (gamepad2.start && currentLeftTrigger && !previousLeftTrigger && !currentLeftBumper) {
                previousShooterSpeed = shooterSpeed;
                shooterSpeed = clampShooterPower(
                        shooterSpeed - Constants.SHOOTER_POWER_INCREMENT);
            }


            previousGamepad2A = gamepad2.a;
            previousLeftBumper = currentLeftBumper;
            previousLeftTrigger = currentLeftTrigger;


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
            // Servo direction is configured above to match
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
            telemetry.addData("Power adjustment", gamepad2.start ? "UNLOCKED: tap LB/LT" : "LOCKED: hold START");


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

    /**
     * Keeps an adjustable shooter power inside the configured bounds.
     * The range spans both signs so tuning can reverse the motor if needed.
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
