package org.firstinspires.ftc.teamcode.quantumstorm;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.FlowerIntake;

@TeleOp(name = Constants.TELEOP_NAME, group = "Quantum Storm")
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
    private final org.firstinspires.ftc.teamcode.biobuzz.FeedCadence feedCadence=new org.firstinspires.ftc.teamcode.biobuzz.FeedCadence(Constants.SPINDEXER_FEED_PULSE_SEC,Constants.SPINDEXER_SHOT_GAP_SEC);

    // Flower intake arm (position servo)
    private FlowerIntake flowerIntake;

    // Intake:
    //  1 = forward
    //  0 = stopped
    // -1 = reverse
    private int intakeState = 0;

    private boolean shooterRunning = false;

    // Require a fresh A press for each intake direction command.
    private boolean previousA = false;

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

        flowerIntake = new FlowerIntake(hardwareMap);


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

        // D-pad UP = spindexer forward; DOWN = spindexer reverse, matching autonomous.
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

        // The flower arm moves to UP as soon as INIT is pressed
        flowerIntake.lift();


        telemetry.addLine("Quantum Storm Two Driver Ready");
        telemetry.addLine("");

        telemetry.addLine("GAMEPAD 1 - DRIVE + INTAKE");
        telemetry.addLine("Left Stick = Drive / Strafe");
        telemetry.addLine("Right Stick X = Turn");
        telemetry.addLine("A = Intake Forward ON");
        telemetry.addLine("B = Intake OFF (also with START held)");
        telemetry.addLine("Hold START + tap A = Intake Reverse ON");
        telemetry.addLine("START + D-Pad DOWN = Flower Intake DROP");
        telemetry.addLine("START + D-Pad UP = Flower Intake LIFT");
        telemetry.addLine("LB + D-Pad UP/DOWN = Flower Angle Nudge");

        telemetry.addLine("");

        telemetry.addLine("GAMEPAD 2 - SHOOTER + SPINDEXER");
        telemetry.addLine("A = Shooter 1 START, B = Shooter 1 STOP");
        telemetry.addLine("Hold START + tap LB = Shooter 1 Power Up");
        telemetry.addLine("Hold START + tap LT = Shooter 1 Power Down");
        telemetry.addLine("Release START before using shooter START/STOP buttons");
        telemetry.addLine("Hold D-Pad UP = Spindexer Forward (pulsed)");
        telemetry.addLine("Hold D-Pad DOWN = Spindexer Reverse");
        telemetry.addLine("Release D-Pad = Spindexer STOP");

        telemetry.update();


        waitForStart();

        // Start+A assigns gamepad 1 in the Driver Station. A held through INIT/START
        // must be released before it can command the intake during TeleOp.
        previousA = gamepad1.a;


        // =========================================================
        // MAIN LOOP
        // =========================================================

        while (opModeIsActive()) {

            // =====================================================
            // GAMEPAD 1 - MECANUM DRIVE
            // =====================================================

            // This robot's tested forward/backward convention is inverted.
            // Correct only TeleOp's Y input; keep strafe, turn and autonomous unchanged.
            drive.drive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);


            // =====================================================
            // GAMEPAD 1 - INTAKE START / STOP
            // =====================================================

            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;


            // Fresh A selects forward; holding Start at that press selects reverse.
            // Repeated A presses do not toggle off. Releasing Start while A is held
            // does not change direction; holding A then pressing Start does not reverse.
            if (currentA && !previousA) {
                intakeState = gamepad1.start ? -1 : 1;
            }
            // STOP always wins, including A+B and Start+A+B.
            if (currentB) intakeState = 0;
            previousA = currentA;


            // =====================================================
            // INTAKE MOTOR + INTAKE SERVOS
            // =====================================================

            double intakeOutput =
                    intakeState * Constants.INTAKE_POWER;

            intake.setPower(intakeOutput);

            rightIntakeServo.setPower(intakeOutput);
            leftIntakeServo.setPower(intakeOutput);


            // =====================================================
            // GAMEPAD 1 - FLOWER INTAKE SERVO
            // =====================================================

            // START + D-pad UP/DOWN = lift/drop, LB + D-pad = nudge
            flowerIntake.handleGamepad(gamepad1);


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
            // GAMEPAD 2 - SPINDEXER CR SERVO
            // =====================================================
            //
            // UP = FORWARD
            // DOWN = REVERSE
            //
            // Servo direction is configured above to match
            // your actual physical mechanism.
            // =====================================================

            if (gamepad2.dpad_up) {

                shooterServo.setPower(feedCadence.apply(1,System.nanoTime()/1e9)*Constants.SHOOTER_SERVO_POWER);

            } else if (gamepad2.dpad_down) {

                shooterServo.setPower(feedCadence.apply(-1,System.nanoTime()/1e9)*Constants.SHOOTER_SERVO_POWER);

            } else {

                feedCadence.apply(0,System.nanoTime()/1e9);
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

            telemetry.addData(
                    "Flower Intake",
                    "%s  %.0f deg  (servo pos %.3f)",
                    flowerIntake.isDown() ? "DOWN" : "UP",
                    flowerIntake.getAngle(),
                    flowerIntake.getPosition());


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
                        "Spindexer",
                        "FORWARD");

            } else if (gamepad2.dpad_down) {

                telemetry.addData(
                        "Spindexer",
                        "REVERSE");

            } else {

                telemetry.addData(
                        "Spindexer",
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
        feedCadence.apply(0,System.nanoTime()/1e9);
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
