package org.firstinspires.ftc.teamcode.quantumstorm;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.FlowerIntake;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.ShooterVelocityControl;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.SpindexerControls;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration.Solution;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = Constants.TELEOP_NAME, group = "Quantum Storm")
public class QuantumStormTeleOp extends LinearOpMode {

    // Drive chain
    private MecanumDrive drive;

    // Mechanism motors
    private DcMotor intake;
    private DcMotorEx shooter;

    // CR servos
    private CRServo shooterServo;
    private final SpindexerControls spindexerControls = new SpindexerControls();
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

    // Signed fraction of the configured shooter base RPM.
    private double shooterSetting;

    // Track even during controller assignment so releasing Start cannot start a shooter.
    private boolean previousGamepad2A = false;

    // For the shoulder buttons and triggers used as speed steps
    private boolean previousLeftBumper = false;
    private boolean previousLeftTrigger = false;

    // Setting before the most recent change, for tuning telemetry.
    private double previousShooterSetting;

    // Standard TeleOp uses the same rear-shooter distance/RPM table as autonomous.
    private final TableShooterCalibration distanceShooter=new TableShooterCalibration();
    private PinpointOdometry shooterOdometry;
    private Quadrant shooterStart=Quadrant.SOUTHWEST;
    private Solution distanceShot;

    protected void initializeShooterTarget() {
        shooterOdometry=new PinpointOdometry(hardwareMap,shooterStartPose());
    }
    private Pose2D shooterStartPose(){
        return new Pose2D(DistanceUnit.INCH,shooterStart.startX,shooterStart.startY,
                AngleUnit.DEGREES,shooterStart.startHeadingDegrees);
    }
    protected void waitForDriverStart() {
        while(!isStarted()&&!isStopRequested()){
            double x=shooterStart.startX,y=shooterStart.startY;
            if(gamepad1.dpad_left!=gamepad1.dpad_right)x=gamepad1.dpad_left?-12.75:12.75;
            if(gamepad1.dpad_down!=gamepad1.dpad_up)y=gamepad1.dpad_down?-62.9:62.9;
            Quadrant selected=Quadrant.at(x,y);
            if(selected!=shooterStart){shooterStart=selected;shooterOdometry.setPose(shooterStartPose());}
            updateShooterTarget(0);
            telemetry.addLine("INIT: D-pad LEFT/RIGHT selects field X; DOWN/UP selects field Y.");
            telemetry.addData("Shooter start reference",shooterStart);
            telemetry.addLine("Keep the robot still while Pinpoint initializes. Shooter and feeder stay OFF.");
            addShooterTargetTelemetry();telemetry.update();idle();
        }
    }
    protected double updateShooterTarget(double currentSetting) {
        distanceShot=null;
        if(shooterOdometry==null)return 0;
        shooterOdometry.update();
        if(!shooterOdometry.isValid())return 0;
        distanceShot=distanceShooter.atPose(shooterOdometry.getX(),shooterOdometry.getY(),shooterOdometry.getHeading());
        return shooterTargetReady()?distanceShot.rpm/Constants.SHOOTER_BASE_RPM:0;
    }
    protected boolean manualShooterAdjustment() { return false; }
    protected boolean feederRequiresShooterRunning() { return true; }
    protected boolean feedingAllowed() { return true; }
    protected boolean shooterStartAllowed() { return true; }
    protected void updateDriverAssistance(boolean freshShooterA) { }
    protected void driveRobot(MecanumDrive drive) {
        // This robot's tested forward/backward convention is inverted; strafe/turn retain their signs.
        drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
    protected boolean shooterTargetReady() { return distanceShot!=null&&distanceShot.ready(); }
    protected void addShooterTargetTelemetry() {
        telemetry.addLine("AUTO SHOOTER: Pinpoint distance selects RPM; A starts, B stops, D-pad feeds.");
        if(distanceShot!=null&&distanceShot.quadrant!=null){
            telemetry.addData("Shooter-to-Hive distance (in)","%.1f",distanceShot.distanceInches);
            telemetry.addData("Auto shooter RPM / multiplier","%.0f / %.3f",distanceShot.rpm,distanceShot.rpm/Constants.SHOOTER_BASE_RPM);
            telemetry.addData("Shot solution",distanceShot.status);
        }else telemetry.addData("Shot solution","WAITING FOR VALID PINPOINT POSE");
        if(shooterOdometry!=null)shooterOdometry.addTelemetry(telemetry);
    }
    protected double initialShooterSetting() { return Constants.SHOOTER_RPM_MULTIPLIER; }
    protected String shooterControlLabel() { return "Automatic Pinpoint distance / RPM table"; }
    protected void configureShooterMotor(DcMotorEx motor) { ShooterVelocityControl.configure(motor); }
    protected boolean applyShooterOutput(DcMotorEx motor, double multiplier) {
        ShooterVelocityControl.setOutput(motor, multiplier);
        return true;
    }

    @Override
    public void runOpMode() {
        shooterSetting = initialShooterSetting();
        previousShooterSetting = shooterSetting;

        // =========================================================
        // HARDWARE MAP
        // =========================================================

        drive = new MecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);

        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR);

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

        // D-pad UP = spindexer forward; DOWN = spindexer reverse, matching autonomous.
        shooterServo.setDirection(CRServo.Direction.REVERSE);


        // =========================================================
        // ZERO POWER BEHAVIOR
        // =========================================================

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);
        configureShooterMotor(shooter);


        // =========================================================
        // START EVERYTHING STOPPED
        // =========================================================

        stopAll();

        // The flower arm moves to UP as soon as INIT is pressed
        flowerIntake.lift();

        try {
            initializeShooterTarget();
            runDriverControls();
        } finally {
            stopAll();
        }
    }

    private void runDriverControls() {
        telemetry.addLine("Quantum Storm Two Driver Ready");
        telemetry.addData("Shooter control", shooterControlLabel());
        telemetry.addData("Shooter encoder scale", "%.0f ticks/rev; %.1f:1; %.0f max RPM",
                shooter.getMotorType().getTicksPerRev(), shooter.getMotorType().getGearing(),
                shooter.getMotorType().getMaxRPM());
        telemetry.addLine("");

        telemetry.addLine("GAMEPAD 1 - DRIVE + INTAKE");
        telemetry.addLine("Left Stick = Drive / Strafe");
        telemetry.addLine("Right Stick X = Turn");
        telemetry.addLine("A (without BACK/START) = Intake Forward ON");
        telemetry.addLine("B = Intake OFF (also with START held)");
        telemetry.addLine("Hold BACK + tap A = Intake Reverse ON (release START)");
        telemetry.addLine("START + A = Assign gamepad 1; no intake direction command");

        telemetry.addLine("");

        telemetry.addLine("GAMEPAD 2 - SHOOTER + SPINDEXER");
        telemetry.addLine("A = Shooter 1 START, B = Shooter 1 STOP");
        telemetry.addLine(manualShooterAdjustment()
                ? "Hold BACK + tap LB/LT = Shooter RPM +60/-60"
                : "Shooter RPM follows distance automatically; LB/LT speed adjustment disabled");
        telemetry.addLine("Release START before using shooter START/STOP buttons");
        telemetry.addLine("Hold D-Pad UP = Spindexer Forward (pulsed)");
        telemetry.addLine("Hold D-Pad DOWN = Spindexer Reverse");
        telemetry.addLine("Release D-Pad = Spindexer STOP");
        telemetry.addLine("BACK + D-Pad DOWN = Flower Intake DROP (-135 deg)");
        telemetry.addLine("BACK + D-Pad UP = Flower Intake UP (0 deg)");
        telemetry.addLine("LB + D-Pad UP/DOWN = Flower Angle Nudge");
        telemetry.addLine("After flower controls, release D-Pad before running spindexer");

        addShooterTargetTelemetry();
        telemetry.update();


        waitForDriverStart();

        // A held through INIT/START must be released before it can command either motor.
        previousA = gamepad1.a;
        previousGamepad2A = gamepad2.a;


        // =========================================================
        // MAIN LOOP
        // =========================================================

        while (opModeIsActive()) {

            updateDriverAssistance(!gamepad2.start && gamepad2.a && !previousGamepad2A);
            shooterSetting = updateShooterTarget(shooterSetting);
            boolean targetReady = shooterTargetReady();
            if (!targetReady) shooterRunning = false;

            // =====================================================
            // GAMEPAD 1 - MECANUM DRIVE
            // =====================================================

            driveRobot(drive);


            // =====================================================
            // GAMEPAD 1 - INTAKE START / STOP
            // =====================================================

            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;


            // Fresh A selects forward; holding Back at that press selects reverse.
            // Start+A is reserved for Driver Station assignment. Track A even while
            // Start is held so releasing Start cannot issue an intake command.
            // Changing Back while A is held does not change direction.
            if (!gamepad1.start && currentA && !previousA) {
                intakeState = gamepad1.back ? -1 : 1;
            }
            // STOP always wins, including A+B, Back+A+B and Start+A+B.
            if (currentB) intakeState = 0;
            previousA = currentA;


            // =====================================================
            // INTAKE MOTOR
            // =====================================================

            double intakeOutput =
                    intakeState * Constants.INTAKE_POWER;

            intake.setPower(intakeOutput);


            // =====================================================
            // GAMEPAD 2 - FLOWER INTAKE SERVO
            // =====================================================

            // Gamepad 2: BACK + D-pad UP/DOWN = lift/drop, LB + D-pad = nudge
            flowerIntake.handleGamepad(gamepad2);

            // A flower-arm press stays reserved until the D-pad is released,
            // even if BACK/LB is released first.
            int spindexerDemand = spindexerControls.update(
                    gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.back, gamepad2.left_bumper);
            boolean spindexerDpadUp = spindexerDemand > 0;
            boolean spindexerDpadDown = spindexerDemand < 0;


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
                if (targetReady && shooterStartAllowed() && gamepad2.a && !previousGamepad2A) shooterRunning = true;
                if (gamepad2.b) shooterRunning = false;
            }


            // Require Back already held when a shoulder button is freshly pressed.
            // Track raw buttons below even while locked: holding LB then pressing Back
            // must not accidentally adjust the shooter setting. Opposite requests cancel each other.
            if (manualShooterAdjustment() && gamepad2.back && currentLeftBumper && !previousLeftBumper && !currentLeftTrigger) {
                previousShooterSetting = shooterSetting;
                shooterSetting = clampShooterSetting(shooterSetting + Constants.SHOOTER_RPM_MULTIPLIER_INCREMENT);
            }

            // Back + LT = shooter setting down
            if (manualShooterAdjustment() && gamepad2.back && currentLeftTrigger && !previousLeftTrigger && !currentLeftBumper) {
                previousShooterSetting = shooterSetting;
                shooterSetting = clampShooterSetting(shooterSetting - Constants.SHOOTER_RPM_MULTIPLIER_INCREMENT);
            }


            previousGamepad2A = gamepad2.a;
            previousLeftBumper = currentLeftBumper;
            previousLeftTrigger = currentLeftTrigger;


            if (!applyShooterOutput(shooter, shooterRunning ? shooterSetting : 0)) {
                shooterRunning = false;
                targetReady = false;
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

            if (!targetReady || !feedingAllowed()
                    || (feederRequiresShooterRunning() && (!shooterRunning || shooterSetting == 0))) {
                feedCadence.apply(0,System.nanoTime()/1e9);
                shooterServo.setPower(0);
                spindexerDpadUp = false;
                spindexerDpadDown = false;
            } else if (spindexerDpadUp) {

                shooterServo.setPower(feedCadence.apply(1,System.nanoTime()/1e9)*Constants.SHOOTER_SERVO_POWER);

            } else if (spindexerDpadDown) {

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

            telemetry.addLine("");
            telemetry.addLine("=== DRIVER 2 ===");
            telemetry.addData("Shooter adjustment", manualShooterAdjustment()
                    ? (gamepad2.back ? "UNLOCKED: tap LB/LT" : "LOCKED: hold BACK")
                    : "AUTOMATIC: distance calibration");


            telemetry.addData("Shooter control", shooterControlLabel());
            telemetry.addData(
                    "Shooter 1",
                    "%s | RPM multiplier %.2f",
                    shooterRunning ? "ON" : "OFF",
                    shooterSetting);

            telemetry.addData("Shooter RPM actual / target", "%.0f / %.0f",
                    shooter.getVelocity(AngleUnit.DEGREES) / 6.0,
                    shooterRunning ? Constants.SHOOTER_BASE_RPM * shooterSetting : 0);
            telemetry.addData("Shooter ticks/sec actual / target", "%.0f / %.0f",
                    shooter.getVelocity(), shooterRunning
                            ? Constants.SHOOTER_BASE_RPM * shooterSetting
                                    * shooter.getMotorType().getTicksPerRev() / 60.0 : 0);
            telemetry.addData("Shooter ticks/rev", "%.0f", shooter.getMotorType().getTicksPerRev());

            // Previous -> current setting for comparison between runs
            telemetry.addData(
                    "Shooter RPM multiplier",
                    "%.2f -> %.2f",
                    previousShooterSetting,
                    shooterSetting);


            telemetry.addData(
                    "Flower Intake",
                    "%s  %.0f deg  (servo pos %.3f)",
                    flowerIntake.isDown() ? "DOWN" : "UP",
                    flowerIntake.getAngle(),
                    flowerIntake.getPosition());


            if (spindexerDpadUp) {

                telemetry.addData(
                        "Spindexer",
                        "FORWARD");

            } else if (spindexerDpadDown) {

                telemetry.addData(
                        "Spindexer",
                        "REVERSE");

            } else {

                telemetry.addData(
                        "Spindexer",
                        "STOPPED");
            }


            addShooterTargetTelemetry();
            telemetry.update();
        }
    }

    private void stopAll() {

        drive.stop();

        intake.setPower(0);
        applyShooterOutput(shooter, 0);

        feedCadence.apply(0,System.nanoTime()/1e9);
        shooterServo.setPower(0);
    }

    /**
     * Keeps the adjustable RPM fraction inside [-1, 1].
     * The range spans both signs so tuning can reverse the motor if needed.
     */
    protected double clampShooterSetting(double setting) {

        if (setting < -1.0) {
            return -1.0;
        }

        if (setting > 1.0) {
            return 1.0;
        }

        // Keep button steps on the configured grid, including an exact zero target.
        double step = Constants.SHOOTER_RPM_MULTIPLIER_INCREMENT;
        return Math.round(setting / step) * step;
    }
}
