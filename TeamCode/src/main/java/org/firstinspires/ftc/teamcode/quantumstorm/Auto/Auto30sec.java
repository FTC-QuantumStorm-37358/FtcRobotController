package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.Navigator;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;

import java.util.List;

/** BioBuzz odometry navigation, Limelight pickup/aiming, intake, and timed feeding. */
@Autonomous(name = "Ryan's auto test", group = "BioBuzz")
public class Auto30sec extends LinearOpMode {
    // Options: "red 1 fire red", "blue 1 fire blue", "red 2 no fire", "blue 2 no fire".
    private static final String Team = "blue 1 fire blue";
    private static final String INDEXER_SERVO = "indexer_servo";
    private static final double PARK_SECONDS = 27.0;
    private static final double STOP_SECONDS = 30.0;
    private static final double PICKUP_TRAVEL_INCHES = 18.0;
    private static final double PICKUP_POWER = 0.25;
    private static final double VISION_SETTLE_SECONDS = 0.3;

    private enum State { START_FIRE, TO_GOAL, AIM, FIRE, TO_PICKUP, ALIGN_PICKUP, INTAKE, PARK, RETREAT, DONE }

    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private final ElapsedTime pipelineTimer = new ElapsedTime();
    private MecanumDrive drive;
    private Navigator navigator;
    private PinpointOdometry odometry;
    private DcMotor intake;
    private DcMotor shooter;
    private CRServo feeder;
    private Limelight3A limelight;
    private Auto30secSettings.TeamOption team;
    private AutoStartPosition position;
    private State state;
    private int pipeline = -1;
    private boolean routeConfigured;
    private boolean shooterRunning;
    private boolean tagAllowsFire;
    private double tagTx = Double.NaN;
    private double ballArea = Double.NaN;
    private Pose2D pickupStart;
    private Pose2D retreatStart;
    private final DcMotor[] driveMotors = new DcMotor[4];
    private DistanceSensor frontDistance;
    private DistanceSensor rearDistance;
    private double frontInches = Double.NaN;
    private double rearInches = Double.NaN;
    private String status = "";

    @Override
    public void runOpMode() {
        team = Auto30secSettings.select(Team);
        position = AutoStartPosition.valueOf(team.name());
        try {
            drive = new MecanumDrive(hardwareMap);
            String[] driveNames = {Constants.FRONT_LEFT_DRIVE, Constants.FRONT_RIGHT_DRIVE,
                    Constants.BACK_LEFT_DRIVE, Constants.BACK_RIGHT_DRIVE};
            for (int i = 0; i < driveNames.length; i++) {
                driveMotors[i] = hardwareMap.get(DcMotor.class, driveNames[i]);
                driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            frontDistance = hardwareMap.tryGet(DistanceSensor.class, "sensor_distance");
            rearDistance = hardwareMap.tryGet(DistanceSensor.class, "rear_distance");
            intake = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);
            shooter = hardwareMap.get(DcMotor.class, Constants.SHOOTER_MOTOR);
            initializeMotor(intake, DcMotor.ZeroPowerBehavior.BRAKE);
            initializeMotor(shooter, DcMotor.ZeroPowerBehavior.FLOAT);
            feeder = hardwareMap.get(CRServo.class, INDEXER_SERVO);
            feeder.setDirection(CRServo.Direction.REVERSE);
            feeder.setPower(0);
            odometry = new PinpointOdometry(hardwareMap, position.start);
            navigator = new Navigator(drive, odometry);
            limelight = hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT);
            if (limelight != null) {
                limelight.setPollRateHz(50);
                limelight.start();
            }
            setPipeline(Constants.LIMELIGHT_APRILTAG_PIPELINE);
            // The existing zero poses are placeholders, not real pickup/parking destinations.
            routeConfigured = validFieldPose(position.start) && validFieldPose(position.hive1)
                    && validFieldPose(position.garden) && validFieldPose(position.park)
                    && separated(position.start, position.hive1)
                    && separated(position.hive1, position.garden)
                    && separated(position.hive1, position.park);
            telemetry.setMsTransmissionInterval(100);
            while (opModeInInit()) {
                odometry.update();
                updateTag();
                addTelemetry();
                telemetry.addLine("Keep still during Pinpoint calibration; no motors run in INIT.");
                telemetry.addLine(team.fireOnStart ? "PLAY starts shooter AND feeder immediately."
                        : "No startup fire; later shots require a fresh alliance tag within +/-10 degrees.");
                telemetry.update();
                idle();
            }
            waitForStart();
            if (!opModeIsActive()) {
                return;
            }
            matchTimer.reset();
            enter(team.fireOnStart ? State.START_FIRE : routeConfigured ? State.TO_GOAL : State.AIM);
            while (opModeIsActive() && matchTimer.seconds() < STOP_SECONDS) {
                odometry.update();
                if (!readyOdometry()) {
                    status = "Pinpoint not ready or pose invalid; stopped";
                    enter(State.DONE);
                }
                if (matchTimer.seconds() >= PARK_SECONDS && state != State.PARK && state != State.DONE) {
                    enter(routeConfigured && state != State.RETREAT ? State.PARK : State.DONE);
                }
                updateTag();
                step();
                checkObstacle();
                addTelemetry();
                telemetry.update();
                idle();
            }
        } finally {
            stopOutputs();
            if (limelight != null) {
                limelight.stop();
            }
        }
    }

    private void step() {
        switch (state) {
            case START_FIRE:
                // Explicit team-1 override applies only to the preload, never to later shots.
                spinShooter();
                feeder.setPower(Constants.SHOOTER_SERVO_POWER);
                if (stateTimer.seconds() >= Constants.AUTO_FEED_TIME_SEC) {
                    enter(routeConfigured ? State.TO_PICKUP : State.DONE);
                }
                break;
            case TO_GOAL:
                spinShooter();
                moveTo(position.hive1, State.AIM);
                break;
            case AIM:
                spinShooter();
                if (tagAllowsFire && shooterTimer.seconds() >= Constants.AUTO_SHOOTER_SPINUP_SEC) {
                    enter(State.FIRE);
                } else if (stateTimer.seconds() >= Constants.AUTO_APRILTAG_TIMEOUT_SEC) {
                    status = "No qualifying AprilTag; skipped shot";
                    enter(routeConfigured ? State.TO_PICKUP : State.DONE);
                } else {
                    navigator.driveRobotRelative(0, 0,
                            Auto30secSettings.searchTurn(routeConfigured && pipelineReady(), tagTx));
                }
                break;
            case FIRE:
                spinShooter();
                // Tag loss, stale frames, or excess angle revoke feeding on the next loop.
                feeder.setPower(tagAllowsFire ? Constants.SHOOTER_SERVO_POWER : 0);
                if (stateTimer.seconds() >= Constants.AUTO_FEED_TIME_SEC) {
                    enter(routeConfigured ? State.TO_PICKUP : State.DONE);
                }
                break;
            case TO_PICKUP:
                moveTo(position.garden, State.ALIGN_PICKUP);
                break;
            case ALIGN_PICKUP:
                double tx = pickupAngle();
                if (Double.isFinite(tx)) {
                    if (Math.abs(tx) <= 2.0) {
                        pickupStart = odometry.getPose();
                        enter(State.INTAKE);
                    } else {
                        navigator.driveRobotRelative(0, Range.clip(tx * 0.02, -PICKUP_POWER, PICKUP_POWER), 0);
                    }
                } else {
                    navigator.driveRobotRelative(0, 0, Auto30secSettings.searchTurn(pipelineReady(), tx));
                }
                if (state == State.ALIGN_PICKUP && stateTimer.seconds() >= Constants.AUTO_PICKUP_TIMEOUT_SEC) {
                    status = "No centered pickup target; skipped intake";
                    enter(State.TO_GOAL);
                }
                break;
            case INTAKE:
                intake.setPower(Constants.INTAKE_POWER);
                double traveled = Math.hypot(odometry.getX() - pickupStart.getX(DistanceUnit.INCH),
                        odometry.getY() - pickupStart.getY(DistanceUnit.INCH));
                if (traveled >= PICKUP_TRAVEL_INCHES || stateTimer.seconds() >= Constants.AUTO_PICKUP_TIMEOUT_SEC) {
                    enter(State.TO_GOAL);
                } else {
                    double pickupTx = pickupAngle();
                    double headingError = AngleUnit.normalizeDegrees(
                            pickupStart.getHeading(AngleUnit.DEGREES) - odometry.getHeading());
                    if (Double.isFinite(pickupTx)) {
                        navigator.driveRobotRelative(Auto30secSettings.pickupForward(pipelineReady(), pickupTx),
                                Range.clip(pickupTx * 0.02, -0.15, 0.15),
                                Range.clip(-headingError * Constants.MOVE_KP_HEADING, -0.2, 0.2));
                    } else {
                        navigator.stop();
                    }
                }
                break;
            case PARK:
                moveTo(position.park, State.DONE);
                break;
            case RETREAT:
                runRetreat();
                break;
            case DONE:
                stopOutputs();
                break;
        }
    }

    private void checkObstacle() {
        frontInches = frontDistance == null ? Double.NaN : frontDistance.getDistance(DistanceUnit.INCH);
        if (state == State.DONE || state == State.RETREAT || frontDistance == null) return;
        double forwardPower = 0;
        for (DcMotor motor : driveMotors) forwardPower += motor.getPower() / 4.0;
        if (forwardPower > 0.02 && !Auto30secSettings.validRange(frontInches)) {
            status = "Front distance reading invalid; stopped";
            enter(State.DONE);
        } else if (Auto30secSettings.obstacleAhead(frontInches, forwardPower)) {
            retreatStart = odometry.getPose();
            status = "Obstacle ahead: braking, then backing up";
            enter(State.RETREAT);
        }
    }

    private void runRetreat() {
        double traveled = Math.hypot(odometry.getX() - retreatStart.getX(DistanceUnit.INCH),
                odometry.getY() - retreatStart.getY(DistanceUnit.INCH));
        rearInches = rearDistance == null ? Double.NaN : rearDistance.getDistance(DistanceUnit.INCH);
        if (traveled >= Auto30secSettings.RETREAT_INCHES
                || stateTimer.seconds() >= Auto30secSettings.RETREAT_TIMEOUT_SECONDS) {
            status = "Obstacle retreat complete; staying stopped";
            enter(State.DONE);
        } else if (!Auto30secSettings.canBackUp(odometry.getX(), odometry.getY(), odometry.getHeading(),
                rearDistance != null, rearInches)) {
            status = "Retreat blocked by rear range or field boundary; staying stopped";
            enter(State.DONE);
        } else if (stateTimer.seconds() >= Auto30secSettings.RETREAT_PAUSE_SECONDS) {
            navigator.driveRobotRelative(-Auto30secSettings.RETREAT_POWER, 0, 0);
        } else {
            navigator.stop();
        }
    }

    private void moveTo(Pose2D target, State next) {
        if (navigator.moveTo(target)) {
            enter(next);
        } else if (navigator.isTimedOut()) {
            status = "Navigation timed out; stopped";
            enter(State.DONE);
        }
    }

    private void enter(State next) {
        if (navigator != null) {
            navigator.stop();
        }
        intake.setPower(0);
        feeder.setPower(0);
        boolean keepShooter = next == State.START_FIRE || next == State.TO_GOAL
                || next == State.AIM || next == State.FIRE;
        if (!keepShooter) {
            shooter.setPower(0);
            shooterRunning = false;
        }
        state = next;
        stateTimer.reset();
        setPipeline(next == State.TO_PICKUP || next == State.ALIGN_PICKUP || next == State.INTAKE
                ? (team.blue ? Constants.LIMELIGHT_BLUE_PIPELINE : Constants.LIMELIGHT_RED_PIPELINE)
                : Constants.LIMELIGHT_APRILTAG_PIPELINE);
        if (next == State.ALIGN_PICKUP || next == State.INTAKE) {
            intake.setPower(Constants.INTAKE_POWER);
        }
        if (next == State.START_FIRE) {
            spinShooter();
            feeder.setPower(Constants.SHOOTER_SERVO_POWER);
        }
    }

    private void spinShooter() {
        if (!shooterRunning) {
            shooterTimer.reset();
            shooterRunning = true;
        }
        shooter.setPower(Constants.SHOOTER_POWER);
    }

    private void setPipeline(int next) {
        if (pipeline != next) {
            pipeline = next;
            pipelineTimer.reset();
            if (limelight != null) {
                limelight.pipelineSwitch(next);
            }
        }
    }

    private boolean pipelineReady() {
        return currentResult() != null;
    }

    private LLResult currentResult() {
        if (limelight == null || !limelight.isConnected() || pipelineTimer.seconds() < VISION_SETTLE_SECONDS) {
            return null;
        }
        LLResult result = limelight.getLatestResult();
        // A fresh no-target frame still permits searching; a stale/wrong-pipeline frame does not.
        return result != null && result.getPipelineIndex() == pipeline
                && result.getStaleness() >= 0 && result.getStaleness() <= 250 ? result : null;
    }

    private LLResult freshResult() {
        LLResult result = currentResult();
        return result != null && result.isValid() ? result : null;
    }

    private void updateTag() {
        tagAllowsFire = false;
        tagTx = Double.NaN;
        if (pipeline != Constants.LIMELIGHT_APRILTAG_PIPELINE) {
            return;
        }
        LLResult result = freshResult();
        if (result == null || result.getFiducialResults() == null) {
            return;
        }
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (!Auto30secSettings.matchesAlliance(tag.getFiducialId(), team.blue)) {
                continue;
            }
            double tx = tag.getTargetXDegrees();
            if (Double.isFinite(tx) && (!Double.isFinite(tagTx) || Math.abs(tx) < Math.abs(tagTx))) {
                tagTx = tx;
            }
            Pose3D pose = tag.getTargetPoseCameraSpace();
            double pitch = pose != null && pose.getOrientation() != null
                    ? pose.getOrientation().getPitch(AngleUnit.DEGREES) : Double.NaN;
            tagAllowsFire |= Auto30secSettings.mayFire(tag.getFiducialId(), team.blue, tx, pitch,
                    result.isValid(), true, result.getPipelineIndex(), pipeline, result.getStaleness());
        }
    }

    private double pickupAngle() {
        ballArea = Double.NaN;
        LLResult result = freshResult();
        if (result == null) {
            return Double.NaN;
        }
        List<LLResultTypes.ColorResult> colors = result.getColorResults();
        double closestTx = Double.NaN;
        if (colors != null) {
            for (LLResultTypes.ColorResult color : colors) {
                double tx = color.getTargetXDegrees();
                double area = color.getTargetArea();
                if (Auto30secSettings.preferBall(area, tx, ballArea, closestTx)) {
                    closestTx = tx;
                    ballArea = area;
                }
            }
        }
        return closestTx;
    }

    private boolean readyOdometry() {
        return "READY".equals(odometry.getStatus().name()) && Double.isFinite(odometry.getX())
                && Double.isFinite(odometry.getY()) && Double.isFinite(odometry.getHeading());
    }

    private static boolean validFieldPose(Pose2D pose) {
        return Auto30secSettings.isOnField(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH))
                && Double.isFinite(pose.getHeading(AngleUnit.DEGREES));
    }

    private static boolean separated(Pose2D a, Pose2D b) {
        return Math.hypot(a.getX(DistanceUnit.INCH) - b.getX(DistanceUnit.INCH),
                a.getY(DistanceUnit.INCH) - b.getY(DistanceUnit.INCH)) > Constants.MOVE_POSITION_TOLERANCE_IN;
    }

    private static void initializeMotor(DcMotor motor, DcMotor.ZeroPowerBehavior behavior) {
        motor.setPower(0);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(behavior);
    }

    private void stopOutputs() {
        if (drive != null) drive.stop();
        if (intake != null) intake.setPower(0);
        if (shooter != null) shooter.setPower(0);
        if (feeder != null) feeder.setPower(0);
    }

    private void addTelemetry() {
        telemetry.addData("Auto / Team", "Ryan's auto test / " + team.label);
        telemetry.addData("State / seconds", "%s / %.1f", state == null ? "INIT" : state, state == null ? 0 : matchTimer.seconds());
        telemetry.addData("Field tiles", "%d x %d = %d tiles; %.0f x %.0f inches each",
                Auto30secSettings.TILES_PER_SIDE, Auto30secSettings.TILES_PER_SIDE,
                Auto30secSettings.TILE_COUNT, Auto30secSettings.TILE_SIZE_INCHES, Auto30secSettings.TILE_SIZE_INCHES);
        telemetry.addData("Field size / X,Y bounds", "%.0f x %.0f inches / -%.0f to +%.0f inches",
                Auto30secSettings.FIELD_SIZE_INCHES, Auto30secSettings.FIELD_SIZE_INCHES,
                Auto30secSettings.FIELD_HALF_SIZE_INCHES, Auto30secSettings.FIELD_HALF_SIZE_INCHES);
        telemetry.addData("Route", routeConfigured ? "Pose navigation enabled" : "Placeholder or invalid field poses: driving disabled; stationary firing only");
        telemetry.addData("Limelight / pipeline", "%s / %d", limelight != null && limelight.isConnected() ? "Connected" : "Missing", pipeline);
        telemetry.addData("Pipeline purpose", pipeline == Constants.LIMELIGHT_APRILTAG_PIPELINE
                ? "AprilTag search / aim" : team.blue ? "Blue ball color search / intake" : "Red ball color search / intake");
        telemetry.addData("Pipeline fresh / ready", pipelineReady());
        telemetry.addData("Alliance tag tx / fire allowed", "%.1f / %b", tagTx, tagAllowsFire);
        if (state == State.ALIGN_PICKUP || state == State.INTAKE) {
            telemetry.addData("Nearest visible ball tx / intake power", "%.1f / %.2f", pickupAngle(), intake.getPower());
            telemetry.addData("Ball area (nearer estimate, % image)", ballArea);
        }
        telemetry.addData("Obstacle protection", frontDistance == null
                ? "NO front sensor: walls/robots not detected" : "Front distance stop-and-retreat enabled");
        telemetry.addData("Front / rear range (in)", "%.1f / %.1f", frontInches, rearInches);
        telemetry.addData("Rear protection", rearDistance == null ? "No rear sensor; reverse path not sensed" : "Rear distance sensor present");
        telemetry.addData("Status", status);
        if (odometry != null) odometry.addTelemetry(telemetry);
    }
}

