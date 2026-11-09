package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.Navigator;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;

import java.util.List;

/** Autonomous integration test: preload shot, AprilTag correction, then one timed floor pickup. */
@Autonomous(name = "Ryan: Limelight + Odometry Auto", group = "Vision")
public class RyanAutoTest extends LinearOpMode {

    private static final int APRILTAG_PIPELINE = Constants.LIMELIGHT_APRILTAG_PIPELINE;
    private static final int BLUE_PIPELINE = Constants.LIMELIGHT_BLUE_PIPELINE;
    private static final int RED_PIPELINE = Constants.LIMELIGHT_RED_PIPELINE;
    private static final double AIM_TOLERANCE_DEG = 5.0;
    private static final double AIM_TURN_GAIN = 0.02;
    private static final double AIM_MAX_TURN = 0.25;
    private static final double PICKUP_SECONDS = 1.2;
    private static final double PICKUP_FORWARD_POWER = 0.25;
    private static final double INDEXER_FEED_SECONDS = 0.5;
    private static final double VISION_SETTLE_SECONDS = 0.3;
    private static final double VISION_TIMEOUT_SECONDS = 6.0;
    private static final double AUTO_TIMEOUT_SECONDS = 30.0;

    // Tune these on the robot after measuring the tag range and shooter response.
    private static final double CLOSE_SHOT_RANGE_IN = 24.0;
    private static final double FAR_SHOT_RANGE_IN = 120.0;
    private static final double CLOSE_SHOOTER_POWER = Constants.SHOOTER_POWER;
    private static final double FAR_SHOOTER_POWER = 0.80;
    private static final double INTAKE_CURRENT_SPIKE_AMPS = 4.0;

    private enum State { TO_GOAL, AIM_AND_SHOOT, FEED, TO_PICKUP, ALIGN_PICKUP, INTAKE, DONE }

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime pipelineTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private final AutoStartPosition[] startPositions = AutoStartPosition.values();
    private MecanumDrive drive;
    private Navigator navigator;
    private PinpointOdometry odometry;
    private Limelight3A limelight;
    private Servo cameraTilt;
    private DcMotorEx shooter;
    private DcMotorEx intake;
    private CRServo feeder;
    private CRServo rightIntake;
    private CRServo leftIntake;
    private AutoStartPosition position = AutoStartPosition.RED_1;
    private State state = State.TO_GOAL;
    private int pipeline = APRILTAG_PIPELINE;
    private boolean routeConfigured;
    private boolean poseReset;
    private boolean shooterRangeSet;
    private double targetDistanceInches = Double.NaN;
    private double shooterPower = CLOSE_SHOOTER_POWER;
    private double intakeCurrentAmps = Double.NaN;
    private String status = "Initializing";

    @Override
    public void runOpMode() {
        try {
            initializeHardware();
            boolean previousLeft = false;
            boolean previousRight = false;

            while (opModeInInit()) {
                if (gamepad1.dpad_left && !previousLeft) {
                    selectStartPosition(-1);
                } else if (gamepad1.dpad_right && !previousRight) {
                    selectStartPosition(1);
                }
                previousLeft = gamepad1.dpad_left;
                previousRight = gamepad1.dpad_right;
                odometry.update();
                updateMegaTagHeading();
                routeConfigured = hasConfiguredRoute();
                telemetry.addData("Start", position.label);
                telemetry.addData("Route", routeConfigured ? "Configured" : "BLOCKED: field poses are placeholders");
                telemetry.addLine("D-pad left/right selects alliance and start position.");
                telemetry.addLine("Set real start, Hive, and Garden poses in Constants before running.");
                telemetry.addLine("Pipeline map: AprilTag 0, Yellow 1, Blue 2, Red 3.");
                telemetry.addData("Limelight", limelight != null && limelight.isConnected() ? "Connected" : "Missing");
                odometry.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }

            waitForStart();
            if (!opModeIsActive()) {
                return;
            }
            matchTimer.reset();
            if (!routeConfigured || limelight == null || !limelight.isConnected() || cameraTilt == null) {
                status = !routeConfigured ? "Stopped: calibrate field poses"
                        : limelight == null || !limelight.isConnected()
                        ? "Stopped: Limelight unavailable" : "Stopped: camera tilt servo unavailable";
                state = State.DONE;
            } else {
                state = State.TO_GOAL;
                stateTimer.reset();
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooter.setPower(CLOSE_SHOOTER_POWER);
                shooterTimer.reset();
                status = "Driving to goal; shooter spinning up";
            }

            while (opModeIsActive() && state != State.DONE && matchTimer.seconds() < AUTO_TIMEOUT_SECONDS) {
                odometry.update();
                updateMegaTagHeading();
                step();
                addTelemetry();
                telemetry.update();
                idle();
            }
            if (state != State.DONE) {
                status = "Autonomous time limit reached";
                state = State.DONE;
            }
            addTelemetry();
            telemetry.update();
        } finally {
            stopOutputs();
            if (limelight != null) {
                limelight.stop();
            }
        }
    }

    private void initializeHardware() {
        drive = new MecanumDrive(hardwareMap);
        position = AutoStartPosition.RED_1;
        odometry = new PinpointOdometry(hardwareMap, position.start);
        navigator = new Navigator(drive, odometry);
        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR);
        intake = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_MOTOR);
        feeder = hardwareMap.get(CRServo.class, Constants.SHOOTER_SERVO);
        rightIntake = hardwareMap.get(CRServo.class, Constants.RIGHT_INTAKE_SERVO);
        leftIntake = hardwareMap.get(CRServo.class, Constants.LEFT_INTAKE_SERVO);
        limelight = hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT);
        cameraTilt = hardwareMap.tryGet(Servo.class, Constants.LIMELIGHT_TILT_SERVO);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(0);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        feeder.setDirection(CRServo.Direction.REVERSE);
        feeder.setPower(0);
        rightIntake.setDirection(CRServo.Direction.FORWARD);
        leftIntake.setDirection(CRServo.Direction.REVERSE);
        rightIntake.setPower(0);
        leftIntake.setPower(0);

        if (limelight != null) {
            limelight.setPollRateHz(50);
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            limelight.start();
        }
        if (cameraTilt != null) {
            cameraTilt.setPosition(Constants.LIMELIGHT_APRILTAG_TILT_POSITION);
        }
    }

    private void selectStartPosition(int offset) {
        int current = position.ordinal();
        int next = (current + offset + startPositions.length) % startPositions.length;
        position = startPositions[next];
        odometry.setPose(position.start);
    }

    private boolean hasConfiguredRoute() {
        return validPose(position.start) && validPose(position.hive1)
                && validPose(position.garden)
                && separated(position.start, position.hive1)
                && separated(position.hive1, position.garden);
    }

    private boolean isBlueAlliance() {
        return position.name().startsWith("BLUE");
    }

    private boolean validPose(Pose2D pose) {
        return pose != null
                && Double.isFinite(pose.getX(DistanceUnit.INCH))
                && Double.isFinite(pose.getY(DistanceUnit.INCH))
                && Double.isFinite(pose.getHeading(AngleUnit.DEGREES))
                && Math.abs(pose.getX(DistanceUnit.INCH)) <= 72
                && Math.abs(pose.getY(DistanceUnit.INCH)) <= 72;
    }

    private boolean separated(Pose2D first, Pose2D second) {
        return Math.hypot(first.getX(DistanceUnit.INCH) - second.getX(DistanceUnit.INCH),
                first.getY(DistanceUnit.INCH) - second.getY(DistanceUnit.INCH))
                > Constants.MOVE_POSITION_TOLERANCE_IN;
    }

    private void updateMegaTagHeading() {
        if (limelight != null && odometry != null) {
            limelight.updateRobotOrientation(odometry.getHeading());
        }
    }

    private void step() {
        switch (state) {
            case TO_GOAL:
                if (navigator.moveTo(position.hive1)) {
                    navigator.stop();
                    setPipeline(APRILTAG_PIPELINE);
                    poseReset = false;
                    enter(State.AIM_AND_SHOOT, "At goal; looking for alliance AprilTag");
                } else if (navigator.isTimedOut()) {
                    finish("Could not reach goal pose");
                }
                break;
            case AIM_AND_SHOOT:
                LLResult tagResult = currentResult();
                LLResultTypes.FiducialResult tag = findAllianceTag(tagResult);
                if (tag != null) {
                    updateRobotPose(tagResult, tag);
                    targetDistanceInches = getTagRangeInches(tag);
                    if (Double.isFinite(targetDistanceInches) && !shooterRangeSet) {
                        shooterPower = shooterPowerForRange(targetDistanceInches);
                        shooter.setPower(shooterPower);
                        shooterTimer.reset();
                        shooterRangeSet = true;
                    }
                    double tx = tag.getTargetXDegrees();
                    if (poseReset && shooterRangeSet && Double.isFinite(tx)
                            && Math.abs(tx) <= AIM_TOLERANCE_DEG
                            && shooterTimer.seconds() >= Constants.AUTO_SHOOTER_SPINUP_SEC) {
                        navigator.stop();
                        feeder.setPower(Constants.SHOOTER_SERVO_POWER);
                        enter(State.FEED, "Aligned; feeding preload");
                    } else if (Double.isFinite(tx)) {
                        navigator.driveRobotRelative(0, 0,
                                Range.clip(tx * AIM_TURN_GAIN, -AIM_MAX_TURN, AIM_MAX_TURN));
                    } else {
                        navigator.stop();
                    }
                } else {
                    navigator.stop();
                }
                if (state == State.AIM_AND_SHOOT && stateTimer.seconds() >= VISION_TIMEOUT_SECONDS) {
                    finish("No usable alliance tag / MegaTag2 pose; shot skipped");
                }
                break;
            case FEED:
                if (stateTimer.seconds() >= INDEXER_FEED_SECONDS) {
                    feeder.setPower(0);
                    shooter.setPower(0);
                    setPipeline(isBlueAlliance() ? BLUE_PIPELINE : RED_PIPELINE);
                    enter(State.TO_PICKUP, "Preload feed complete; switching to floor target");
                }
                break;
            case TO_PICKUP:
                if (navigator.moveTo(position.garden)) {
                    navigator.stop();
                    enter(State.ALIGN_PICKUP, "At pickup pose; aligning to nearest piece");
                } else if (navigator.isTimedOut()) {
                    finish("Could not reach pickup pose");
                }
                break;
            case ALIGN_PICKUP:
                LLResult colorResult = currentResult();
                double targetTx = closestColorTargetTx(colorResult);
                if (Double.isFinite(targetTx)) {
                    if (Math.abs(targetTx) <= AIM_TOLERANCE_DEG) {
                        navigator.stop();
                        intake.setPower(Constants.INTAKE_POWER);
                        rightIntake.setPower(Constants.INTAKE_POWER);
                        leftIntake.setPower(Constants.INTAKE_POWER);
                        enter(State.INTAKE, "Target centered; timed intake started");
                    } else {
                        navigator.driveRobotRelative(0, 0,
                                Range.clip(targetTx * AIM_TURN_GAIN, -AIM_MAX_TURN, AIM_MAX_TURN));
                    }
                } else {
                    navigator.driveRobotRelative(0, 0, 0.15);
                }
                if (state == State.ALIGN_PICKUP && stateTimer.seconds() >= VISION_TIMEOUT_SECONDS) {
                    finish("No color target found; pickup skipped");
                }
                break;
            case INTAKE:
                intakeCurrentAmps = intake.getCurrent(CurrentUnit.AMPS);
                if (Double.isFinite(intakeCurrentAmps) && intakeCurrentAmps >= INTAKE_CURRENT_SPIKE_AMPS) {
                    finish("Intake current spike detected; piece acquired");
                } else if (stateTimer.seconds() >= PICKUP_SECONDS) {
                    finish("Timed intake complete");
                } else {
                    LLResult trackingResult = currentResult();
                    double trackingTx = closestColorTargetTx(trackingResult);
                    double turn = Double.isFinite(trackingTx)
                            ? Range.clip(trackingTx * AIM_TURN_GAIN, -0.12, 0.12) : 0;
                    navigator.driveRobotRelative(PICKUP_FORWARD_POWER, 0, turn);
                }
                break;
            case DONE:
                stopOutputs();
                break;
        }
    }

    private void setPipeline(int nextPipeline) {
        if (pipeline == nextPipeline) {
            return;
        }
        pipeline = nextPipeline;
        pipelineTimer.reset();
        if (cameraTilt != null) {
            cameraTilt.setPosition(pipeline == APRILTAG_PIPELINE
                    ? Constants.LIMELIGHT_APRILTAG_TILT_POSITION : Constants.LIMELIGHT_TILT_DOWN);
        }
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    private LLResult currentResult() {
        if (limelight == null || !limelight.isConnected()
                || pipelineTimer.seconds() < VISION_SETTLE_SECONDS) {
            return null;
        }
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid() && result.getPipelineIndex() == pipeline
                && result.getStaleness() >= 0 && result.getStaleness() <= 250 ? result : null;
    }

    private LLResultTypes.FiducialResult findAllianceTag(LLResult result) {
        if (result == null || result.getFiducialResults() == null) {
            return null;
        }
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult candidate : result.getFiducialResults()) {
            if (!Auto30secSettings.matchesAlliance(candidate.getFiducialId(), isBlueAlliance())) {
                continue;
            }
            if (best == null || Math.abs(candidate.getTargetXDegrees())
                    < Math.abs(best.getTargetXDegrees())) {
                best = candidate;
            }
        }
        return best;
    }

    private void updateRobotPose(LLResult result, LLResultTypes.FiducialResult tag) {
        if (poseReset || result.getBotposeTagCount() < 1) {
            return;
        }
        Pose3D fieldPose = result.getBotpose_MT2();
        if (fieldPose == null || fieldPose.getPosition() == null || fieldPose.getOrientation() == null) {
            return;
        }
        Position positionInches = fieldPose.getPosition().toUnit(DistanceUnit.INCH);
        double x = positionInches.x;
        double y = positionInches.y;
        double heading = fieldPose.getOrientation().getYaw(AngleUnit.DEGREES);
        if (!Auto30secSettings.isOnField(x, y) || !Double.isFinite(heading)) {
            return;
        }
        odometry.setPose(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading));
        poseReset = true;
        status = "Pinpoint reset from MegaTag2 using tag " + tag.getFiducialId();
    }

    private double getTagRangeInches(LLResultTypes.FiducialResult tag) {
        Pose3D cameraToTag = tag.getTargetPoseCameraSpace();
        if (cameraToTag == null || cameraToTag.getPosition() == null) {
            return Double.NaN;
        }
        Position inches = cameraToTag.getPosition().toUnit(DistanceUnit.INCH);
        return Math.sqrt(inches.x * inches.x + inches.y * inches.y + inches.z * inches.z);
    }

    private double shooterPowerForRange(double rangeInches) {
        double fraction = Range.clip((rangeInches - CLOSE_SHOT_RANGE_IN)
                / (FAR_SHOT_RANGE_IN - CLOSE_SHOT_RANGE_IN), 0.0, 1.0);
        return Range.clip(CLOSE_SHOOTER_POWER
                + fraction * (FAR_SHOOTER_POWER - CLOSE_SHOOTER_POWER), 0.0, 1.0);
    }

    private double closestColorTargetTx(LLResult result) {
        if (result == null) {
            return Double.NaN;
        }
        double bestArea = -1;
        double bestTx = Double.NaN;
        List<LLResultTypes.ColorResult> colors = result.getColorResults();
        if (colors != null) {
            for (LLResultTypes.ColorResult target : colors) {
                if (target.getTargetArea() > bestArea) {
                    bestArea = target.getTargetArea();
                    bestTx = target.getTargetXDegrees();
                }
            }
        }
        List<LLResultTypes.DetectorResult> objects = result.getDetectorResults();
        if (objects != null) {
            for (LLResultTypes.DetectorResult target : objects) {
                if (target.getTargetArea() > bestArea) {
                    bestArea = target.getTargetArea();
                    bestTx = target.getTargetXDegrees();
                }
            }
        }
        return bestTx;
    }

    private void enter(State next, String message) {
        state = next;
        stateTimer.reset();
        status = message;
    }

    private void finish(String message) {
        navigator.stop();
        state = State.DONE;
        status = message;
        stopOutputs();
    }

    private void stopOutputs() {
        if (drive != null) drive.stop();
        if (shooter != null) shooter.setPower(0);
        if (intake != null) intake.setPower(0);
        if (feeder != null) feeder.setPower(0);
        if (rightIntake != null) rightIntake.setPower(0);
        if (leftIntake != null) leftIntake.setPower(0);
    }

    private void addTelemetry() {
        telemetry.addData("State / time", "%s / %.1f s", state, matchTimer.seconds());
        telemetry.addData("Start / alliance", "%s / %s", position.label, isBlueAlliance() ? "Blue" : "Red");
        telemetry.addData("Pipeline", "%d (%s)", pipeline,
                pipeline == APRILTAG_PIPELINE ? "AprilTag" : isBlueAlliance() ? "Blue" : "Red");
        telemetry.addData("Pose reset", poseReset);
        telemetry.addData("Tag range / shooter power", "%.1f in / %.2f", targetDistanceInches, shooterPower);
        telemetry.addData("Shooter encoder", "%d ticks/s", shooter == null ? 0 : (int) shooter.getVelocity());
        telemetry.addData("Intake current", "%.2f A (trip %.2f A)", intakeCurrentAmps, INTAKE_CURRENT_SPIKE_AMPS);
        telemetry.addData("Status", status);
        odometry.addTelemetry(telemetry);
    }
}
