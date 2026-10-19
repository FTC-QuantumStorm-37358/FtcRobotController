package org.firstinspires.ftc.teamcode.quantumstorm.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.Shooter;

import java.util.List;

/** Collects four camera-tracked balls, then centers on an AprilTag and feeds the shooter. */
@Autonomous(name = "Approach Blue or Yellow (Mecanum Test)", group = "Tests")
public class PollenAndAprilTagAuto extends LinearOpMode {

    private static final double APPROACH_POWER = 0.12;
    private static final double STRAFE_GAIN = 0.008;
    private static final double MAX_STRAFE_POWER = 0.12;
    private static final double CENTER_TOLERANCE_DEGREES = 2.0;
    private static final double STOP_TARGET_AREA_PERCENT = 12.0;
    private static final double BALL_LOST_CONFIRM_SEC = 0.35;
    private static final double BALL_SETTLE_SEC = 3.0;
    private static final double HIVE_SEARCH_TURN_POWER = 0.08;

    private enum Stage { COLLECT, SETTLE, FIND_HIVE, FIRE, DONE, FAILED }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        DcMotor intake = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo cameraTilt = hardwareMap.get(Servo.class, Constants.LIMELIGHT_TILT_SERVO);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT);
        Shooter shooter = new Shooter(hardwareMap);
        ElapsedTime stageTimer = new ElapsedTime();
        ElapsedTime targetLostTimer = new ElapsedTime();
        ElapsedTime hiveSearchTimer = new ElapsedTime();
        ElapsedTime spinupTimer = new ElapsedTime();
        ElapsedTime fireTimer = new ElapsedTime();

        int ballsCollected = 0;
        boolean targetWasAtIntake = false;
        boolean targetLostTimerStarted = false;
        Stage stage = Stage.COLLECT;
        String status = "Collecting balls with the color pipeline";

        cameraTilt.setPosition(Constants.LIMELIGHT_TILT_DOWN);
        intake.setPower(0);
        limelight.pipelineSwitch(Constants.LIMELIGHT_COLOR_PIPELINE);
        limelight.start();

        telemetry.addLine("Collect four balls, then find and center on a Hive AprilTag.");
        telemetry.addLine("Camera-based intake counting is an estimate; supervise the robot.");
        telemetry.addLine("Press Start to begin; stop the OpMode from the Driver Station.");
        telemetry.update();

        try {
            waitForStart();
            while (opModeIsActive() && stage != Stage.DONE && stage != Stage.FAILED) {
                LLResult result = limelight.getLatestResult();

                switch (stage) {
                    case COLLECT: {
                        LLResultTypes.ColorResult target = findLargestColorTarget(result);
                        if (target == null) {
                            drive.stop();
                            if (targetWasAtIntake) {
                                intake.setPower(Constants.INTAKE_POWER);
                                if (!targetLostTimerStarted) {
                                    targetLostTimer.reset();
                                    targetLostTimerStarted = true;
                                } else if (targetLostTimer.seconds() >= BALL_LOST_CONFIRM_SEC) {
                                    ballsCollected++;
                                    targetWasAtIntake = false;
                                    targetLostTimerStarted = false;
                                    status = "Ball captured; allowing time for the intake";
                                    stage = ballsCollected >= Constants.AUTO_BALLS_TO_SHOOT
                                            ? Stage.FIND_HIVE : Stage.SETTLE;
                                    stageTimer.reset();

                                    if (stage == Stage.FIND_HIVE) {
                                        cameraTilt.setPosition(Constants.LIMELIGHT_TILT_UP);
                                        limelight.pipelineSwitch(Constants.LIMELIGHT_APRILTAG_PIPELINE);
                                        shooter.spinUp();
                                        hiveSearchTimer.reset();
                                        spinupTimer.reset();
                                        status = "Four balls collected; searching for Hive AprilTag";
                                    }
                                }
                            } else {
                                intake.setPower(0);
                                status = "No color target visible";
                            }
                        } else {
                            targetLostTimerStarted = false;
                            double angle = target.getTargetXDegrees();
                            double area = target.getTargetArea();

                            if (targetWasAtIntake || area >= STOP_TARGET_AREA_PERCENT) {
                                targetWasAtIntake = true;
                                drive.stop();
                                intake.setPower(Constants.INTAKE_POWER);
                                status = "Target at intake; waiting for camera to lose it";
                            } else {
                                double strafe = Math.abs(angle) <= CENTER_TOLERANCE_DEGREES
                                        ? 0.0
                                        : clamp(angle * STRAFE_GAIN,
                                        -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
                                drive.drive(APPROACH_POWER, strafe, 0);
                                intake.setPower(Constants.INTAKE_POWER);
                                status = "Approaching and centering on target";
                            }
                            telemetry.addData("Target angle", "%.1f degrees", angle);
                            telemetry.addData("Target area", "%.1f%%", area);
                        }
                        break;
                    }

                    case SETTLE:
                        drive.stop();
                        intake.setPower(Constants.INTAKE_POWER);
                        if (stageTimer.seconds() >= BALL_SETTLE_SEC) {
                            intake.setPower(0);
                            stage = Stage.COLLECT;
                            status = "Ready to collect the next ball";
                        } else {
                            status = "Intake settling after capture";
                            telemetry.addData("Settle time remaining", "%.1f s",
                                    BALL_SETTLE_SEC - stageTimer.seconds());
                        }
                        break;

                    case FIND_HIVE: {
                        boolean currentAprilTagResult = result != null && result.isValid()
                                && result.getPipelineIndex() == Constants.LIMELIGHT_APRILTAG_PIPELINE;
                        boolean tagVisible = currentAprilTagResult
                                && !result.getFiducialResults().isEmpty();

                        drive.stop();
                        intake.setPower(0);
                        if (!tagVisible) {
                            drive.drive(0, 0, HIVE_SEARCH_TURN_POWER);
                            status = "Searching for Hive AprilTag";
                        } else {
                            double angle = result.getTx();
                            telemetry.addData("AprilTag angle", "%.1f degrees", angle);
                            if (Math.abs(angle) <= Constants.AUTO_AIM_TOLERANCE_DEG) {
                                drive.stop();
                                status = "Hive centered; waiting for shooter";
                                if (spinupTimer.seconds() >= Constants.AUTO_SHOOTER_SPINUP_SEC) {
                                    shooter.feed();
                                    stage = Stage.FIRE;
                                    fireTimer.reset();
                                    status = "Firing collected balls";
                                }
                            } else {
                                double turn = Range.clip(angle * Constants.AUTO_AIM_TURN_KP,
                                        -Constants.MOVE_MAX_POWER, Constants.MOVE_MAX_POWER);
                                drive.drive(0, 0, turn);
                                status = "Centering on Hive AprilTag";
                            }
                        }

                        if (stage == Stage.FIND_HIVE
                                && hiveSearchTimer.seconds() >= Constants.AUTO_APRILTAG_TIMEOUT_SEC) {
                            drive.stop();
                            shooter.stop();
                            stage = Stage.FAILED;
                            status = tagVisible
                                    ? "Could not center on AprilTag before timeout"
                                    : "Hive AprilTag not found before timeout";
                        }
                        break;
                    }

                    case FIRE:
                        drive.stop();
                        shooter.feed();
                        if (fireTimer.seconds() >= Constants.AUTO_FEED_TIME_SEC) {
                            shooter.stop();
                            stage = Stage.DONE;
                            status = "Firing complete";
                        }
                        break;

                    default:
                        break;
                }

                telemetry.addData("Stage", stage);
                telemetry.addData("Balls collected (estimated)", "%d / %d",
                        ballsCollected, Constants.AUTO_BALLS_TO_SHOOT);
                telemetry.addData("Status", status);
                telemetry.update();
                sleep(50);
            }
        } finally {
            intake.setPower(0);
            shooter.stop();
            drive.stop();
            limelight.stop();
        }
    }

    private LLResultTypes.ColorResult findLargestColorTarget(LLResult result) {
        if (result == null || !result.isValid()
                || result.getPipelineIndex() != Constants.LIMELIGHT_COLOR_PIPELINE) {
            return null;
        }

        List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
        if (colorTargets == null || colorTargets.isEmpty()) {
            return null;
        }

        LLResultTypes.ColorResult largest = null;
        for (LLResultTypes.ColorResult target : colorTargets) {
            if (largest == null || target.getTargetArea() > largest.getTargetArea()) {
                largest = target;
            }
        }
        return largest;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
