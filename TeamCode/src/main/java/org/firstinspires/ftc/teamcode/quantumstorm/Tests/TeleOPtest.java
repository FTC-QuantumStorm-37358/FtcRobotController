package org.firstinspires.ftc.teamcode.quantumstorm.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.HeadingHold;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.Vision.HiveRaisedDetector;

import java.util.ArrayList;
import java.util.List;

/**
 * Robot-centric mecanum with one intake and one shooter motor.
 * Gamepad 1: left stick drive/strafe, right stick turn, LB slow mode, Y half-speed,
 * RB intake forward, LT intake reverse, RT fixed 60% shooter power while held,
 * IMU heading hold during translation with the turn stick released.
 * A toggles the indexer continuous-rotation servo; X switches its direction.
 * Limelight AprilTag detection is informational only: camera-space pitch 0 +/- 10
 * degrees with matched 30-degree camera/tag tilt. Validate the pose on the actual robot.
 */
@TeleOp(name = "Test: Mecanum Intake Shooter", group = "Quantum Storm")
public class TeleOPtest extends LinearOpMode {
    private static final String INDEXER_SERVO = "indexer_servo";
    private static final double INDEXER_SERVO_POWER = 1.0;
    private static final double SHOOTER_POWER = 0.60;
    private static final double SLOW_DRIVE_SCALE = 0.35;
    private static final double STICK_DEADBAND = 0.05;
    private static final double TRIGGER_DEADBAND = 0.05;
    private static final long MAX_RESULT_AGE_MS = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        String[] driveNames = {Constants.FRONT_LEFT_DRIVE, Constants.FRONT_RIGHT_DRIVE,
                Constants.BACK_LEFT_DRIVE, Constants.BACK_RIGHT_DRIVE};
        String[] driveLabels = {"Front left", "Front right", "Back left", "Back right"};
        DcMotor[] driveMotors = new DcMotor[driveNames.length];
        for (int i = 0; i < driveNames.length; i++) {
            driveMotors[i] = hardwareMap.get(DcMotor.class, driveNames[i]);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_MOTOR);
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR);
        CRServo indexerServo = hardwareMap.get(CRServo.class, INDEXER_SERVO);
        initializeMotor(intake, DcMotor.ZeroPowerBehavior.BRAKE);
        initializeMotor(shooter, DcMotor.ZeroPowerBehavior.FLOAT);
        indexerServo.setDirection(CRServo.Direction.REVERSE);
        indexerServo.setPower(0);
        boolean indexerRunning = false;
        boolean previousA = false;
        boolean previousX = false;
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT);
        telemetry.setMsTransmissionInterval(100);

        try {
            IMU imu = hardwareMap.get(IMU.class, "imu");
            if (!imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)))) {
                throw new IllegalStateException("IMU initialization failed; check configuration and mounting");
            }
            HeadingHold headingHold = new HeadingHold();
            if (limelight != null) {
                limelight.setPollRateHz(50);
                limelight.pipelineSwitch(Constants.LIMELIGHT_APRILTAG_PIPELINE);
                limelight.start();
            }
            addControlInstructions();
            telemetry.addLine("IMU: logo LEFT, USB FORWARD; heading hold while translating.");
            telemetry.addLine("Hive status does NOT control the shooter.");
            telemetry.addLine("Camera/tag tilt: 30 degrees; expected relative pitch: 0 +/- 10.");
            telemetry.addData("AprilTag pipeline", Constants.LIMELIGHT_APRILTAG_PIPELINE);
            telemetry.addLine("Hive AprilTag IDs: 30-45; 3D target pose required.");
            telemetry.addData("Limelight", limelight == null ? "Not configured" : "Initialized");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                addControlInstructions();
                double driveScale = gamepad1.left_bumper ? SLOW_DRIVE_SCALE : 1.0;
                double forward = deadband(-gamepad1.left_stick_y);
                double strafe = deadband(gamepad1.left_stick_x);
                double manualTurn = deadband(gamepad1.right_stick_x);
                double yawDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double turn = headingHold.update(yawDegrees,
                        forward != 0.0 || strafe != 0.0, manualTurn);
                if (gamepad1.y) {
                    // Normalize first so Y halves actual wheel power even with combined stick inputs.
                    double normalization = Math.max(1.0,
                            (Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)) * driveScale);
                    driveScale *= 0.5 / normalization;
                }
                drive.drive(forward * driveScale, strafe * driveScale, turn * driveScale);
                telemetry.addData("IMU yaw (degrees)", "%.1f", yawDegrees);
                telemetry.addData("Heading hold", !Double.isFinite(yawDegrees) ? "IMU unavailable"
                        : headingHold.isActive() ? "ACTIVE" : "Idle / manual turn");
                telemetry.addData("Heading error (degrees)", headingHold.getErrorDegrees());
                telemetry.addData("Turn command", "%.1f%%", turn * driveScale * 100.0);

                double intakePower = gamepad1.left_trigger > TRIGGER_DEADBAND
                        ? -Constants.INTAKE_POWER
                        : gamepad1.right_bumper ? Constants.INTAKE_POWER : 0.0;
                double shooterPower = gamepad1.right_trigger > TRIGGER_DEADBAND
                        ? SHOOTER_POWER : 0.0;
                if (gamepad1.a && !previousA) {
                    indexerRunning = !indexerRunning;
                }
                previousA = gamepad1.a;
                if (gamepad1.x && !previousX) {
                    indexerServo.setPower(0);
                    indexerServo.setDirection(indexerServo.getDirection() == CRServo.Direction.REVERSE
                            ? CRServo.Direction.FORWARD : CRServo.Direction.REVERSE);
                }
                previousX = gamepad1.x;

                intake.setPower(intakePower);
                shooter.setPower(shooterPower);
                indexerServo.setPower(indexerRunning ? INDEXER_SERVO_POWER : 0.0);

                addHiveTelemetry(limelight);
                telemetry.addData("Intake power", "%.1f%%", intakePower * 100.0);
                telemetry.addData("Shooter power (manual)", "%.1f%%", shooterPower * 100.0);
                telemetry.addData("Indexer servo", "%s (%.1f%%)",
                        indexerRunning ? "RUNNING" : "STOPPED", indexerServo.getPower() * 100.0);
                telemetry.addData("Indexer direction (X to switch)", indexerServo.getDirection());
                telemetry.addData("Slow drive", gamepad1.left_bumper);
                telemetry.addData("Y half-speed", gamepad1.y);
                telemetry.addLine("Motor percentages are commanded power, not measured RPM; negative = reverse.");
                for (int i = 0; i < driveMotors.length; i++) {
                    telemetry.addData(driveLabels[i] + " power", "%.1f%%", driveMotors[i].getPower() * 100.0);
                }
                telemetry.update();
                sleep(20);
            }
        } finally {
            drive.stop();
            intake.setPower(0);
            shooter.setPower(0);
            indexerServo.setPower(0);
            if (limelight != null) {
                limelight.stop();
            }
        }
    }

    private void addControlInstructions() {
        telemetry.addLine("GAMEPAD 1 CONTROLS (Gamepad 2 unused)");
        telemetry.addLine("Left stick: forward/back + strafe | Right stick X: turn");
        telemetry.addLine("Hold LB: 35% drive scale | Hold Y: halve drive output (stacks with LB)");
        telemetry.addLine("Hold RB: intake forward | Hold LT: intake reverse (priority over RB)");
        telemetry.addLine("Hold RT: shooter 60%; release to stop | Press A: toggle indexer");
        telemetry.addLine("Press X: switch indexer direction (running or stopped)");
        telemetry.addLine("IMU holds heading while translating; right stick overrides. STOP ends all motion.");
    }

    private void addHiveTelemetry(Limelight3A limelight) {
        boolean connected = limelight != null && limelight.isConnected();
        LLResult result = connected ? limelight.getLatestResult() : null;
        long ageMs = result == null ? -1 : result.getStaleness();
        boolean fresh = result != null && result.isValid()
                && result.getPipelineIndex() == Constants.LIMELIGHT_APRILTAG_PIPELINE
                && ageMs >= 0 && ageMs <= MAX_RESULT_AGE_MS;
        int tagId = -1;
        double pitch = Double.NaN;
        double yaw = Double.NaN;
        double roll = Double.NaN;
        double closestError = Double.POSITIVE_INFINITY;
        List<Integer> visibleIds = new ArrayList<>();
        int hiveTags = 0;
        int recommendedTagId = -1;
        double recommendedTx = Double.NaN;

        if (fresh) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags != null) {
                for (LLResultTypes.FiducialResult tag : tags) {
                    visibleIds.add(tag.getFiducialId());
                    if (!HiveRaisedDetector.isHiveTag(tag.getFiducialId())) {
                        continue;
                    }
                    hiveTags++;
                    // 3D tag orientation, not the image-position angles tx/ty, determines pitch.
                    Pose3D pose = tag.getTargetPoseCameraSpace();
                    if (pose == null || pose.getOrientation() == null) {
                        continue;
                    }
                    double candidatePitch = pose.getOrientation().getPitch(AngleUnit.DEGREES);
                    double tagTx = tag.getTargetXDegrees();
                    // Aim and raised-pitch checks must belong to the same tag, not the primary result.
                    if (HiveRaisedDetector.isShootRecommended(tag.getFiducialId(), candidatePitch, tagTx, fresh)
                            && (recommendedTagId == -1 || Math.abs(tagTx) < Math.abs(recommendedTx))) {
                        recommendedTagId = tag.getFiducialId();
                        recommendedTx = tagTx;
                    }
                    double error = Math.abs(candidatePitch - HiveRaisedDetector.RAISED_PITCH_DEGREES);
                    // Report the closest raised match when multiple hive tags are visible.
                    if (Double.isFinite(candidatePitch) && error < closestError) {
                        closestError = error;
                        tagId = tag.getFiducialId();
                        pitch = candidatePitch;
                        yaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);
                        roll = pose.getOrientation().getRoll(AngleUnit.DEGREES);
                    }
                }
            }
        }

        HiveRaisedDetector.Status status = HiveRaisedDetector.classify(tagId, pitch, fresh);
        telemetry.addData("Shooting advice", recommendedTagId != -1
                ? "Shoot recommended" : "Shoot not recommended");
        telemetry.addData("Recommended tag ID / horizontal angle", "%d / %.1f degrees",
                recommendedTagId, recommendedTx);
        telemetry.addData("Limelight", connected ? "Connected" : "Unavailable");
        telemetry.addData("AprilTag pipeline expected / actual", "%d / %d",
                Constants.LIMELIGHT_APRILTAG_PIPELINE, result == null ? -1 : result.getPipelineIndex());
        telemetry.addData("Vision valid / fresh", "%b / %b", result != null && result.isValid(), fresh);
        telemetry.addData("Vision age (ms; max 250)", ageMs);
        telemetry.addData("Visible AprilTag IDs (fresh only)", visibleIds);
        telemetry.addData("Hive tags visible (30-45)", hiveTags);
        telemetry.addData("Hive status (any visible hive)", status);
        telemetry.addData("Selected hive tag (30-45)", tagId);
        telemetry.addData("Tag pose yaw / pitch / roll (degrees)", "%.1f / %.1f / %.1f", yaw, pitch, roll);
        telemetry.addData("Pitch error (degrees)", Double.isFinite(pitch) ? closestError : Double.NaN);
        telemetry.addData("Raised pitch range", "-10 to +10 degrees inclusive");
        telemetry.addData("Primary target image tx / ty (not tag tilt)", "%.1f / %.1f",
                fresh ? result.getTx() : Double.NaN, fresh ? result.getTy() : Double.NaN);
        telemetry.addData("Vision explanation", !connected ? "Camera missing or disconnected"
                : result == null ? "Waiting for camera result"
                  : result.getPipelineIndex() != Constants.LIMELIGHT_APRILTAG_PIPELINE ? "Wrong pipeline"
                    : !result.isValid() ? "No valid target"
                      : !fresh ? "Stale camera result"
                        : visibleIds.isEmpty() ? "No AprilTags detected"
                          : hiveTags == 0 ? "No hive tag IDs 30-45"
                            : tagId == -1 ? "Hive tag has no usable 3D pitch; check pipeline pose setup"
                              : status == HiveRaisedDetector.Status.RAISED ? "Hive tag within pitch tolerance"
                                : "Hive tag outside pitch tolerance");
    }

    private static void initializeMotor(DcMotorEx motor, DcMotor.ZeroPowerBehavior behavior) {
        motor.setPower(0);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(behavior);
    }

    private static double deadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0.0 : value;
    }
}
