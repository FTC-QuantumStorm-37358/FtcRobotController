package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.RearHiveAim;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.ShooterPidfControl;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;

/** Manual RPM test with optional gamepad 2 Y rear-shooter aiming from absolute field odometry. */
@TeleOp(name = "Biobuzz-Quantum Storm New Shooter Power Control", group = "Quantum Storm Test")
public class QuantumStormShooterPidfTeleOp extends QuantumStormTeleOp {
    private ShooterPidfControl shooterPidf;
    private PinpointOdometry odometry;
    private Quadrant start = Quadrant.SOUTHWEST;
    private final RearHiveAim aim = new RearHiveAim();
    private final TableShooterCalibration distance = new TableShooterCalibration();
    // A Y held through INIT/START must be released before it can start aiming.
    private boolean previousGamepad2Y;
    private boolean manualDriveOverride;
    // Preserve the failure sample even if Pinpoint recovers before telemetry is transmitted.
    private String aimFaultPinpointStatus;
    private double aimFaultX, aimFaultY, aimFaultHeading;
    private boolean aimFaultPoseValid;

    @Override protected void initializeShooterTarget() {
        odometry = new PinpointOdometry(hardwareMap, startPose());
    }

    private Pose2D startPose() {
        return new Pose2D(DistanceUnit.INCH, start.startX, start.startY,
                AngleUnit.DEGREES, start.startHeadingDegrees);
    }

    @Override protected void waitForDriverStart() {
        while (!isStarted() && !isStopRequested()) {
            double x = start.startX, y = start.startY;
            if (gamepad1.dpad_left != gamepad1.dpad_right) x = gamepad1.dpad_left ? -12.75 : 12.75;
            if (gamepad1.dpad_down != gamepad1.dpad_up) y = gamepad1.dpad_down ? -DistanceShooterCalibration.START_Y_INCHES
                    : DistanceShooterCalibration.START_Y_INCHES;
            Quadrant selected = Quadrant.at(x, y);
            if (selected != start) {
                start = selected;
                odometry.setPose(startPose());
            }
            odometry.update();
            telemetry.addLine("INIT: gamepad 1 D-pad LEFT/RIGHT = X sign; DOWN/UP = Y sign");
            telemetry.addData("Place robot at (in / deg)", "%.2f, %.2f / %.0f",
                    start.startX, start.startY, start.startHeadingDegrees);
            telemetry.addLine("Keep still for Pinpoint; front faces Y wall, rear shooter into field");
            addShooterTargetTelemetry();
            telemetry.update();
            previousGamepad2Y = gamepad2.y;
            idle();
        }
        previousGamepad2Y = gamepad2.y;
    }

    @Override protected void updateDriverAssistance(boolean freshShooterA) {
        odometry.update();
        boolean freshY = gamepad2.y && !previousGamepad2Y;
        previousGamepad2Y = gamepad2.y;
        if (freshY) aimFaultPinpointStatus = null;
        manualDriveOverride = Math.max(Math.abs(gamepad1.left_stick_y),
                Math.max(Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.right_stick_x)))
                > RearHiveAim.MANUAL_OVERRIDE_DEADBAND;
        aim.update(odometry.getX(), odometry.getY(), odometry.getHeading(), odometry.isValid(),
                System.nanoTime() / 1e9, freshY,
                manualDriveOverride || gamepad2.b || gamepad2.start);
        if (aim.getState() == RearHiveAim.State.INVALID_POSE && aimFaultPinpointStatus == null) {
            aimFaultPinpointStatus = String.valueOf(odometry.getStatus());
            aimFaultPoseValid = odometry.isValid();
            aimFaultX = odometry.getX();
            aimFaultY = odometry.getY();
            aimFaultHeading = odometry.getHeading();
        }
    }

    @Override protected void driveRobot(MecanumDrive drive) {
        if (!manualDriveOverride && aim.controlsDriveThisLoop()) {
            drive.drive(0, 0, aim.getTurnPower());
        } else {
            super.driveRobot(drive);
        }
    }

    @Override protected boolean feedingAllowed() { return !aim.controlsDriveThisLoop(); }

    // The normal TeleOp now selects RPM from distance. This separate manual entry
    // owns its odometry only for aiming and must not inherit that target/readiness gate.
    @Override protected double updateShooterTarget(double currentSetting) { return currentSetting; }
    @Override protected boolean manualShooterAdjustment() { return true; }
    @Override protected boolean shooterTargetReady() { return true; }

    @Override protected double initialShooterSetting() {
        return Constants.SHOOTER_PIDF_TEST_DEFAULT_MULTIPLIER;
    }

    @Override protected String shooterControlLabel() { return "FTCLib PIDF / voltage-compensated power"; }

    @Override protected void configureShooterMotor(DcMotorEx motor) {
        shooterPidf = new ShooterPidfControl(motor, hardwareMap.voltageSensor);
    }

    @Override protected boolean applyShooterOutput(DcMotorEx motor, double multiplier) {
        return shooterPidf.update(multiplier);
    }

    @Override protected boolean feederRequiresShooterRunning() { return true; }

    @Override protected double clampShooterSetting(double setting) {
        return Math.max(0, super.clampShooterSetting(setting));
    }

    @Override protected void addShooterTargetTelemetry() {
        telemetry.addLine("A starts shooter; tap Y aims rear at quadrant hive; B stops/cancels");
        telemetry.addLine("Drive sticks cancel aim; BACK + LB/LT adjusts target by 60 RPM");
        telemetry.addData("Rear hive aim", aim.getState());
        if (aim.getState() == RearHiveAim.State.INVALID_POSE) {
            telemetry.addData("Aim blocked reason", aim.getInvalidPoseReason());
            telemetry.addData("Pinpoint at aim fault", "%s; sample valid: %s",
                    aimFaultPinpointStatus, aimFaultPoseValid);
            telemetry.addData("Pose at aim fault (in / deg)", "%.2f, %.2f / %.1f",
                    aimFaultX, aimFaultY, aimFaultHeading);
            telemetry.addLine("If sample invalid, pose above is LAST KNOWN; correct fault then tap Y again");
        }
        if (aim.getQuadrant() != null) {
            telemetry.addData("Aim hive (in)", "%.2f, %.2f",
                    aim.getQuadrant().hiveX, aim.getQuadrant().hiveY);
            telemetry.addData("Front heading target / error (deg)", "%.1f / %.1f",
                    aim.getTargetHeading(), aim.getHeadingError());
        }
        telemetry.addData("Aim clockwise turn power", "%.3f", aim.getTurnPower());
        if (odometry.isValid()) {
            TableShooterCalibration.Solution shot = distance.atPose(
                    odometry.getX(), odometry.getY(), odometry.getHeading());
            if (shot.ready()) {
                telemetry.addData("Shooter-to-hive distance (in) / table RPM", "%.1f / %.0f",
                        shot.distanceInches, shot.rpm);
            }
        }
        odometry.addTelemetry(telemetry);
        telemetry.addLine("PIDF: bounded I corrects persistent RPM error near target; D is zero");
        telemetry.addData("Shooter PIDF state", shooterPidf.getStatus());
        telemetry.addData("PIDF sampled / target RPM", "%.0f / %.0f",
                shooterPidf.getCurrentRpm(), shooterPidf.getTargetRpm());
        telemetry.addData("Battery voltage during control (V)", "%.2f", shooterPidf.getBatteryVolts());
        telemetry.addData("PIDF / compensated / applied power", "%.3f / %.3f / %.3f",
                shooterPidf.getRawPower(), shooterPidf.getCompensatedPower(), shooterPidf.getAppliedPower());
        telemetry.addData("PIDF integral power / limit", "%.3f / %.2f",
                shooterPidf.getIntegralPower(), Constants.SHOOTER_PIDF_MAX_INTEGRAL_POWER);
        telemetry.addData("PIDF P / I / D / F", "%.6f / %.6f / %.6f / %.6f",
                Constants.SHOOTER_PIDF_P, Constants.SHOOTER_PIDF_I,
                Constants.SHOOTER_PIDF_D, Constants.SHOOTER_PIDF_F);
        telemetry.addLine("Feeder requires shooter ON; after a sensor fault, release A then press it to retry");
    }
}
