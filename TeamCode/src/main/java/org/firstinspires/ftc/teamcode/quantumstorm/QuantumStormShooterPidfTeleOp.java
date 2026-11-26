package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.RearHiveAim;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.AutoShooterTarget;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.ShooterPidfControl;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;

/** Distance target plus driver trim, external PIDF/battery control, and gamepad 2 Y rear aiming. */
@TeleOp(name = Constants.TELEOP_NAME, group = "Quantum Storm")
public class QuantumStormShooterPidfTeleOp extends QuantumStormTeleOp {
    private ShooterPidfControl shooterPidf;
    private PinpointOdometry odometry;
    private Quadrant start = Quadrant.SOUTHWEST;
    private final RearHiveAim aim = new RearHiveAim();
    private final AutoShooterTarget shooterTarget = new AutoShooterTarget();
    // A Y held through INIT/START must be released before it can start aiming.
    private boolean previousGamepad2Y;
    private boolean manualDriveOverride;
    // Preserve the failure sample even if Pinpoint recovers before telemetry is transmitted.
    private String aimFaultPinpointStatus;
    private double aimFaultX, aimFaultY, aimFaultHeading;
    private boolean aimFaultPoseValid;

    @Override protected void initializeShooterTarget() {
        shooterTarget.reset();
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
            updateOdometryAndTarget();
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
        updateOdometryAndTarget();
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

    private void updateOdometryAndTarget() {
        odometry.update();
        shooterTarget.updatePose(odometry.isValid(),
                odometry.getX(), odometry.getY(), odometry.getHeading());
    }

    // Use this entry's own pose and readiness, not the old TeleOp's separate odometry.
    // updateDriverAssistance (or the INIT loop) already sampled Pinpoint once this loop.
    @Override protected double updateShooterTarget(double currentSetting) {
        return shooterTarget.getMultiplier();
    }
    @Override protected boolean manualShooterAdjustment() { return true; }
    @Override protected boolean shooterTargetReady() { return shooterTarget.ready(); }

    @Override protected String shooterAdjustmentInstructions() {
        return "Hold BACK + tap LB/LT = distance target trim +0.005/-0.005 (+30/-30 RPM)";
    }

    @Override protected double adjustShooterSetting(double currentSetting, int direction) {
        shooterTarget.adjustTrim(direction);
        return shooterTarget.getMultiplier();
    }

    @Override protected double initialShooterSetting() {
        return Constants.SHOOTER_AUTO_START_MULTIPLIER;
    }

    @Override protected String shooterControlLabel() { return "Distance + trim / FTCLib PIDF / battery compensation"; }

    @Override protected void configureShooterMotor(DcMotorEx motor) {
        shooterPidf = new ShooterPidfControl(motor, hardwareMap.voltageSensor);
    }

    @Override protected boolean applyShooterOutput(DcMotorEx motor, double multiplier) {
        return shooterPidf.update(multiplier);
    }

    @Override protected boolean feederRequiresShooterRunning() { return true; }

    @Override protected void addShooterTargetTelemetry() {
        telemetry.addLine("A starts shooter; tap Y aims rear at quadrant hive; B stops/cancels");
        telemetry.addLine("Drive sticks cancel aim; BACK + LB/LT trims distance target by 30 RPM");
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
        TableShooterCalibration.Solution shot = shooterTarget.getSolution();
        telemetry.addData("Start reference", start);
        telemetry.addData("Shot solution", shot == null ? "NO VALID ODOMETRY" : shot.status);
        if (shooterTarget.ready()) {
            telemetry.addData("Hive quadrant / target (in)", "%s / %.2f, %.2f",
                    shot.quadrant, shot.quadrant.hiveX, shot.quadrant.hiveY);
            telemetry.addData("Rear shooter position (in)", "%.2f, %.2f", shot.shooterX, shot.shooterY);
            telemetry.addData("Shooter-to-hive / lookup distance (in)", "%.2f / %.2f",
                    shot.distanceInches, shot.lookupDistanceInches);
            telemetry.addData("Auto base RPM / multiplier", "%.0f / %.4f",
                    shooterTarget.getBaseMultiplier() * Constants.SHOOTER_BASE_RPM,
                    shooterTarget.getBaseMultiplier());
            telemetry.addData("Trimmed RPM / multiplier (when ON)", "%.0f / %.4f",
                    shooterTarget.getMultiplier() * Constants.SHOOTER_BASE_RPM,
                    shooterTarget.getMultiplier());
        } else {
            telemetry.addLine("Shooter/feed blocked; press A again after a valid shot solution returns");
        }
        telemetry.addData("Driver trim multiplier / RPM", "%+.3f / %+.0f",
                shooterTarget.getTrimMultiplier(), shooterTarget.getTrimMultiplier() * Constants.SHOOTER_BASE_RPM);
        if (odometry != null) odometry.addTelemetry(telemetry);
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
