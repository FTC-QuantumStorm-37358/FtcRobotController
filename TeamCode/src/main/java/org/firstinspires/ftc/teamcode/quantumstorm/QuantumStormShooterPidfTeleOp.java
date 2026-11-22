package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.RearHiveAim;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.ShooterPidfControl;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;

/** Manual RPM test with optional BACK+A rear-shooter aiming from absolute field odometry. */
@TeleOp(name = "Biobuzz-Quantum Storm New Shooter Power Control", group = "Quantum Storm Test")
public class QuantumStormShooterPidfTeleOp extends QuantumStormTeleOp {
    private ShooterPidfControl shooterPidf;
    private PinpointOdometry odometry;
    private Quadrant start = Quadrant.SOUTHWEST;
    private final RearHiveAim aim = new RearHiveAim();
    private boolean manualDriveOverride;

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
            if (gamepad1.dpad_down != gamepad1.dpad_up) y = gamepad1.dpad_down ? -62.9 : 62.9;
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
            idle();
        }
    }

    @Override protected void updateDriverAssistance(boolean freshShooterA) {
        odometry.update();
        manualDriveOverride = Math.max(Math.abs(gamepad1.left_stick_y),
                Math.max(Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.right_stick_x)))
                > RearHiveAim.MANUAL_OVERRIDE_DEADBAND;
        aim.update(odometry.getX(), odometry.getY(), odometry.getHeading(), odometry.isValid(),
                System.nanoTime() / 1e9, freshShooterA && gamepad2.back,
                manualDriveOverride || gamepad2.b || gamepad2.start);
    }

    @Override protected void driveRobot(MecanumDrive drive) {
        if (!manualDriveOverride && aim.controlsDriveThisLoop()) {
            drive.drive(0, 0, aim.getTurnPower());
        } else {
            super.driveRobot(drive);
        }
    }

    @Override protected boolean shooterStartAllowed() { return !gamepad2.back; }
    @Override protected boolean feedingAllowed() { return !aim.controlsDriveThisLoop(); }

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
        telemetry.addLine("A starts shooter; BACK + tap A aims rear at quadrant hive; B stops/cancels");
        telemetry.addLine("Drive sticks cancel aim; BACK + LB/LT adjusts target by 60 RPM");
        telemetry.addData("Rear hive aim", aim.getState());
        if (aim.getQuadrant() != null) {
            telemetry.addData("Aim hive (in)", "%.2f, %.2f",
                    aim.getQuadrant().hiveX, aim.getQuadrant().hiveY);
            telemetry.addData("Front heading target / error (deg)", "%.1f / %.1f",
                    aim.getTargetHeading(), aim.getHeadingError());
        }
        telemetry.addData("Aim clockwise turn power", "%.3f", aim.getTurnPower());
        odometry.addTelemetry(telemetry);
        telemetry.addLine("Provisional gains: tune P/F on hardware; I/D initially zero");
        telemetry.addData("Shooter PIDF state", shooterPidf.getStatus());
        telemetry.addData("PIDF sampled / target RPM", "%.0f / %.0f",
                shooterPidf.getCurrentRpm(), shooterPidf.getTargetRpm());
        telemetry.addData("Battery voltage during control (V)", "%.2f", shooterPidf.getBatteryVolts());
        telemetry.addData("PIDF / compensated / applied power", "%.3f / %.3f / %.3f",
                shooterPidf.getRawPower(), shooterPidf.getCompensatedPower(), shooterPidf.getAppliedPower());
        telemetry.addData("PIDF P / I / D / F", "%.6f / %.6f / %.6f / %.6f",
                Constants.SHOOTER_PIDF_P, Constants.SHOOTER_PIDF_I,
                Constants.SHOOTER_PIDF_D, Constants.SHOOTER_PIDF_F);
        telemetry.addLine("Feeder requires shooter ON; after a sensor fault, release A then press it to retry");
    }
}
