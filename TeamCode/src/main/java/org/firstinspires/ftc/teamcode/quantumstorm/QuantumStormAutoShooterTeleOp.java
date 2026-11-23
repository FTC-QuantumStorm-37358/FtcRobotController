package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration.Solution;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;

/** Driver aims and starts/stops the shooter; absolute field odometry selects quadrant and RPM. */
@TeleOp(name = "Biobuzz-Quantum Storm Auto-Shooter-Power", group = "Quantum Storm")
public class QuantumStormAutoShooterTeleOp extends QuantumStormTeleOp {
    private final TableShooterCalibration calibration = new TableShooterCalibration();
    private PinpointOdometry odometry;
    private Quadrant start = Quadrant.SOUTHWEST;
    private Solution solution;

    @Override protected void initializeShooterTarget() {
        odometry = new PinpointOdometry(hardwareMap, startPose());
    }

    private Pose2D startPose() {
        return new Pose2D(DistanceUnit.INCH, start.startX, start.startY,
                AngleUnit.DEGREES, start.startHeadingDegrees);
    }

    @Override protected void waitForDriverStart() {
        while (!isStarted() && !isStopRequested()) {
            // INIT only: left/right choose the X sign, down/up choose the Y sign.
            double x = start.startX, y = start.startY;
            if (gamepad1.dpad_left != gamepad1.dpad_right) x = gamepad1.dpad_left ? -12.75 : 12.75;
            if (gamepad1.dpad_down != gamepad1.dpad_up) y = gamepad1.dpad_down ? -62.9 : 62.9;
            Quadrant selected = Quadrant.at(x, y);
            if (selected != start) {
                start = selected;
                odometry.setPose(startPose());
            }
            updateShooterTarget(0);
            telemetry.addLine("INIT: gamepad 1 D-pad LEFT/RIGHT = X sign; DOWN/UP = Y sign");
            telemetry.addData("Place robot at start (in / deg)", "%.2f, %.2f / %.0f",
                    start.startX, start.startY, start.startHeadingDegrees);
            telemetry.addLine("Keep still for Pinpoint. Shooter and feeder stay OFF during INIT.");
            addShooterTargetTelemetry();
            telemetry.update();
            idle();
        }
    }

    @Override protected double updateShooterTarget(double currentSetting) {
        solution = null;
        odometry.update();
        if (!odometry.isValid()) return 0;
        solution = calibration.atPose(odometry.getX(), odometry.getY(), odometry.getHeading());
        return shooterTargetReady() ? solution.rpm / Constants.SHOOTER_BASE_RPM : 0;
    }

    @Override protected boolean manualShooterAdjustment() { return false; }
    @Override protected boolean shooterTargetReady() { return solution != null && solution.ready(); }

    @Override protected void addShooterTargetTelemetry() {
        telemetry.addLine("AUTO SHOOTER: aim rear shooter at hive; A starts, B stops; D-pad feeds");
        telemetry.addLine("Stealth RPM table / fixed hood: starting calibration; aim before feeding");
        telemetry.addData("Start reference", start);
        if (solution != null && solution.quadrant != null) {
            telemetry.addData("Hive quadrant", solution.quadrant);
            telemetry.addData("Hive target (in)", "%.2f, %.2f",
                    solution.quadrant.hiveX, solution.quadrant.hiveY);
            telemetry.addData("Shooter-to-hive distance (in)", "%.2f", solution.distanceInches);
            telemetry.addData("Rear shooter position (in)", "%.2f, %.2f",
                    solution.shooterX, solution.shooterY);
            telemetry.addData("Table lookup distance (in)", "%.2f", solution.lookupDistanceInches);
            telemetry.addData("Calculated RPM / multiplier", "%.0f / %.4f",
                    solution.rpm, solution.rpm / Constants.SHOOTER_BASE_RPM);
        }
        telemetry.addData("Shot solution", solution == null ? "NO VALID ODOMETRY" : solution.status);
        if (!shooterTargetReady()) {
            telemetry.addLine("Shooter/feed blocked; press A again after a valid shot solution returns");
        }
        odometry.addTelemetry(telemetry);
    }
}
