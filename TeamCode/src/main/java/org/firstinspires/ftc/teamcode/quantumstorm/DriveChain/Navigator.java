package org.firstinspires.ftc.teamcode.quantumstorm.DriveChain;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;

/**
 * Drives the mecanum chassis to field poses using Pinpoint odometry.
 *
 * moveTo() is non-blocking: call it every loop, it returns true once the
 * robot has arrived (and stops the motors). This fits the AutoStateMachine loop:
 *
 *     if (navigator.moveTo(Constants.RED_1_HIVE1_POSE)) state = State.SHOOTING;
 *     else if (navigator.isTimedOut()) state = State.ERROR;
 */
public class Navigator {

    private final MecanumDrive drive;
    private final PinpointOdometry odometry;

    private final ElapsedTime moveTimer = new ElapsedTime();
    private Pose2D currentTarget = null;

    private double distanceError = 0;
    private double headingError = 0;

    public Navigator(MecanumDrive drive, PinpointOdometry odometry) {
        this.drive = drive;
        this.odometry = odometry;
    }

    /** Update odometry. Call once at the top of every loop. */
    public void update() {
        odometry.update();
    }

    /**
     * Drive one step toward target.
     *
     * @return true when within tolerance of the target (motors are stopped)
     */
    public boolean moveTo(Pose2D target) {

        // New target -> restart the timeout clock
        if (target != currentTarget) {
            currentTarget = target;
            moveTimer.reset();
        }

        double x = odometry.getX();
        double y = odometry.getY();
        double heading = odometry.getHeading();

        // Field-frame errors
        double dx = target.getX(DistanceUnit.INCH) - x;
        double dy = target.getY(DistanceUnit.INCH) - y;
        distanceError = Math.hypot(dx, dy);
        headingError = AngleUnit.normalizeDegrees(target.getHeading(AngleUnit.DEGREES) - heading);

        if (distanceError < Constants.MOVE_POSITION_TOLERANCE_IN
                && Math.abs(headingError) < Constants.MOVE_HEADING_TOLERANCE_DEG) {
            drive.stop();
            return true;
        }

        // Rotate field-frame error into the robot frame (forward / left)
        double headingRad = Math.toRadians(heading);
        double forward = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
        double left = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);

        // Proportional control, translation capped at MOVE_MAX_POWER
        double forwardPower = forward * Constants.MOVE_KP_TRANSLATION;
        double leftPower = left * Constants.MOVE_KP_TRANSLATION;
        double translation = Math.hypot(forwardPower, leftPower);
        if (translation > Constants.MOVE_MAX_POWER) {
            forwardPower *= Constants.MOVE_MAX_POWER / translation;
            leftPower *= Constants.MOVE_MAX_POWER / translation;
        }

        // Heading is CCW positive; MecanumDrive turn is clockwise positive
        double turnPower = Range.clip(
                headingError * Constants.MOVE_KP_HEADING,
                -Constants.MOVE_MAX_POWER, Constants.MOVE_MAX_POWER);

        // MecanumDrive strafe is right positive
        drive.drive(forwardPower, -leftPower, -turnPower);
        return false;
    }

    /** True if the current moveTo() has run longer than MOVE_TIMEOUT_SEC. */
    public boolean isTimedOut() {
        return currentTarget != null && moveTimer.seconds() > Constants.MOVE_TIMEOUT_SEC;
    }

    /**
     * Drive relative to the robot instead of to a field pose (e.g. chasing a
     * ball seen by the camera). Cancels any moveTo() target.
     */
    public void driveRobotRelative(double forward, double strafeRight, double turnClockwise) {
        currentTarget = null;
        drive.drive(forward, strafeRight, turnClockwise);
    }

    /** Stop the motors and forget the target (the next moveTo() restarts its timeout). */
    public void stop() {
        drive.stop();
        currentTarget = null;
    }

    public void addTelemetry(Telemetry telemetry) {
        odometry.addTelemetry(telemetry);
        telemetry.addData("Distance to target (in)", "%.1f", distanceError);
        telemetry.addData("Heading error (deg)", "%.1f", headingError);
    }
}
