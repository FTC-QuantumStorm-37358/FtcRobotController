package org.firstinspires.ftc.teamcode.quantumstorm.DriveChain;

import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;

/** One nonblocking, shortest-path turn to aim a centered rear shooter at the same-quadrant hive. */
public final class RearHiveAim {
    public static final double TOLERANCE_DEGREES = 1;
    public static final double SETTLE_SECONDS = .20;
    public static final double TIMEOUT_SECONDS = 5;
    public static final double KP = .015;
    public static final double MIN_TURN_POWER = .06;
    public static final double MAX_TURN_POWER = .25;
    public static final double MANUAL_OVERRIDE_DEADBAND = .10;

    public enum State { IDLE, TURNING, SETTLING, ALIGNED, CANCELLED, INVALID_POSE, TOO_CLOSE, TIMED_OUT }

    private State state = State.IDLE;
    private Quadrant quadrant;
    private double targetHeading = Double.NaN, headingError = Double.NaN, turnPower;
    private double startedAt, lastUpdate, settledSince = Double.NaN;
    private boolean controlsDrive;

    /** Heading is front-facing, degrees CCW; returned wheel-mixer turn power is clockwise positive. */
    public double update(double x, double y, double heading, boolean validPose, double now,
                         boolean request, boolean cancel) {
        boolean wasActive = active();
        controlsDrive = wasActive || request;
        turnPower = 0;
        if (cancel) {
            if (controlsDrive) state = State.CANCELLED;
            return 0;
        }
        if (request) {
            state = State.TURNING;
            startedAt = now;
            lastUpdate = now;
            settledSince = Double.NaN;
            quadrant = null;
        }
        if (!active()) return 0;
        if (!validPose || !Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)
                || Math.abs(x) > 72 || Math.abs(y) > 72 || !Double.isFinite(now) || now < lastUpdate) {
            state = State.INVALID_POSE;
            return 0;
        }
        lastUpdate = now;
        if (now - startedAt >= TIMEOUT_SECONDS) {
            state = State.TIMED_OUT;
            return 0;
        }
        Quadrant next = Quadrant.at(x, y);
        if (next != quadrant) settledSince = Double.NaN;
        quadrant = next;
        double dx = quadrant.hiveX - x, dy = quadrant.hiveY - y;
        if (Math.hypot(dx, dy) <= DistanceShooterCalibration.BALL_EXIT_OFFSET_INCHES) {
            state = State.TOO_CLOSE;
            return 0;
        }
        targetHeading = rearFacingHeading(x, y, quadrant.hiveX, quadrant.hiveY);
        headingError = wrapDegrees(targetHeading - heading);
        if (Math.abs(headingError) <= TOLERANCE_DEGREES) {
            if (!Double.isFinite(settledSince)) settledSince = now;
            state = now - settledSince >= SETTLE_SECONDS ? State.ALIGNED : State.SETTLING;
            return 0;
        }
        settledSince = Double.NaN;
        state = State.TURNING;
        double magnitude = Math.min(MAX_TURN_POWER, Math.max(MIN_TURN_POWER, Math.abs(headingError) * KP));
        turnPower = -Math.copySign(magnitude, headingError);
        return turnPower;
    }

    /** Robot front points away from the hive; an 8-inch offset along the rear axis has the same aim line. */
    public static double rearFacingHeading(double x, double y, double hiveX, double hiveY) {
        return wrapDegrees(Math.toDegrees(Math.atan2(hiveY - y, hiveX - x)) + 180);
    }

    public static double wrapDegrees(double degrees) {
        double wrapped = degrees % 360;
        if (wrapped >= 180) wrapped -= 360;
        if (wrapped < -180) wrapped += 360;
        return wrapped;
    }

    public boolean active() { return state == State.TURNING || state == State.SETTLING; }
    // Includes the completion/fault/cancel loop so the previous turn command is explicitly cleared.
    public boolean controlsDriveThisLoop() { return controlsDrive; }
    public State getState() { return state; }
    public Quadrant getQuadrant() { return quadrant; }
    public double getTargetHeading() { return targetHeading; }
    public double getHeadingError() { return headingError; }
    public double getTurnPower() { return turnPower; }
}
