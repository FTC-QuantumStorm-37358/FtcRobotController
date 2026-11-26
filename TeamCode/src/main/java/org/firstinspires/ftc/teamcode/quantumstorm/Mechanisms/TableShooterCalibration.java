package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.arcrobotics.ftclib.util.InterpLUT;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;

/**
 * Starting RPM calibration from Stealth Robotics' AutonomousPrime2026 ShotCalculator.
 * Source commit: 7259c012a7725d9c79e0abc5648350a549c068e2. See AUTO_SHOOTER_CALIBRATION.md.
 * Their table accompanies an adjustable hood; these speeds need testing with our fixed hood.
 */
public final class TableShooterCalibration {
    public static final double MIN_LOOKUP_DISTANCE = 24.1;
    public static final double MAX_LOOKUP_DISTANCE = 79.9;
    private static final double REAR_OFFSET_INCHES = DistanceShooterCalibration.BALL_EXIT_OFFSET_INCHES;
    private final InterpLUT rpmTable = new InterpLUT();

    public TableShooterCalibration() {
        rpmTable.add(24, 2900);
        rpmTable.add(30, 3000);
        rpmTable.add(40, 3100);
        rpmTable.add(47, 3120);
        // Team calibration lowered these neighbors from the original Stealth table.
        // Latest TeleOp and autonomous apply their start-reference bias in AutoShooterTarget.
        rpmTable.add(50, 2910);
        rpmTable.add(60, 2935);
        rpmTable.add(70, 3300);
        rpmTable.add(80, 3400);
        rpmTable.createLUT();
    }

    public enum Status { READY, CLAMPED_NEAR, CLAMPED_FAR, INVALID_POSE }

    public static final class Solution {
        public final Quadrant quadrant;
        public final double shooterX, shooterY, distanceInches, lookupDistanceInches, rpm;
        public final Status status;

        private Solution(Quadrant quadrant, double shooterX, double shooterY,
                         double distance, double lookupDistance, double rpm, Status status) {
            this.quadrant = quadrant;
            this.shooterX = shooterX;
            this.shooterY = shooterY;
            this.distanceInches = distance;
            this.lookupDistanceInches = lookupDistance;
            this.rpm = rpm;
            this.status = status;
        }

        /** A valid RPM command, not a guarantee that the robot is aimed or the ball will land. */
        public boolean ready() { return status != Status.INVALID_POSE; }
    }

    /** Field inches; heading in degrees, counterclockwise from +X, describing the robot FRONT. */
    public Solution atPose(double x, double y, double headingDegrees) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(headingDegrees)
                || Math.abs(x) > 72 || Math.abs(y) > 72) {
            return new Solution(null, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                    0, Status.INVALID_POSE);
        }
        Quadrant quadrant = Quadrant.at(x, y);
        double heading = Math.toRadians(headingDegrees % 360);
        // Rotate the rear ball-exit offset into the field frame before measuring distance.
        double shooterX = x - REAR_OFFSET_INCHES * Math.cos(heading);
        double shooterY = y - REAR_OFFSET_INCHES * Math.sin(heading);
        double distance = Math.hypot(quadrant.hiveX - shooterX, quadrant.hiveY - shooterY);
        double lookupDistance = clampDistance(distance);
        Status status = distance < MIN_LOOKUP_DISTANCE ? Status.CLAMPED_NEAR
                : distance > MAX_LOOKUP_DISTANCE ? Status.CLAMPED_FAR : Status.READY;
        return new Solution(quadrant, shooterX, shooterY, distance, lookupDistance,
                rpmForDistance(distance), status);
    }

    /** Match Stealth's endpoint clamping; FTCLib's LUT excludes its exact 24/80 endpoints. */
    public double rpmForDistance(double distanceInches) {
        if (!Double.isFinite(distanceInches) || distanceInches < 0) return 0;
        return rpmTable.get(clampDistance(distanceInches));
    }

    private static double clampDistance(double distance) {
        return Math.max(MIN_LOOKUP_DISTANCE, Math.min(MAX_LOOKUP_DISTANCE, distance));
    }
}
