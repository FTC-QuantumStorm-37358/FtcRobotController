package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

/** Ideal projectile model calibrated to the two straight shots, with an affine ball-speed/RPM fit. */
public final class DistanceShooterCalibration {
    public static final double GRAVITY_INCHES_PER_SECOND_SQUARED = 386.09;
    public static final double LAUNCH_ANGLE_RADIANS = Math.toRadians(70);
    public static final double LAUNCH_HEIGHT_INCHES = 8;
    public static final double TARGET_HEIGHT_INCHES = 60;
    /** Legacy A/B physics fit only: its calibration shots were measured with this 8" offset. */
    public static final double SHOOTER_OFFSET_INCHES = 8;
    /** Measured ball center at exit: 3.4" inside the rear edge of the 17.5" robot. */
    public static final double BALL_EXIT_OFFSET_INCHES = 17.5 / 2 - 3.4;
    public static final double MAX_RPM = 6000;
    /** TeleOp start: robot front against the Y wall; center is half the 17.5" length from it. */
    public static final double START_Y_INCHES = 72 - 17.5 / 2;
    public static final double MIN_CALIBRATED_DISTANCE = 43;
    public static final double MAX_CALIBRATED_DISTANCE = 48.9;
    public static final double MIN_FALLING_DISTANCE = 2 * (TARGET_HEIGHT_INCHES - LAUNCH_HEIGHT_INCHES)
            / Math.tan(LAUNCH_ANGLE_RADIANS);

    // A: 48.9 inches / 3000 RPM; B: 43 inches / 2940 RPM. The discarded .54 shot is not used.
    public static final double SPEED_PER_RPM = (requiredBallSpeed(MAX_CALIBRATED_DISTANCE)
            - requiredBallSpeed(MIN_CALIBRATED_DISTANCE)) / (3000 - 2940);
    public static final double SPEED_INTERCEPT = requiredBallSpeed(MIN_CALIBRATED_DISTANCE)
            - SPEED_PER_RPM * 2940;

    /** Mirrored hive back centers. Axis ties deterministically use the positive side. */
    public enum Quadrant {
        SOUTHWEST(-1, -1), SOUTHEAST(1, -1), NORTHWEST(-1, 1), NORTHEAST(1, 1);
        public final double hiveX, hiveY, startX, startY, startHeadingDegrees;

        Quadrant(int xSign, int ySign) {
            hiveX = xSign * 12.75;
            hiveY = ySign * 6;
            startX = xSign * 12.75;
            startY = ySign * START_Y_INCHES;
            // Rear-mounted shooter faces into the field; robot front faces the Y wall.
            startHeadingDegrees = ySign * 90;
        }

        public static Quadrant at(double x, double y) {
            if (!Double.isFinite(x) || !Double.isFinite(y)) return null;
            return y < 0 ? (x < 0 ? SOUTHWEST : SOUTHEAST)
                    : (x < 0 ? NORTHWEST : NORTHEAST);
        }
    }

    public enum Status { READY, INVALID_POSE, TOO_CLOSE_FOR_FALLING_SHOT, RPM_OUT_OF_RANGE }

    public static final class Solution {
        public final Quadrant quadrant;
        public final double distanceInches, rpm;
        public final Status status;

        private Solution(Quadrant quadrant, double distance, double rpm, Status status) {
            this.quadrant = quadrant;
            this.distanceInches = distance;
            this.rpm = rpm;
            this.status = status;
        }

        public boolean ready() { return status == Status.READY; }
        public boolean calibratedRange() {
            return ready() && distanceInches >= MIN_CALIBRATED_DISTANCE
                    && distanceInches <= MAX_CALIBRATED_DISTANCE;
        }
    }

    private DistanceShooterCalibration() { }

    /** Select the hive in the robot-center quadrant, then subtract the aimed ball-exit offset. */
    public static Solution atPosition(double x, double y) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || Math.abs(x) > 72 || Math.abs(y) > 72) {
            return new Solution(null, Double.NaN, 0, Status.INVALID_POSE);
        }
        Quadrant quadrant = Quadrant.at(x, y);
        double distance = Math.max(0, Math.hypot(x - quadrant.hiveX, y - quadrant.hiveY)
                - SHOOTER_OFFSET_INCHES);
        if (distance <= MIN_FALLING_DISTANCE) {
            return new Solution(quadrant, distance, 0, Status.TOO_CLOSE_FOR_FALLING_SHOT);
        }
        double rpm = rpmForDistance(distance);
        return new Solution(quadrant, distance, rpm,
                rpm > 0 ? Status.READY : Status.RPM_OUT_OF_RANGE);
    }

    /** Inches/second at release. Reject the ascending branch and apex-only arrival. */
    public static double requiredBallSpeed(double distance) {
        if (!Double.isFinite(distance) || distance <= MIN_FALLING_DISTANCE) return Double.NaN;
        double height = TARGET_HEIGHT_INCHES - LAUNCH_HEIGHT_INCHES;
        double cosine = Math.cos(LAUNCH_ANGLE_RADIANS);
        return Math.sqrt(GRAVITY_INCHES_PER_SECOND_SQUARED * distance * distance
                / (2 * cosine * cosine * (distance * Math.tan(LAUNCH_ANGLE_RADIANS) - height)));
    }

    /** Zero means no achievable falling-shot command; do not clamp to a different trajectory. */
    public static double rpmForDistance(double distance) {
        double rpm = (requiredBallSpeed(distance) - SPEED_INTERCEPT) / SPEED_PER_RPM;
        return Double.isFinite(rpm) && rpm > 0 && rpm <= MAX_RPM ? rpm : 0;
    }
}
