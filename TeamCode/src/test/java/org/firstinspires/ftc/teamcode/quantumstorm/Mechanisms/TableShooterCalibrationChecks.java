package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration.Solution;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration.Status;

/** Desktop checks using the real FTCLib interpolation implementation. */
public final class TableShooterCalibrationChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }

    private static void near(double actual, double expected, String why) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-7,
                why + ": expected " + expected + ", got " + actual);
    }

    public static void main(String[] args) {
        TableShooterCalibration table = new TableShooterCalibration();
        double[] distances = {30, 40, 47, 50, 60, 70};
        double[] rpms = {3000, 3100, 3120, 3200, 3300, 3400};
        for (int i = 0; i < distances.length; i++) {
            near(table.rpmForDistance(distances[i]), rpms[i], "Published interior table point");
        }
        // Independent Hermite calculations with endpoint tangent slopes from the published table.
        near(table.rpmForDistance(24.1), 2901.6721296296296, "Lower boundary interpolation");
        near(table.rpmForDistance(79.9), 3499, "Upper boundary interpolation");
        near(table.rpmForDistance(55), 3260.4166666666665, "Cubic interpolation, not linear");
        // Preserve the upstream library/table behavior, including its small dip around 42-44 in.
        near(table.rpmForDistance(42), 3104.499514091351, "Upstream curve at 42 in");
        near(table.rpmForDistance(44), 3102.3906705539357, "Upstream curve at 44 in");
        for (double d : new double[]{0, 1, 24, 24.1}) {
            near(table.rpmForDistance(d), table.rpmForDistance(24.1), "Near clamp");
        }
        for (double d : new double[]{79.9, 80, 120, Double.MAX_VALUE}) {
            near(table.rpmForDistance(d), 3499, "Far clamp");
        }
        double previousRpm = table.rpmForDistance(0);
        for (int i = 0; i <= 10000; i++) {
            double rpm = table.rpmForDistance(i / 100.0);
            check(Math.abs(rpm - previousRpm) < 1 && rpm >= 2900 && rpm <= 3500,
                    "Sweep is continuous, bounded and never throws at endpoints");
            previousRpm = rpm;
        }
        double exit = DistanceShooterCalibration.BALL_EXIT_OFFSET_INCHES;
        for (int xs : new int[]{-1, 1}) for (int ys : new int[]{-1, 1}) {
            Solution a = table.atPose(xs * 12.75, ys * 62.9, ys * 90);
            Solution b = table.atPose(xs * 12.75, ys * 57, ys * 90);
            double cx = xs * 36, cy = ys * 62.9;
            double aimedHeading = Math.toDegrees(Math.atan2(ys * 6 - cy, xs * 12.75 - cx)) + 180;
            Solution c = table.atPose(cx, cy, aimedHeading);
            near(a.distanceInches, 56.9 - exit, "A rear distance in mirrored quadrant");
            near(b.distanceInches, 51 - exit, "B rear distance in mirrored quadrant");
            near(c.distanceInches, Math.hypot(23.25, 56.9) - exit, "C aimed rear distance");
            for (Solution shot : new Solution[]{a, b, c}) {
                check(shot.ready() && shot.status == Status.READY, "Reference shot has table command");
                near(shot.quadrant.hiveX, xs * 12.75, "Hive X sign");
                near(shot.quadrant.hiveY, ys * 6, "Hive Y sign");
                near(shot.rpm, table.rpmForDistance(shot.distanceInches), "Geometry drives RPM");
            }
        }
        Solution side = table.atPose(-12.75, -57, 0);
        near(side.shooterX, -12.75 - exit, "Rear offset at zero heading");
        near(side.shooterY, -57, "Rear offset Y at zero heading");
        near(side.distanceInches, Math.hypot(exit, 51), "Unaligned distance uses actual shooter point");
        near(table.atPose(-12.75, -57, 90).distanceInches, 51 + exit, "Facing away puts exit farther away");
        near(table.atPose(-12.75, -57, 270).distanceInches, 51 - exit, "Wrapped heading");
        check(table.atPose(0, 0, 0).quadrant == Quadrant.NORTHEAST, "Axis tie uses positive side");
        Solution close = table.atPose(-12.75, -6 - exit, -90);
        check(close.ready() && close.status == Status.CLAMPED_NEAR, "Close shots use table clamp");
        near(close.distanceInches, 0, "Coincident exit has finite distance");
        Solution far = table.atPose(-72, -72, 45);
        check(far.ready() && far.status == Status.CLAMPED_FAR, "Far shots use table clamp");
        near(far.lookupDistanceInches, 79.9, "Telemetry exposes clamped distance");
        for (double bad : new double[]{Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY}) {
            for (Solution shot : new Solution[]{table.atPose(bad, -57, 0),
                    table.atPose(-12.75, bad, 0), table.atPose(-12.75, -57, bad)}) {
                check(!shot.ready() && shot.rpm == 0 && shot.status == Status.INVALID_POSE,
                        "Invalid pose or heading blocks shooter");
            }
            near(table.rpmForDistance(bad), 0, "Invalid distance has no RPM command");
        }
        near(table.rpmForDistance(-1), 0, "Negative distance rejected");
        for (double outside : new double[]{-72.01, 72.01}) {
            check(!table.atPose(outside, 0, 0).ready() && !table.atPose(0, outside, 0).ready(),
                    "Out-of-field pose rejected");
        }
        for (int x = -72; x <= 72; x += 6) for (int y = -72; y <= 72; y += 6) {
            for (int heading = -180; heading < 180; heading += 45) {
                Solution shot = table.atPose(x, y, heading);
                check(shot.ready() && shot.rpm >= 2900 && shot.rpm <= 3500, "Field command bounds");
                near(Math.hypot(shot.shooterX - x, shot.shooterY - y), exit, "Rotated exit stays at the measured offset");
            }
        }
        System.out.println("RPM table, cubic interpolation, clamps, A/B/C, headings, quadrants, invalid poses and field sweep passed.");
    }
}
