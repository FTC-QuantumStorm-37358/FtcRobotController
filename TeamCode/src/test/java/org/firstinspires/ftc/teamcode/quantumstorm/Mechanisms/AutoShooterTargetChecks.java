package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;

/** Distance geometry, reference calibration and persistent relative trim with real FTCLib interpolation. */
public final class AutoShooterTargetChecks {
    private static void check(boolean value, String why) { if (!value) throw new AssertionError(why); }
    private static void near(double actual, double expected, String why) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-9,
                why + ": expected " + expected + ", got " + actual);
    }
    public static void main(String[] args) {
        AutoShooterTarget target = new AutoShooterTarget();
        TableShooterCalibration table = new TableShooterCalibration();
        check(!target.ready(), "Starts without a valid sample");
        for (Quadrant q : Quadrant.values()) {
            target.updatePose(true, q.startX, q.startY, q.startHeadingDegrees);
            near(target.getSolution().distanceInches, 51.9, "Default shooter distance in every quadrant");
            near(target.getBaseMultiplier(), .495, "Default base multiplier in every quadrant");
            near(target.getMultiplier() * 6000, 2970, "Default RPM in every quadrant");
            check(target.getSolution().quadrant == q, "Correct closest hive");
        }
        // A, B, C in all quadrants, with rear pointing toward the selected hive.
        for (int xs : new int[]{-1, 1}) for (int ys : new int[]{-1, 1}) {
            for (double[] point : new double[][]{{12.75, 63.25}, {12.75, 57}, {36, 62.9}}) {
                double x = xs * point[0], y = ys * point[1];
                double heading = Math.toDegrees(Math.atan2(ys * 6 - y, xs * 12.75 - x)) + 180;
                target.updatePose(true, x, y, heading);
                double distance = Math.hypot(x - xs * 12.75, y - ys * 6) - 5.35;
                near(target.getSolution().distanceInches, distance, "A/B/C geometry");
                near(target.getBaseMultiplier() * 6000,
                        2970 + table.rpmForDistance(distance) - table.rpmForDistance(51.9),
                        "Distance curve retains existing RPM differences");
            }
        }
        target.adjustTrim(1);
        target.adjustTrim(1);
        near(target.getTrimMultiplier(), .010, "Two presses add 60 RPM");
        for (int x = -72; x <= 72; x += 6) for (int y = -72; y <= 72; y += 6) {
            for (int heading = -180; heading < 180; heading += 45) {
                target.updatePose(true, x, y, heading);
                check(target.ready(), "Finite field pose produces a target");
                near(target.getMultiplier() - target.getBaseMultiplier(), .010,
                        "Same relative trim at every pose and heading");
                check(target.getMultiplier() > 0 && target.getMultiplier() < 1, "Field target bounds");
            }
        }
        target.updatePose(true, -12.75, -63.25, -90);
        for (int i = 0; i < 1000; i++) target.adjustTrim(1);
        near(target.getMultiplier(), 1, "Upper clipping after trim");
        near(target.getTrimMultiplier(), 1, "Trim storage bounded");
        for (int i = 0; i < 2000; i++) target.adjustTrim(-1);
        near(target.getMultiplier(), 0, "Lower clipping never reverses shooter");
        near(target.getTrimMultiplier(), -1, "Negative trim storage bounded");
        target.reset();
        check(!target.ready(), "Re-INIT clears readiness");
        near(target.getTrimMultiplier(), 0, "Re-INIT clears trim");
        target.adjustTrim(1);
        for (double bad : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -72.1, 72.1}) {
            target.updatePose(true, bad, -63.25, -90);
            check(!target.ready(), "Invalid pose blocks readiness");
            near(target.getMultiplier(), 0, "Positive trim cannot override invalid pose");
        }
        target.updatePose(false, -12.75, -63.25, -90);
        near(target.getMultiplier(), 0, "Invalid Pinpoint sample blocks despite valid last-known pose");
        target.updatePose(true, -12.75, -63.25, -90);
        near(target.getMultiplier(), .500, "Recovery restores distance target plus existing trim");
        target.adjustTrim(-1);
        near(target.getMultiplier(), .495, "One negative press exactly cancels positive trim");
        System.out.println("Default 2970 RPM, mirrored geometry, table bias, persistent trim, limits and invalid-pose checks passed.");
    }
}
