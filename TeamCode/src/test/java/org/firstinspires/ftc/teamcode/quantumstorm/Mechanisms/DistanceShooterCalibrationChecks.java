package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Solution;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Status;

/** Checks shot measurements, all field quadrants, falling arrivals, and impossible solutions. */
public final class DistanceShooterCalibrationChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }
    private static void near(double actual, double expected, double tolerance, String why) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) <= tolerance, why + ": " + actual);
    }
    private static void fallingArrival(Solution solution) {
        double speed = DistanceShooterCalibration.SPEED_PER_RPM * solution.rpm
                + DistanceShooterCalibration.SPEED_INTERCEPT;
        double angle = Math.toRadians(70), gravity = 386.09;
        double time = solution.distanceInches / (speed * Math.cos(angle));
        // Independently integrate the requested flight, rather than reusing the inverse equation.
        near(8 + speed * Math.sin(angle) * time - gravity * time * time / 2, 60, 1e-8,
                "Trajectory reaches target height");
        check(speed * Math.sin(angle) - gravity * time < 0, "Ball arrives while falling");
    }

    public static void main(String[] args) {
        for (int xs : new int[]{-1, 1}) for (int ys : new int[]{-1, 1}) {
            Solution a = DistanceShooterCalibration.atPosition(xs * 12.75, ys * 62.9);
            Solution b = DistanceShooterCalibration.atPosition(xs * 12.75, ys * 57);
            Solution c = DistanceShooterCalibration.atPosition(xs * 36, ys * 62.9);
            for (Solution shot : new Solution[]{a, b, c}) {
                check(shot.ready(), "Reference shot ready in all quadrants");
                near(shot.quadrant.hiveX, xs * 12.75, 0, "Select same-X hive");
                near(shot.quadrant.hiveY, ys * 6, 0, "Select same-Y hive");
                fallingArrival(shot);
            }
            near(a.distanceInches, 48.9, 1e-10, "A ball-exit distance");
            near(a.rpm / 6000, .50, 1e-12, "A measured multiplier");
            near(b.distanceInches, 43, 1e-10, "B ball-exit distance");
            near(b.rpm / 6000, .49, 1e-12, "B measured multiplier");
            near(c.distanceInches, 53.466840654128, 1e-8, "Corrected C distance");
            near(c.rpm / 6000, .50986677994, 1e-9, "Physics prediction at C");
            check(a.calibratedRange() && b.calibratedRange() && !c.calibratedRange(), "Extrapolation flag");
            near(a.quadrant.startX, xs * 12.75, 0, "Start X");
            near(a.quadrant.startY, ys * 62.9, 0, "Start Y");
            near(a.quadrant.startHeadingDegrees, ys * 90, 0, "Wall-facing heading");
        }
        check(Quadrant.at(0, 0) == Quadrant.NORTHEAST, "Origin tie is positive");
        check(Quadrant.at(0, -62.9) == Quadrant.SOUTHEAST, "X axis tie");
        check(Quadrant.at(-62.9, 0) == Quadrant.NORTHWEST, "Y axis tie");
        near(DistanceShooterCalibration.atPosition(-1e-8, -62.9).rpm,
                DistanceShooterCalibration.atPosition(1e-8, -62.9).rpm, 1e-9, "Continuous RPM across X axis");
        near(DistanceShooterCalibration.atPosition(-62.9, -1e-8).rpm,
                DistanceShooterCalibration.atPosition(-62.9, 1e-8).rpm, 1e-9, "Continuous RPM across Y axis");
        for (double bad : new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, -73, 73}) {
            for (Solution shot : new Solution[]{DistanceShooterCalibration.atPosition(bad, -57),
                    DistanceShooterCalibration.atPosition(-12.75, bad)}) {
                check(shot.status == Status.INVALID_POSE && shot.rpm == 0, "Bad pose stops shooter");
            }
        }
        for (double badDistance : new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, -1, 0, 30, DistanceShooterCalibration.MIN_FALLING_DISTANCE, 10000}) {
            near(DistanceShooterCalibration.rpmForDistance(badDistance), 0, 0, "Impossible shot stops");
        }
        check(DistanceShooterCalibration.atPosition(-12.75, -6).status == Status.TOO_CLOSE_FOR_FALLING_SHOT,
                "Reject shooter at hive center");
        check(DistanceShooterCalibration.atPosition(-12.75, -40).status == Status.TOO_CLOSE_FOR_FALLING_SHOT,
                "Reject ascending-only trajectory");
        int valid = 0;
        for (int x = -72; x <= 72; x += 3) for (int y = -72; y <= 72; y += 3) {
            Solution shot = DistanceShooterCalibration.atPosition(x, y);
            double selectedDistance = Math.hypot(x - shot.quadrant.hiveX, y - shot.quadrant.hiveY);
            for (Quadrant other : Quadrant.values()) {
                check(selectedDistance <= Math.hypot(x - other.hiveX, y - other.hiveY) + 1e-9,
                        "Same-quadrant hive is nearest for mirrored centers");
            }
            if (shot.ready()) {
                check(shot.rpm > 0 && shot.rpm <= 6000, "Motor RPM bounds");
                fallingArrival(shot);
                valid++;
            } else check(shot.rpm == 0, "Invalid solution commands zero");
        }
        System.out.println("A/B/C, quadrant selection, axis ties, invalid inputs, bounds and " + valid
                + " field-grid falling trajectories passed.");
    }
}
