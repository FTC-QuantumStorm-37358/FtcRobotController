package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.RearHiveAim;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.RearHiveAim.State;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;

/** Pure geometry and timed state-machine checks; no robot or blocking waits. */
public final class RearHiveAimChecks {
    private static void check(boolean value, String why) { if (!value) throw new AssertionError(why); }
    private static void near(double actual, double expected, String why) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-9, why + ": " + actual);
    }
    private static void rearPointsAtHive(double x, double y, double heading, Quadrant hive) {
        double dx = hive.hiveX - x, dy = hive.hiveY - y, distance = Math.hypot(dx, dy);
        double rear = Math.toRadians(heading + 180);
        near(Math.cos(rear), dx / distance, "Rear X points at hive");
        near(Math.sin(rear), dy / distance, "Rear Y points at hive");
    }
    private static RearHiveAim started() {
        RearHiveAim aim = new RearHiveAim();
        aim.update(-36, -62.9, -90, true, 0, true, false);
        check(aim.active() && aim.getTurnPower() > 0, "C turns clockwise from -90 degrees");
        return aim;
    }
    public static void main(String[] args) {
        near(RearHiveAim.rearFacingHeading(-12.75, -62.9, -12.75, -6), -90, "A rear aim");
        near(RearHiveAim.wrapDegrees(181), -179, "Positive wrap");
        near(RearHiveAim.wrapDegrees(-181), 179, "Negative wrap");
        near(RearHiveAim.wrapDegrees(540), -180, "Half-turn tie");
        for (int xs : new int[]{-1, 1}) for (int ys : new int[]{-1, 1}) {
            double x = xs * 36, y = ys * 62.9;
            Quadrant hive = Quadrant.at(x, y);
            double target = RearHiveAim.rearFacingHeading(x, y, hive.hiveX, hive.hiveY);
            rearPointsAtHive(x, y, target, hive);
            for (double heading : new double[]{target - 179, target - 10, target + 10, target + 179}) {
                RearHiveAim aim = new RearHiveAim();
                double power = aim.update(x, y, heading, true, 0, true, false);
                double error = RearHiveAim.wrapDegrees(target - heading);
                check(power * error < 0, "Convert CCW heading error to clockwise wheel command");
                check(Math.abs(power) <= .25, "Turn power capped");
                // Simulated heading update: the drive mixer's positive power turns clockwise.
                double nextHeading = heading - power * 90 * .02;
                check(Math.abs(RearHiveAim.wrapDegrees(target - nextHeading)) < Math.abs(error),
                        "Command reduces shortest-path heading error");
            }
            // Entire closed-loop turn with a simple 360 deg/s full-power plant.
            RearHiveAim aim = new RearHiveAim();
            double heading = target + 120;
            for (int i = 0; i < 250; i++) {
                double power = aim.update(x, y, heading, true, i * .02, i == 0, false);
                heading = RearHiveAim.wrapDegrees(heading - power * 360 * .02);
                if (aim.getState() == State.ALIGNED) break;
            }
            check(aim.getState() == State.ALIGNED, "Mirrored turn settles before timeout");
            check(Math.abs(RearHiveAim.wrapDegrees(target - heading)) <= 1, "Settled heading tolerance");
        }
        RearHiveAim aim = started();
        double target = aim.getTargetHeading();
        near(aim.update(-36, -62.9, target, true, .1, false, false), 0, "Stop turning inside tolerance");
        check(aim.getState() == State.SETTLING, "Require dwell, not one passing sample");
        aim.update(-36, -62.9, target + 2, true, .2, false, false);
        aim.update(-36, -62.9, target, true, .3, false, false);
        check(aim.getState() == State.SETTLING, "Leaving tolerance restarts dwell");
        aim.update(-36, -62.9, target, true, .51, false, false);
        check(aim.getState() == State.ALIGNED && aim.controlsDriveThisLoop(), "Completion explicitly clears drive output");
        aim.update(-36, -62.9, target + 10, true, .6, false, false);
        check(!aim.active() && !aim.controlsDriveThisLoop(), "One-shot assist does not restart without a new request");
        for (boolean valid : new boolean[]{true, false}) {
            aim = started();
            near(aim.update(-36, -62.9, -90, valid, .1, false, true), 0, "Cancel immediately stops turn");
            check(aim.getState() == State.CANCELLED && !aim.active(), "Cancel wins over pose fault");
        }
        aim = new RearHiveAim();
        aim.update(-36, -62.9, -90, true, 0, true, true);
        check(aim.getState() == State.CANCELLED, "B/stick/START cancellation wins over simultaneous request");
        aim = started();
        near(aim.update(-36, -62.9, -90, false, .1, false, false), 0, "Invalid odometry stops");
        check(aim.getState() == State.INVALID_POSE, "Pose fault visible");
        aim.update(-36, -62.9, -90, true, .2, false, false);
        check(!aim.active(), "Pose recovery needs fresh request");
        for (double[] bad : new double[][]{{Double.NaN, -57, 0}, {-36, Double.POSITIVE_INFINITY, 0},
                {-36, -57, Double.NaN}, {73, -57, 0}}) {
            aim = new RearHiveAim();
            near(aim.update(bad[0], bad[1], bad[2], true, 0, true, false), 0, "Nonfinite/outside pose rejected");
            check(aim.getState() == State.INVALID_POSE, "Invalid geometry status");
        }
        aim = started();
        near(aim.update(-36, -62.9, -90, true, 5, false, false), 0, "Timeout stops a stalled/reversed robot");
        check(aim.getState() == State.TIMED_OUT, "Timeout visible");
        aim = new RearHiveAim();
        aim.update(-12.75, -6, 0, true, 0, true, false);
        check(aim.getState() == State.TOO_CLOSE && aim.getTurnPower() == 0, "Undefined aim at hive rejected");
        aim = new RearHiveAim();
        aim.update(0, -62.9, 0, true, 0, true, false);
        check(aim.getQuadrant() == Quadrant.SOUTHEAST, "Same axis tie as distance shooter");
        aim.update(-.1, -62.9, 0, true, .1, false, false);
        check(aim.getQuadrant() == Quadrant.SOUTHWEST, "Recompute quadrant after position change");
        System.out.println("Rear-hive geometry, turn signs, wraparound, mirrored closed-loop turns, settling, cancel, faults and timeout passed.");
    }
}
