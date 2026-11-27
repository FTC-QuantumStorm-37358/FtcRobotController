package org.firstinspires.ftc.teamcode.biobuzz;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/**
 * Normal Team One parking from the Garden after a long continuous intake. A mecanum strafe
 * drifts forward a little, so the robot can pass beside the Flower/Hive gap waypoint without
 * entering its 3-inch circle. The first strafe must still stop at the gap instead of running
 * on toward the Hive and then reversing into a Hive post.
 */
public final class TeamOneParkRouteChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }

    /** Kinematic robot: 45 in/s at full power, strafes at 80% with 20% forward drift. */
    private static final class IO implements RobotIO {
        double time = 26;
        Pose pose;
        Outputs out = new Outputs();
        public double nowSeconds() { return time; }
        public boolean stopRequested() { return false; }
        public Sensors readSensors() {
            return new Sensors(pose, time, true, true, -1, Vision.empty(1, time));
        }
        public void apply(Outputs o) { out = o.copy(); }
        void move(double dt) {
            double v = 45, h = pose.heading;
            double fwd = (out.forward + .20 * Math.abs(out.strafeRight)) * v, right = out.strafeRight * .8 * v;
            pose = new Pose(pose.x + (fwd * Math.cos(h) + right * Math.sin(h)) * dt,
                    pose.y + (fwd * Math.sin(h) - right * Math.cos(h)) * dt, h);
        }
    }

    /**
     * Returns {farthest X reached toward the Hive, final parked depth along the reverse}; both
     * are signed so larger means farther (toward +X / +Y for Red, -X / -Y for Blue).
     */
    private static double[] parkFrom(int alliance, double startX, double stopEarlier) throws Exception {
        IO io = new IO();
        FieldLayout field = new FieldLayout();
        Pose garden = field.garden[alliance];
        io.pose = new Pose(startX, garden.y + (alliance == 0 ? 3 : -3), garden.heading);
        TeamOnePollenAuto auto = new TeamOnePollenAuto(io, field);
        auto.surveyBeforePickup = false;
        auto.parkStopEarlierInches = stopEarlier;
        auto.start(alliance, .48);
        Field current = TeamOnePollenAuto.class.getDeclaredField("current");
        current.setAccessible(true);
        current.set(auto, io.pose);
        Method park = TeamOnePollenAuto.class.getDeclaredMethod("park");
        park.setAccessible(true);
        park.invoke(auto);
        double towardHive = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < 100 && auto.isRunning(); i++) {
            io.time += .04;
            auto.tick();
            io.move(.04);
            towardHive = Math.max(towardHive, alliance == 0 ? io.pose.x : -io.pose.x);
        }
        check(!auto.isRunning(), "Parking finishes and turns outputs off");
        return new double[]{towardHive, alliance == 0 ? io.pose.y : -io.pose.y};
    }

    public static void main(String[] args) throws Exception {
        // gapX is -42 (Red) / +42 (Blue). The Hive lies beyond it; a few inches of coast is fine.
        for (int alliance = 0; alliance < 2; alliance++) {
            for (double offset : new double[]{-4, 0, 6}) {
                double startX = (alliance == 0 ? -58 : 58) + (alliance == 0 ? offset : -offset);
                double reach = parkFrom(alliance, startX, 0)[0];
                check(reach <= -42 + 4, "Alliance " + alliance + " from x=" + startX
                        + ": first strafe overshot the Flower/Hive gap toward the Hive (reached "
                        + String.format(java.util.Locale.US, "%.1f", alliance == 0 ? reach : -reach) + ")");
            }
            // Stopping 4 inches earlier parks the robot about 4 inches toward its front.
            double start = alliance == 0 ? -58 : 58;
            double moved = parkFrom(alliance, start, 0)[1] - parkFrom(alliance, start, 4)[1];
            check(moved >= 2 && moved <= 6, "Alliance " + alliance + ": parkStopEarlierInches=4 moved the parked robot "
                    + String.format(java.util.Locale.US, "%.1f", moved) + " in");
        }
        System.out.println("Team One parking stops each strafe at the Flower/Hive gap and parks earlier when asked.");
    }
}
