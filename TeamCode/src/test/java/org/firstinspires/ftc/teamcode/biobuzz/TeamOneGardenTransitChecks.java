package org.firstinspires.ftc.teamcode.biobuzz;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** The faster Garden trip applies only to the retry branch and ends each leg at the usual power. */
public final class TeamOneGardenTransitChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }

    private static void set(Object target, String name, Object value) throws Exception {
        Field field = TeamOnePollenAuto.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(target, value);
    }

    public static void main(String[] args) throws Exception {
        RobotIO io = new RobotIO() {
            public double nowSeconds() { return 0; }
            public Sensors readSensors() { return null; }
            public void apply(Outputs outputs) { }
        };
        TeamOnePollenAuto auto = new TeamOnePollenAuto(io, new FieldLayout());
        auto.gardenTransitPower = .70;
        auto.gardenRetryTransitPower = .85;
        auto.start(0, .48); // validation accepts the faster retry power
        Method power = TeamOnePollenAuto.class.getDeclaredMethod("gardenOutboundPower", double.class);
        power.setAccessible(true);
        Pose legStart = new Pose(-12.75, -50, -Math.PI / 2);
        set(auto, "gardenLegStart", legStart);
        set(auto, "current", legStart);

        check((double) power.invoke(auto, 45.0) == .70, "Without a retry, the Garden trip keeps 0.70");
        set(auto, "repeatStartShotAfterGarden", true);
        check((double) power.invoke(auto, 45.0) == .85, "Retry branch cruises at the faster power");
        set(auto, "current", new Pose(-12.75 - 40, -50, -Math.PI / 2));
        check((double) power.invoke(auto, 45.0) == .70, "Last 6 inches of the leg return to 0.70");

        auto.gardenRetryTransitPower = 1.2;
        boolean rejected = false;
        try { auto.start(0, .48); } catch (IllegalArgumentException e) { rejected = true; }
        check(rejected, "Retry power above 1 is rejected");
        System.out.println("Team One retry Garden transit speed checks passed.");
    }
}
