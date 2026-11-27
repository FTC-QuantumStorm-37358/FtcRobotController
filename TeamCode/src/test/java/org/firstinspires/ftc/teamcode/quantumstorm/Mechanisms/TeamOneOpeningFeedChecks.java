package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import java.lang.reflect.Proxy;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.biobuzz.FieldLayout;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO;
import org.firstinspires.ftc.teamcode.biobuzz.TeamOnePollenAuto;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * End-to-end opening shot: the real Team One routine (with the robot's autonomous settings)
 * driving the real autonomous Shooter, the way AutoStateMachine wires them on the robot.
 * The flywheel holds 11 RPM below target, as reported on the robot.
 */
public final class TeamOneOpeningFeedChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }

    public static void main(String[] args) {
        run(true);
        run(false);
        System.out.println("Opening shot feeds with and without a valid odometry pose.");
    }

    private static void run(boolean odometryValid) {
        double[] clock = {0}, target = {0}, feed = {0};
        MotorConfigurationType[] type = {new MotorConfigurationType()};
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, values) -> {
                    switch (method.getName()) {
                        case "getMotorType": return type[0];
                        case "setMotorType": type[0] = (MotorConfigurationType) values[0]; return null;
                        case "setVelocity": target[0] = (double) values[0] / 6.0; return null;
                        case "getVelocity": return target[0] == 0 ? 0.0 : (target[0] - 11) * 6.0;
                        default: return null;
                    }
                });
        CRServo feeder = (CRServo) Proxy.newProxyInstance(CRServo.class.getClassLoader(),
                new Class<?>[]{CRServo.class}, (proxy, method, values) -> {
                    if (method.getName().equals("setPower")) feed[0] = (double) values[0];
                    return null;
                });
        Shooter shooter = new Shooter(motor, feeder, () -> clock[0]);
        FieldLayout field = new FieldLayout();
        RobotIO.Pose start = field.hive[0];
        RobotIO io = new RobotIO() {
            public boolean stopRequested() { return false; }
            public double nowSeconds() { return clock[0]; }
            public Sensors readSensors() {
                return new Sensors(start, clock[0], odometryValid, false, -1, Vision.empty(0, clock[0]));
            }
            public void apply(Outputs o) {
                shooter.apply(o.shooterPower, o.feederPower, clock[0] < Constants.AUTO_LAST_FEED_START_SEC, o.forceFeed);
            }
            public int detectedShots() { return shooter.getDetectedShots(); }
            public int emptyFeedPulses() { return shooter.getEmptyPulseStreak(); }
        };
        TeamOnePollenAuto one = new TeamOnePollenAuto(io, field);
        one.spinUpSeconds = Constants.AUTO_SHOOTER_SPINUP_SEC;
        one.feedSeconds = Constants.AUTO_TEAM_ONE_FEED_TIME_SEC;
        one.observationHeadingOffsetRadians = Math.toRadians(Constants.TEAM_ONE_TAG_VIEW_TURN_DEG);
        one.surveyBeforePickup = false;
        one.gardenAdvanceInches = Constants.AUTO_GARDEN_ADVANCE_IN;
        one.gardenAdvanceSeconds = Constants.AUTO_GARDEN_ADVANCE_SEC;
        one.gardenAdvancePower = .60;
        one.gardenFirstPassSeconds = Constants.AUTO_GARDEN_FIRST_PASS_SEC;
        one.gardenStagingOffsetInches = 12.0;
        one.gardenTransitPower = .70;
        one.gardenRetryTransitPower = Constants.AUTO_GARDEN_RETRY_TRANSIT_POWER;
        one.parkStopEarlierInches = Constants.AUTO_PARK_STOP_EARLIER_IN;
        one.forceFeedLastSeconds = Constants.AUTO_FORCE_FEED_LAST_SEC;
        one.gardenRepositionPower = .60;
        one.gardenIntakeHeadingRadians = Math.toRadians(Constants.TEAM_ONE_GARDEN_INTAKE_HEADING_DEG);
        one.allowAnyFreshAprilTag = Constants.PRACTICE_ALLOW_ANY_FRESH_APRILTAG;
        one.colorPickupEnabled = true;
        one.gardenYellowWindowEnabled = false;
        one.translationPowerMultiplier = 1;
        one.start(0, Constants.SHOOTER_RPM_MULTIPLIER);

        int pulses = 0;
        boolean wasOn = false;
        double firstPulse = Double.NaN;
        String state = "";
        for (int i = 0; i < 400 && one.getState() == TeamOnePollenAuto.State.SHOOTING; i++) {
            clock[0] = i * .02;
            one.tick();
            boolean on = feed[0] > 0;
            if (on && !wasOn) { pulses++; if (Double.isNaN(firstPulse)) firstPulse = clock[0]; }
            wasOn = on;
            state = one.status();
        }
        System.out.printf("Opening shot (odometry " + (odometryValid ? "valid" : "INVALID") + "): target %.0f RPM, first pulse at %.2f s, %d pulses, left SHOOTING at %.2f s (%s)%n",
                target[0], firstPulse, pulses, clock[0], state);
        check(!Double.isNaN(firstPulse) && firstPulse < 1.5, "The opening shot feeds right after spin-up");
        check(pulses >= 8, "The opening window keeps feeding (got " + pulses + " pulses)");
    }
}
