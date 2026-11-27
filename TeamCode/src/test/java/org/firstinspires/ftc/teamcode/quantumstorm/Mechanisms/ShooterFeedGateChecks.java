package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import java.lang.reflect.Proxy;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/** Autonomous Shooter.apply: feed pulses start only after the measured RPM has recovered. */
public final class ShooterFeedGateChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }

    private static final double[] rpm = {0}, feed = {Double.NaN}, clock = {0};
    private static final int[] velocityReads = {0};

    private static Shooter newShooter() {
        clock[0] = 0; // each scenario starts a fresh match clock
        MotorConfigurationType[] type = {new MotorConfigurationType()};
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, values) -> {
                    switch (method.getName()) {
                        case "getMotorType": return type[0];
                        case "setMotorType": type[0] = (MotorConfigurationType) values[0]; return null;
                        case "getVelocity":
                            check(values.length == 1 && values[0] == AngleUnit.DEGREES, "Read angular velocity");
                            velocityReads[0]++;
                            return rpm[0] * 6.0;
                        default: return null;
                    }
                });
        CRServo feeder = (CRServo) Proxy.newProxyInstance(CRServo.class.getClassLoader(),
                new Class<?>[]{CRServo.class}, (proxy, method, values) -> {
                    if (method.getName().equals("setPower")) feed[0] = (double) values[0];
                    return null;
                });
        return new Shooter(motor, feeder, () -> clock[0]);
    }

    private static void step(Shooter s, double t, double measuredRpm, double feedPower) {
        clock[0] = t;
        rpm[0] = measuredRpm;
        s.apply(.48, feedPower);
    }

    /** Every second pulse launches a ball; after that the RPM stays 120 below target (outside the band, above the floor). */
    private static int ballsLaunched(double windowSeconds) {
        Shooter s = newShooter();
        double measured = 2880;
        int pulses = 0;
        boolean wasOn = false;
        for (int i = 0; i * .02 < windowSeconds; i++) {
            step(s, i * .02, measured, 1);
            boolean pulseOn = feed[0] > 0;
            if (pulseOn && !wasOn && ++pulses % 2 == 0) measured = 2880 - 120;
            wasOn = pulseOn;
        }
        return pulses / 2;
    }

    /**
     * Realistic batch: every second pulse launches a ball 0.1 s after it starts; the RPM drops
     * 250 and recovers linearly over 0.5 s. Returns the time the fourth launch is confirmed.
     */
    private static double fourthLaunchConfirmedAt(Shooter s, double windowSeconds) {
        double launchAt = Double.NEGATIVE_INFINITY, confirmed = Double.NaN;
        int pulses = 0;
        boolean wasOn = false;
        for (int i = 0; i * .02 < windowSeconds; i++) {
            double t = i * .02;
            double measured = 2880 - (t >= launchAt ? 250 * Math.max(0, 1 - (t - launchAt) / .5) : 0);
            step(s, t, measured, 1);
            boolean pulseOn = feed[0] > 0;
            if (pulseOn && !wasOn && ++pulses % 2 == 0 && pulses <= 8) launchAt = t + .1;
            wasOn = pulseOn;
            if (Double.isNaN(confirmed) && s.getDetectedShots() >= 4) confirmed = t;
        }
        return confirmed;
    }

    /**
     * Preload batch in a 6 s window: every second pulse launches a ball; each launch drops the RPM
     * by dip and it recovers linearly over recoverSec. The last forceLast seconds feed with forceFeed
     * while fewer than four launches are confirmed. Returns {balls launched, pulses fed unsettled,
     * time of the fourth launch}.
     */
    private static double[] preloadWindow(double dip, double recoverSec, double forceLast) {
        Shooter s = newShooter();
        double window = Constants.AUTO_TEAM_ONE_FEED_TIME_SEC, launchAt = Double.NEGATIVE_INFINITY, fourthAt = Double.NaN;
        int pulses = 0, launched = 0;
        boolean wasOn = false;
        for (int i = 0; i * .02 < window; i++) {
            double t = i * .02;
            clock[0] = t;
            rpm[0] = 2880 - (t >= launchAt ? dip * Math.max(0, 1 - (t - launchAt) / recoverSec) : 0);
            s.apply(.48, 1, true, t >= window - forceLast && s.getDetectedShots() < 4);
            boolean pulseOn = feed[0] > 0;
            if (pulseOn && !wasOn && ++pulses % 2 == 0 && launched < 4) {
                launchAt = t + .1;
                if (++launched == 4) fourthAt = t;
            }
            wasOn = pulseOn;
        }
        return new double[]{launched, s.getForcedPulses(), fourthAt};
    }

    public static void main(String[] args) {
        double settle = Constants.SHOOTER_READY_SETTLE_SEC;
        double pulse = Constants.SPINDEXER_FEED_PULSE_SEC, gap = Constants.SPINDEXER_SHOT_GAP_SEC;
        double on = Constants.SHOOTER_SERVO_POWER;

        Shooter s = newShooter();
        step(s, 0, 1500, 0);
        check(feed[0] == 0, "Spin-up without a feed request never feeds");
        step(s, 1.0, 2700, 1);
        check(feed[0] == 0 && !s.isFeederPulseActive() && s.describeReadiness().startsWith("WAITING"),
                "Feed request below the RPM band is held");
        step(s, 1.05, 2875, 1);
        check(feed[0] == 0, "Just inside the band: still settling");
        double start = 1.05 + settle + 1e-6;
        step(s, start, 2875, 1);
        check(feed[0] == on && s.isFeederPulseActive(), "Pulse starts once the RPM has settled");

        step(s, start + pulse / 2, 2500, 1);
        check(feed[0] == on, "The ball's RPM dip does not cut the active pulse");
        double pulseEnd = start + pulse + .01;
        step(s, pulseEnd, 2500, 1);
        check(feed[0] == 0, "Pulse ends; mandatory OFF gap");
        step(s, pulseEnd + gap + .01, 2650, 1);
        check(feed[0] == 0, "Gap over but flywheel still recovering: held");
        double recovered = pulseEnd + gap + .05;
        step(s, recovered, 2870, 1);
        check(feed[0] == 0, "Recovered: settling before the next ball");
        step(s, recovered + settle + 1e-6, 2870, 1);
        check(feed[0] == on, "Second pulse only after recovery and settle");

        // A flywheel that settles during the timed spin-up feeds without extra delay.
        s = newShooter();
        step(s, 0, 2880, 0);
        step(s, .5, 2880, 0);
        step(s, .51, 2880, 1);
        check(feed[0] == on, "Spin-up settle counts toward readiness");

        // A flywheel that never settles (but is above the RPM floor) is held up to the max wait, then fed.
        double maxWait = Constants.SHOOTER_READY_MAX_WAIT_SEC;
        s = newShooter();
        double firstPulse = Double.NaN;
        for (int i = 0; i <= 300 && Double.isNaN(firstPulse); i++) {
            step(s, i * .01, 2880 * .96, 1);
            if (feed[0] > 0) firstPulse = i * .01;
        }
        check(firstPulse >= maxWait - 1e-9 && firstPulse < maxWait + .02,
                "Unsettled flywheel is held, then fed at the max wait");
        check(s.describeReadiness().contains("timeout: 1"), "Telemetry counts pulses fed on timeout");

        // Below the RPM floor (e.g. encoder unplugged, or spinning down) a forced pulse never starts.
        s = newShooter();
        for (int i = 0; i * .02 < 3; i++) step(s, i * .02, 2880 * .85, 1);
        check(feed[0] == 0 && s.getDetectedShots() == 0, "No forced pulse below the minimum RPM");

        // Parking cutoff: a running pulse finishes, but no new pulse starts.
        s = newShooter();
        step(s, 0, 2880, 0);
        clock[0] = .2; rpm[0] = 2880; s.apply(.48, 1, true);
        check(feed[0] == on, "Pulse starts before the cutoff");
        clock[0] = .3; s.apply(.48, 1, false);
        check(feed[0] == on, "Cutoff does not cut a running pulse");
        boolean newPulse = false;
        for (int i = 0; i * .02 < 2; i++) {
            clock[0] = .4 + i * .02; s.apply(.48, 1, false);
            newPulse |= feed[0] > 0;
        }
        check(!newPulse, "No new pulse after the cutoff");

        // Worst case (flywheel never re-enters the band): 6 s without forcing launches only three.
        check(ballsLaunched(6.0) == 3, "Model reproduces the three-ball field failure at 6 s");

        // Force-feeding the end of the 6 s preload window. The failure case: after a shot the flywheel
        // stays just outside the band (but above the floor), so every pulse waits the full max wait.
        double force = Constants.AUTO_FORCE_FEED_LAST_SEC;
        double[] stuckNoForce = preloadWindow(120, 1e9, 0), stuckForce = preloadWindow(120, 1e9, force);
        check(stuckNoForce[0] == 3, "Without forcing, a flywheel stuck outside the band launches only three in 6 s");
        check(stuckForce[0] == 4 && stuckForce[2] < Constants.AUTO_TEAM_ONE_FEED_TIME_SEC,
                "Forcing the last " + force + " s launches the fourth preload inside 6 s (got " + (int) stuckForce[0] + ")");
        // When the flywheel recovers normally, forcing never changes when the fourth ball leaves.
        for (double[] m : new double[][]{{250, .5}, {200, 1.0}}) {
            double[] off = preloadWindow(m[0], m[1], 0), forced = preloadWindow(m[0], m[1], force);
            check(off[0] == 4 && forced[0] == 4 && Math.abs(off[2] - forced[2]) < 1e-9,
                    "Normal recovery (dip " + m[0] + ", " + m[1] + " s): forcing must not change the fourth launch");
        }
        System.out.printf("Preload window: stuck flywheel 3 -> 4 balls with forcing (4th at %.2f s).%n", stuckForce[2]);

        // Launch detection: four real dips are counted, later empty pulses are not, and the
        // fourth launch is confirmed well before the old 6 s window would have ended.
        s = newShooter();
        double done = fourthLaunchConfirmedAt(s, 8.0); // run long enough to see the empty pulses afterwards
        check(done < 6.0, "Fourth launch confirmed before 6 s (at " + done + ")");
        check(s.getDetectedShots() == 4 && s.getEmptyPulseStreak() >= Constants.AUTO_FEED_EMPTY_PULSES_TO_FINISH,
                "Exactly four launches; the pulses after the load is gone count as empty");

        // Encoder noise of two counts never looks like a launch.
        s = newShooter();
        for (int i = 0; i * .02 < 4; i++) step(s, i * .02, 2880 + (i % 2 == 0 ? 43 : -43), 1);
        check(s.getDetectedShots() == 0 && s.getEmptyPulseStreak() > 0, "Noise is not counted as a launch");

        int reads = velocityReads[0];
        s.stop();
        clock[0] = 4;
        s.apply(0, 1);
        check(feed[0] == 0 && velocityReads[0] == reads && s.describeReadiness().startsWith("OFF"),
                "Stopped shooter neither feeds nor reads the encoder");
        System.out.printf("Autonomous shooter at-speed hold, settle, mid-pulse dip, recovery, spin-up, max-wait, four-ball window, launch detection and STOP checks passed (modeled 4th launch confirmed at %.2f s).%n", done);
    }
}
