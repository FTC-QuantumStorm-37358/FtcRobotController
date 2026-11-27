package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import org.firstinspires.ftc.teamcode.biobuzz.FeedCadence;

/** Desktop checks for the autonomous at-speed gate and FeedCadence's start-allowed input. */
public final class ShooterReadyGateChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }

    public static void main(String[] args) {
        ShooterReadyGate g = new ShooterReadyGate(.02, 50, .10);
        check(Math.abs(g.toleranceRpm(2880) - 57.6) < 1e-9, "2% band at 2880 RPM");
        check(g.toleranceRpm(1000) == 50, "Band never narrower than the 50 RPM floor");
        check(!g.update(0, 0, 0) && g.describe().equals("OFF"), "Zero target is never ready");
        check(!g.update(2880, 2500, 0) && g.describe().startsWith("WAITING"), "Below band blocks and explains why");
        check(!g.update(2880, 2850, .10) && g.describe().equals("SETTLING"), "Entering band starts the settle time");
        check(!g.update(2880, 2900, .19), "Not ready before the full settle time");
        check(g.update(2880, 2870, .20) && g.describe().startsWith("READY"), "Ready after 0.10 s inside the band");
        check(!g.update(2880, 2700, .25), "Post-shot dip clears readiness");
        check(!g.update(2880, 2880, .30) && !g.update(2880, 2880, .39), "Recovery must settle again");
        check(g.update(2880, 2880, .40), "Ready again after recovery settles");
        check(!g.update(2880, 2950, .41), "Overspeed beyond the band also blocks");
        check(!g.update(2880, Double.NaN, .5) && !g.update(Double.NaN, 2880, .5), "Invalid readings fail closed");
        g.update(2880, 2880, 1.0);
        check(g.update(2880, 2880, 1.2), "Ready with a steady in-band reading");
        check(!g.update(2880, 2880, 1.1), "Backwards clock fails closed");

        // The table target moves slightly with odometry noise; that alone must not restart settling.
        g = new ShooterReadyGate(.02, 50, .10);
        g.update(3215.2, 3200, 0);
        check(g.update(3214.8, 3205, .10), "Small target changes keep the settle timer");

        // Asymmetric band: tighter below target, where a slow flywheel lands the shot low.
        g = new ShooterReadyGate(.02, .03, 50, .10);
        check(Math.abs(g.lowToleranceRpm(2880) - 57.6) < 1e-9 && Math.abs(g.highToleranceRpm(2880) - 86.4) < 1e-9,
                "Low side 2%, high side 3% at 2880 RPM");
        g.update(2880, 2810, 0);
        check(!g.update(2880, 2810, .2), "70 RPM low is outside the tighter low side");
        g.update(2880, 2950, .3);
        check(g.update(2880, 2950, .5), "70 RPM high is still inside the high side");
        check(new ShooterReadyGate().lowToleranceRpm(2754) < new ShooterReadyGate().highToleranceRpm(2754),
                "Default band is tighter below target");

        FeedCadence f = new FeedCadence(.18, .35);
        check(f.apply(1, 0, false) == 0, "No new pulse while the flywheel is not at speed");
        check(f.apply(1, .05, true) == 1, "Pulse starts as soon as the gate allows");
        check(f.apply(1, .10, false) == 1, "An active pulse finishes even if RPM dips");
        check(f.apply(1, .23, true) == 0, "Pulse ends after 0.18 s and the OFF gap starts");
        check(f.apply(1, .60, false) == 0, "After the gap, still waits for speed");
        check(f.apply(1, .70, true) == 1, "Next pulse once back at speed");
        check(f.apply(-1, .75, false) == -1, "Reverse unjamming is never gated");
        check(f.apply(1, 2, true) == 1, "Forward resumes after the reverse gap");
        FeedCadence legacy = new FeedCadence(.18, .35);
        check(legacy.apply(1, 0) == 1, "Two-argument apply keeps its previous behavior");

        System.out.println("Shooter at-speed band, settle, dip recovery, overspeed, invalid input and gated feed-start checks passed.");
    }
}
