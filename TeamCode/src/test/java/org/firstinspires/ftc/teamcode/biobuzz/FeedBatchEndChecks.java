package org.firstinspires.ftc.teamcode.biobuzz;

/** Feed batches end early only on flywheel-confirmed launches; no detection keeps the timed window. */
public final class FeedBatchEndChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }

    public static void main(String[] args) {
        final int[] shots = {-1}, empty = {-1};
        RobotIO io = new RobotIO() {
            public double nowSeconds() { return 0; }
            public Sensors readSensors() { return null; }
            public void apply(Outputs outputs) { }
            public int detectedShots() { return shots[0]; }
            public int emptyFeedPulses() { return empty[0]; }
        };
        FeedBatchEnd batch = new FeedBatchEnd();
        batch.begin(io);
        check(!batch.complete(io) && batch.shots(io) == -1, "Without launch detection only the timed window ends a batch");

        shots[0] = 5; empty[0] = 0;
        batch.begin(io);
        shots[0] = 8;
        check(!batch.complete(io) && batch.shots(io) == 3, "Three launches keep feeding");
        empty[0] = 2;
        check(!batch.complete(io), "Two empty pulses after a launch keep feeding");
        empty[0] = 3;
        check(batch.complete(io), "Three empty pulses after a launch: the load is gone");

        batch.begin(io);
        empty[0] = 6;
        check(!batch.complete(io), "An empty streak with no launch in this batch never ends it");
        shots[0] = 12; empty[0] = 0;
        check(batch.complete(io), "Four launches end the batch immediately");

        // Known full load (the preloads): needs one confirming empty pulse after the fourth launch,
        // never ends with two launches unaccounted for, and accepts one uncounted launch only
        // after three empty pulses in a row.
        shots[0] = 20; empty[0] = 0;
        batch.begin(io, true);
        shots[0] = 22; empty[0] = 6;
        check(!batch.complete(io), "Preload batch never ends with two launches unaccounted for");
        shots[0] = 23; empty[0] = 2;
        check(!batch.complete(io), "Three launches and two empty pulses keep feeding the preloads");
        empty[0] = 3;
        check(batch.complete(io), "One uncounted launch (e.g. during spin-up) is accepted after three empty pulses");
        batch.begin(io, true);
        shots[0] = 27; empty[0] = 0;
        check(!batch.complete(io), "Preload batch waits for one confirming pulse after the fourth launch");
        empty[0] = 1;
        check(batch.complete(io), "Fourth launch plus an empty pulse ends the preload batch");

        // Forced feed at the end of a preload window, only while a preload is still inside.
        shots[0] = 30; empty[0] = 0;
        batch.begin(io, true);
        shots[0] = 32;
        check(!batch.forceRemaining(io, 4.4, 6, 1.5), "No forcing before the final 1.5 s");
        check(batch.forceRemaining(io, 4.5, 6, 1.5), "Force from 4.5 s while fewer than four have launched");
        shots[0] = 34;
        check(!batch.forceRemaining(io, 5.0, 6, 1.5), "No forcing once all four have launched");
        check(!batch.forceRemaining(io, 5.0, 6, 0), "Zero seconds disables forcing");
        batch.begin(io, false);
        check(!batch.forceRemaining(io, 5.9, 6, 1.5), "Batches of unknown size are never forced");
        shots[0] = -1;
        batch.begin(io, true);
        check(batch.forceRemaining(io, 5.0, 6, 1.5), "Without launch detection the final stretch is always forced");

        RobotIO plain = new RobotIO() {
            public double nowSeconds() { return 0; }
            public Sensors readSensors() { return null; }
            public void apply(Outputs outputs) { }
        };
        check(plain.detectedShots() == -1 && plain.emptyFeedPulses() == -1,
                "Other RobotIO implementations report no detection by default");
        System.out.println("Feed batch early-finish checks passed.");
    }
}
