package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Ends a paced feed batch as soon as the flywheel shows it is done, instead of always
 * running the full timed window. Without launch detection (counters of -1) it never ends
 * a batch, so the timed window still applies.
 *
 * <p>A known full load (the preloads) ends after all AUTO_BALLS_TO_SHOOT launches and one more
 * pulse that launched nothing, so a doubled detection cannot leave a ball inside. It never ends
 * with two or more launches unaccounted for; one uncounted launch is accepted only after several
 * pulses in a row launch nothing. A load of unknown size (after pickup) also ends once a ball
 * has launched and the following pulses launched nothing.</p>
 */
final class FeedBatchEnd {
    private int shotsAtStart = -1;
    private boolean fullLoad;

    /** Call when a feed batch of unknown size is entered. */
    void begin(RobotIO io) { begin(io, false); }

    /** Call when a feed batch is entered; fullLoad means it holds AUTO_BALLS_TO_SHOOT balls. */
    void begin(RobotIO io, boolean fullLoad) {
        shotsAtStart = io.detectedShots();
        this.fullLoad = fullLoad;
    }

    /**
     * Preload batch near the end of its window with its load not yet confirmed launched:
     * the caller should feed without waiting for flywheel speed. Without detection, always true
     * in that final stretch, so the fourth preload is never held back.
     */
    boolean forceRemaining(RobotIO io, double elapsed, double window, double lastSeconds) {
        return fullLoad && lastSeconds > 0 && elapsed >= window - lastSeconds
                && shots(io) < Constants.AUTO_BALLS_TO_SHOOT;
    }

    /** Launches in this batch, or -1 without detection. */
    int shots(RobotIO io) {
        int shots = io.detectedShots();
        return shotsAtStart < 0 || shots < 0 ? -1 : shots - shotsAtStart;
    }

    boolean complete(RobotIO io) {
        int shots = shots(io), empty = io.emptyFeedPulses();
        if (shots < 0 || empty < 0) return false;
        if (fullLoad) {
            // Normal end: every preload launched, then one confirming pulse launched nothing.
            // A ball can leave uncounted (e.g. carried out during spin-up, outside any pulse), so
            // one missing launch is tolerated after several pulses in a row launch nothing.
            return shots >= Constants.AUTO_BALLS_TO_SHOOT && empty >= 1
                    || shots >= Constants.AUTO_BALLS_TO_SHOOT - 1 && empty >= Constants.AUTO_FEED_EMPTY_PULSES_TO_FINISH;
        }
        return shots >= Constants.AUTO_BALLS_TO_SHOOT
                || (shots >= 1 && empty >= Constants.AUTO_FEED_EMPTY_PULSES_TO_FINISH);
    }
}
