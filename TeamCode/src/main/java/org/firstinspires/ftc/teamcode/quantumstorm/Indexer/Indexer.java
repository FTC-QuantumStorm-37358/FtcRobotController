package org.firstinspires.ftc.teamcode.quantumstorm.Indexer;

/**
 * The ball indexer: holds the balls between intake and shooter.
 * The autonomous asks it how many balls the robot is holding.
 */
public interface Indexer {

    /** Number of balls currently in the indexer. */
    int getBallCount();

    /**
     * Tell the indexer how many balls the robot should now hold, when that is
     * known without sensors (preload, after a full feed, after a pickup).
     * An indexer with real ball sensors ignores this and keeps reporting what
     * it measures, so a mismatch shows up as a jam/fault in the autonomous.
     */
    default void setBallCount(int count) {
    }
}
