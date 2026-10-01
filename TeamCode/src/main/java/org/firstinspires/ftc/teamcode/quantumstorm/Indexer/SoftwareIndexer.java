package org.firstinspires.ftc.teamcode.quantumstorm.Indexer;

/**
 * Stand-in until the indexer has ball sensors: the count is only what the
 * autonomous tells it (preload = 4, 0 after shooting, full after a pickup).
 *
 * TODO: replace with a sensor-based Indexer (e.g. color/distance sensor per slot).
 */
public class SoftwareIndexer implements Indexer {

    private int ballCount = 0;

    @Override
    public int getBallCount() {
        return ballCount;
    }

    @Override
    public void setBallCount(int count) {
        ballCount = count;
    }
}
