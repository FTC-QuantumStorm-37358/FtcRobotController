package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Decides whether the flywheel has recovered enough to start the next feed pulse.
 *
 * <p>Ready means measured RPM has stayed inside the band around the target continuously for
 * the settle time. The band can be tighter below target than above it, since a slow flywheel
 * is what makes a shot land low. Leaving the band, a zero/invalid target, an invalid reading
 * or a backwards clock clears it. SDK-free so desktop checks can drive it.</p>
 */
public final class ShooterReadyGate {
    private final double lowFraction, highFraction, minToleranceRpm, settleSeconds;
    private double inBandSince = Double.NaN, lastNow = Double.NEGATIVE_INFINITY;
    private double targetRpm, measuredRpm = Double.NaN;
    private boolean ready;

    public ShooterReadyGate() {
        this(Constants.SHOOTER_READY_LOW_FRACTION, Constants.SHOOTER_READY_HIGH_FRACTION,
                Constants.SHOOTER_READY_MIN_TOLERANCE_RPM, Constants.SHOOTER_READY_SETTLE_SEC);
    }

    /** Symmetric band of toleranceFraction of target on both sides. */
    public ShooterReadyGate(double toleranceFraction, double minToleranceRpm, double settleSeconds) {
        this(toleranceFraction, toleranceFraction, minToleranceRpm, settleSeconds);
    }

    public ShooterReadyGate(double lowFraction, double highFraction, double minToleranceRpm, double settleSeconds) {
        if (!(lowFraction >= 0) || !(highFraction >= 0) || !(minToleranceRpm >= 0) || !(settleSeconds >= 0)
                || !Double.isFinite(lowFraction + highFraction + minToleranceRpm + settleSeconds)) {
            throw new IllegalArgumentException("Invalid shooter ready gate settings");
        }
        this.lowFraction = lowFraction;
        this.highFraction = highFraction;
        this.minToleranceRpm = minToleranceRpm;
        this.settleSeconds = settleSeconds;
    }

    /** Call every loop with the commanded and measured RPM; returns {@link #isReady()}. */
    public boolean update(double targetRpm, double measuredRpm, double now) {
        this.targetRpm = targetRpm;
        this.measuredRpm = measuredRpm;
        boolean clockValid = Double.isFinite(now) && now >= lastNow;
        if (Double.isFinite(now)) lastNow = Math.max(lastNow, now);
        double error = measuredRpm - targetRpm;
        if (!clockValid || !Double.isFinite(targetRpm) || targetRpm == 0
                || !Double.isFinite(measuredRpm)
                || error < -lowToleranceRpm(targetRpm) || error > highToleranceRpm(targetRpm)) {
            inBandSince = Double.NaN;
            ready = false;
            return false;
        }
        if (Double.isNaN(inBandSince)) inBandSince = now;
        ready = now - inBandSince >= settleSeconds;
        return ready;
    }

    /** How far below target still counts as ready. */
    public double lowToleranceRpm(double targetRpm) {
        return Math.max(minToleranceRpm, Math.abs(targetRpm) * lowFraction);
    }

    /** How far above target still counts as ready. */
    public double highToleranceRpm(double targetRpm) {
        return Math.max(minToleranceRpm, Math.abs(targetRpm) * highFraction);
    }

    /** Symmetric-band helper kept for callers that use one tolerance (the wider side). */
    public double toleranceRpm(double targetRpm) {
        return Math.max(lowToleranceRpm(targetRpm), highToleranceRpm(targetRpm));
    }

    public boolean isReady() { return ready; }

    /** Measured minus target from the last update; NaN before the first reading. */
    public double errorRpm() { return measuredRpm - targetRpm; }

    /** Short telemetry text: OFF, READY, SETTLING or the current RPM error. */
    public String describe() {
        if (targetRpm == 0 || !Double.isFinite(targetRpm)) return "OFF";
        if (!Double.isFinite(measuredRpm)) return "WAITING: no RPM reading";
        if (ready) return String.format(java.util.Locale.US, "READY (%+.0f RPM)", errorRpm());
        if (!Double.isNaN(inBandSince)) return "SETTLING";
        return String.format(java.util.Locale.US, "WAITING: %+.0f RPM (band -%.0f/+%.0f)",
                errorRpm(), lowToleranceRpm(targetRpm), highToleranceRpm(targetRpm));
    }
}
