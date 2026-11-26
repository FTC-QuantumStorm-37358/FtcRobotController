package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.DistanceShooterCalibration.Quadrant;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration.Solution;

/** Latest TeleOp target policy; the shared table and autonomous calibration are unchanged. */
public final class AutoShooterTarget {
    private final TableShooterCalibration table = new TableShooterCalibration();
    private final double startTableRpm;
    private Solution solution;
    // Integer button steps avoid drift. Bound trim to +/- one full RPM multiplier.
    private int trimSteps;
    private static final int MAX_TRIM_STEPS = (int) Math.round(1 / Constants.SHOOTER_AUTO_TRIM_INCREMENT);

    public AutoShooterTarget() {
        Quadrant start = Quadrant.SOUTHWEST;
        startTableRpm = table.atPose(start.startX, start.startY, start.startHeadingDegrees).rpm;
    }

    public void reset() {
        solution = null;
        trimSteps = 0;
    }

    /** Consume the same fresh Pinpoint sample used for aiming; invalid samples erase readiness. */
    public void updatePose(boolean valid, double x, double y, double headingDegrees) {
        solution = valid ? table.atPose(x, y, headingDegrees) : null;
    }

    public boolean ready() { return solution != null && solution.ready(); }
    public Solution getSolution() { return solution; }

    /** Shift the table by a constant RPM offset to anchor the start at 0.495 (2970 RPM). */
    public double getBaseMultiplier() {
        return ready() ? Constants.SHOOTER_AUTO_START_MULTIPLIER
                + (solution.rpm - startTableRpm) / Constants.SHOOTER_BASE_RPM : 0;
    }

    public void adjustTrim(int direction) {
        trimSteps = Math.max(-MAX_TRIM_STEPS,
                Math.min(MAX_TRIM_STEPS, trimSteps + Integer.signum(direction)));
    }

    public double getTrimMultiplier() { return trimSteps * Constants.SHOOTER_AUTO_TRIM_INCREMENT; }

    /** Trim changes the requested RPM, never the final motor-power output. */
    public double getMultiplier() {
        return ready() ? Math.max(0, Math.min(1, getBaseMultiplier() + getTrimMultiplier())) : 0;
    }
}
