package org.firstinspires.ftc.teamcode.quantumstorm.DriveChain;

/** IMU yaw is counterclockwise-positive; mecanum turn power is clockwise-positive. */
public final class HeadingHold {
    private static final double KP = 0.02;
    private static final double MAX_CORRECTION = 0.20;
    private static final double ERROR_DEADBAND_DEGREES = 1.0;
    private double targetDegrees = Double.NaN;
    private double errorDegrees = Double.NaN;
    private boolean active;

    public double update(double yawDegrees, boolean translating, double manualTurn) {
        active = false;
        errorDegrees = Double.NaN;
        if (!Double.isFinite(yawDegrees)) {
            targetDegrees = Double.NaN;
            return manualTurn;
        }
        if (!translating || manualTurn != 0.0 || !Double.isFinite(targetDegrees)) {
            targetDegrees = yawDegrees;
            return manualTurn;
        }
        active = true;
        errorDegrees = ((targetDegrees - yawDegrees) % 360.0 + 540.0) % 360.0 - 180.0;
        if (Math.abs(errorDegrees) <= ERROR_DEADBAND_DEGREES) {
            return 0.0;
        }
        return Math.max(-MAX_CORRECTION, Math.min(MAX_CORRECTION, -KP * errorDegrees));
    }

    public boolean isActive() {
        return active;
    }

    public double getErrorDegrees() {
        return errorDegrees;
    }
}
