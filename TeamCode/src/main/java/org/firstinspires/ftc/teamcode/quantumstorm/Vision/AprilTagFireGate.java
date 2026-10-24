package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

/** Pure firing-permission logic; angles are camera-space degrees, not aiming offsets. */
public final class AprilTagFireGate {
    public static final double ORIENTATION_TOLERANCE_DEGREES = 5.0;
    public static final double STRICT_STARTUP_SECONDS = 10.0;
    private static final int REQUIRED_STABLE_FRAMES = 3;

    private double[] reference;
    private double orientationError = Double.NaN;
    private double lastTimestamp = Double.NaN;
    private int lastTagId = -1;
    private int stableFrames;
    private boolean lastBlue;

    public static boolean matchesAlliance(int tagId, boolean isBlue) {
        return isBlue ? tagId >= 42 && tagId <= 45 : tagId >= 34 && tagId <= 37;
    }

    public boolean calibrate(double yaw, double pitch, double roll) {
        clearCalibration();
        if (!finiteAngles(yaw, pitch, roll)) {
            return false;
        }
        reference = rotation(yaw, pitch, roll);
        return true;
    }

    public void clearCalibration() {
        reference = null;
        orientationError = Double.NaN;
        stableFrames = 0;
        lastTagId = -1;
        lastTimestamp = Double.NaN;
    }

    public boolean isCalibrated() {
        return reference != null;
    }

    public double getOrientationError() {
        return orientationError;
    }

    /** After startup, any fresh visible tag suffices; tag loss still revokes permission. */
    public boolean updateDuringRun(int tagId, boolean isBlue, double yaw, double pitch,
                                   double roll, double timestamp, boolean fresh,
                                   boolean hasVisibleTag, double elapsedSeconds) {
        if (!Double.isFinite(elapsedSeconds) || elapsedSeconds < 0) {
            stableFrames = 0;
            lastTagId = -1;
            orientationError = Double.NaN;
            return false;
        }
        if (elapsedSeconds >= STRICT_STARTUP_SECONDS) {
            stableFrames = 0;
            lastTagId = -1;
            orientationError = Double.NaN;
            // Freshness comes from the Control Hub result age, not the camera timestamp.
            return fresh && hasVisibleTag;
        }
        return update(tagId, isBlue, yaw, pitch, roll, timestamp, fresh && hasVisibleTag);
    }

    /** Invalid, stale, angled, or changed targets immediately revoke permission. */
    public boolean update(int tagId, boolean isBlue, double yaw, double pitch, double roll,
                          double timestamp, boolean fresh) {
        orientationError = Double.NaN;
        boolean valid = fresh && reference != null && matchesAlliance(tagId, isBlue)
                && finiteAngles(yaw, pitch, roll) && Double.isFinite(timestamp);
        if (valid) {
            double[] current = rotation(yaw, pitch, roll);
            // trace(reference^T * current) measures full 3D rotation, including roll.
            double trace = 0;
            for (int i = 0; i < reference.length; i++) {
                trace += reference[i] * current[i];
            }
            double cosine = Math.max(-1.0, Math.min(1.0, (trace - 1.0) / 2.0));
            orientationError = Math.toDegrees(Math.acos(cosine));
            valid = orientationError <= ORIENTATION_TOLERANCE_DEGREES + 1e-9;
        }
        if (!valid) {
            stableFrames = 0;
            lastTagId = -1;
            return false;
        }
        if (lastTagId != tagId || lastBlue != isBlue) {
            stableFrames = 0;
        }
        if (Double.compare(timestamp, lastTimestamp) != 0) {
            stableFrames = Math.min(stableFrames + 1, REQUIRED_STABLE_FRAMES);
            lastTimestamp = timestamp;
        }
        lastTagId = tagId;
        lastBlue = isBlue;
        return stableFrames >= REQUIRED_STABLE_FRAMES;
    }

    private static boolean finiteAngles(double yaw, double pitch, double roll) {
        return Double.isFinite(yaw) && Double.isFinite(pitch) && Double.isFinite(roll);
    }

    private static double[] rotation(double yaw, double pitch, double roll) {
        double cy = Math.cos(Math.toRadians(yaw));
        double sy = Math.sin(Math.toRadians(yaw));
        double cp = Math.cos(Math.toRadians(pitch));
        double sp = Math.sin(Math.toRadians(pitch));
        double cr = Math.cos(Math.toRadians(roll));
        double sr = Math.sin(Math.toRadians(roll));
        return new double[] {
                cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                -sp, cp * sr, cp * cr
        };
    }
}
