package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

/** Pitch-only heuristic for matched camera/tag tilt; unknown does not mean lowered. */
public final class HiveRaisedDetector {
    // Both tilts use the same physical reference; camera-space pose already includes camera tilt.
    public static final double HIVE_TAG_TILT_DEGREES = 30.0;
    public static final double CAMERA_TILT_DEGREES = 30.0;
    public static final double RAISED_PITCH_DEGREES = HIVE_TAG_TILT_DEGREES - CAMERA_TILT_DEGREES;
    public static final double PITCH_TOLERANCE_DEGREES = 10.0;
    public static final double AIM_TOLERANCE_DEGREES = 5.0;
    public static final int MIN_HIVE_TAG_ID = 30;
    public static final int MAX_HIVE_TAG_ID = 45;

    public enum Status {
        UNKNOWN,
        RAISED,
        OUTSIDE_RAISED_RANGE
    }

    private HiveRaisedDetector() {
    }

    public static boolean isHiveTag(int tagId) {
        return tagId >= MIN_HIVE_TAG_ID && tagId <= MAX_HIVE_TAG_ID;
    }

    public static boolean isShootRecommended(int tagId, double cameraPitchDegrees,
                                             double horizontalDegrees, boolean fresh) {
        return classify(tagId, cameraPitchDegrees, fresh) == Status.RAISED
                && Double.isFinite(horizontalDegrees)
                && Math.abs(horizontalDegrees) <= AIM_TOLERANCE_DEGREES;
    }

    public static Status classify(int tagId, double cameraPitchDegrees, boolean fresh) {
        if (!fresh || !isHiveTag(tagId) || !Double.isFinite(cameraPitchDegrees)) {
            return Status.UNKNOWN;
        }
        return Math.abs(cameraPitchDegrees - RAISED_PITCH_DEGREES) <= PITCH_TOLERANCE_DEGREES
                ? Status.RAISED : Status.OUTSIDE_RAISED_RANGE;
    }
}
