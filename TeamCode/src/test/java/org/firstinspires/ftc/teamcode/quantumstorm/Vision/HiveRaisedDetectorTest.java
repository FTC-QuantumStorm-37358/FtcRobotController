package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

/** Dependency-free regression tests, following AprilTagFireGateTest. */
public final class HiveRaisedDetectorTest {
    public static void main(String[] args) {
        for (int id = 0; id <= 50; id++) {
            boolean hive = id >= 30 && id <= 45;
            check(HiveRaisedDetector.isHiveTag(id) == hive, "ID range: " + id);
            expect(id, 0, true, hive ? HiveRaisedDetector.Status.RAISED
                    : HiveRaisedDetector.Status.UNKNOWN);
        }
        check(HiveRaisedDetector.HIVE_TAG_TILT_DEGREES == 30, "Physical tag tilt");
        check(HiveRaisedDetector.CAMERA_TILT_DEGREES == 30, "Physical camera tilt");
        check(HiveRaisedDetector.RAISED_PITCH_DEGREES == 0, "Matched tilt gives zero relative pitch");
        for (double pitch : new double[] {-10, -9.999, 0, 9.999, 10}) {
            expect(30, pitch, true, HiveRaisedDetector.Status.RAISED);
        }
        for (double pitch : new double[] {-30, -10.001, 10.001, 30, 90, 180}) {
            expect(45, pitch, true, HiveRaisedDetector.Status.OUTSIDE_RAISED_RANGE);
        }
        for (double pitch : new double[] {Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY}) {
            expect(30, pitch, true, HiveRaisedDetector.Status.UNKNOWN);
        }
        expect(-1, 0, true, HiveRaisedDetector.Status.UNKNOWN);
        expect(30, 30, false, HiveRaisedDetector.Status.UNKNOWN);
        expect(30, 0, false, HiveRaisedDetector.Status.UNKNOWN);
        expect(30, 0, true, HiveRaisedDetector.Status.RAISED);
        expect(-1, Double.NaN, false, HiveRaisedDetector.Status.UNKNOWN);
        expect(30, 30, true, HiveRaisedDetector.Status.OUTSIDE_RAISED_RANGE);
        for (int id : new int[] {29, 30, 45, 46}) {
            for (double pitch : new double[] {-10.001, -10, 0, 10, 10.001, Double.NaN}) {
                for (double tx : new double[] {-5.001, -5, 0, 5, 5.001,
                        Double.NaN, Double.POSITIVE_INFINITY}) {
                    for (boolean fresh : new boolean[] {false, true}) {
                        boolean expected = fresh && id >= 30 && id <= 45
                                && Double.isFinite(pitch) && Math.abs(pitch) <= 10
                                && Double.isFinite(tx) && Math.abs(tx) <= 5;
                        check(HiveRaisedDetector.isShootRecommended(id, pitch, tx, fresh) == expected,
                                "Recommendation id=" + id + " pitch=" + pitch + " tx=" + tx + " fresh=" + fresh);
                    }
                }
            }
        }
        System.out.println("HiveRaisedDetectorTest: all checks passed");
    }

    private static void expect(int id, double pitch, boolean fresh,
                               HiveRaisedDetector.Status expected) {
        check(HiveRaisedDetector.classify(id, pitch, fresh) == expected,
                "id=" + id + ", pitch=" + pitch + ", fresh=" + fresh + ": " + expected);
    }

    private static void check(boolean condition, String message) {
        if (!condition) {
            throw new AssertionError(message);
        }
    }
}
