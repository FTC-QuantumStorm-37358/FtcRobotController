package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

/** Dependency-free regression tests; run main with the production gate on the classpath. */
public final class AprilTagFireGateTest {
    public static void main(String[] args) {
        for (int id = 0; id <= 50; id++) {
            check(AprilTagFireGate.matchesAlliance(id, false) == (id >= 34 && id <= 37),
                    "Red ID " + id);
            check(AprilTagFireGate.matchesAlliance(id, true) == (id >= 42 && id <= 45),
                    "Blue ID " + id);
        }
        AprilTagFireGate gate = new AprilTagFireGate();
        check(!gate.update(34, false, 0, 0, 0, 1, true), "Calibration required");
        check(!gate.calibrate(Double.NaN, 0, 0), "Invalid calibration rejected");
        check(gate.calibrate(0, 0, 0), "Valid calibration accepted");
        check(!gate.update(34, false, 0, 0, 0, 1, true), "First frame held");
        check(!gate.update(34, false, 0, 0, 0, 1, true), "Repeated frame held");
        check(!gate.update(34, false, 0, 0, 0, 2, true), "Second frame held");
        check(gate.update(34, false, 0, 0, 0, 3, true), "Third frame allowed");
        check(!gate.update(34, false, 0, 0, 0, 3, false), "Stale revokes immediately");
        check(!gate.update(34, false, 0, 0, 0, 3, true), "Old frame not recounted");
        check(!gate.update(-1, false, 0, 0, 0, 4, true), "No/ambiguous target held");
        check(!gate.update(42, false, 0, 0, 0, 5, true), "Opposing alliance held");
        check(!gate.update(34, false, Double.NaN, 0, 0, 6, true), "Missing pose held");
        check(!gate.update(34, false, 0, 0, 0, Double.NaN, true), "Missing timestamp held");

        for (int axis = 0; axis < 3; axis++) {
            for (double angle : new double[] {-5, 5, -5.1, 5.1}) {
                gate.calibrate(0, 0, 0);
                double yaw = axis == 0 ? angle : 0;
                double pitch = axis == 1 ? angle : 0;
                double roll = axis == 2 ? angle : 0;
                boolean allowed = false;
                for (int frame = 1; frame <= 3; frame++) {
                    allowed = gate.update(34, false, yaw, pitch, roll, frame, true);
                }
                check(allowed == (Math.abs(angle) <= 5), "Axis " + axis + " angle " + angle);
            }
        }
        gate.calibrate(0, 0, 0);
        check(!gate.update(34, false, 4, 4, 0, 1, true), "Combined rotation exceeds 5 degrees");
        gate.calibrate(179, 30, 0);
        check(!gate.update(34, false, -179, 30, 0, 1, true), "Wraparound first frame");
        check(!gate.update(34, false, -179, 30, 0, 2, true), "Wraparound second frame");
        check(gate.update(34, false, -179, 30, 0, 3, true), "Wraparound and mount calibration");
        check(!gate.update(35, false, -179, 30, 0, 4, true), "Different tag resets stability");
        check(!gate.update(35, false, -179, 30, 0, 5, true), "Different tag second frame held");
        check(gate.update(35, false, -179, 30, 0, 6, true), "Different tag stable");
        check(!gate.update(42, true, -179, 30, 0, 7, true), "Alliance change resets stability");
        check(!gate.update(42, true, -179, 30, 0, 8, true), "Blue second frame held");
        check(gate.update(42, true, -179, 30, 0, 9, true), "Blue stable allowed");
        check(!gate.update(42, true, -179, 40, 0, 10, true), "Angled tag revokes immediately");
        gate.clearCalibration();
        check(!gate.isCalibrated(), "Calibration cleared");
        check(!gate.update(42, true, -179, 30, 0, 11, true), "Cleared calibration holds");
        testStartupFallback();
        System.out.println("AprilTagFireGateTest: all checks passed");
    }

    private static void testStartupFallback() {
        AprilTagFireGate gate = new AprilTagFireGate();
        check(!gate.updateDuringRun(34, false, 0, 0, 0, 1, true, true, 0),
                "Startup still requires calibration");
        gate.calibrate(0, 0, 0);
        check(!gate.updateDuringRun(34, false, 0, 0, 0, 1, true, true, 0),
                "Startup first frame held");
        check(!gate.updateDuringRun(34, false, 0, 0, 0, 2, true, true, 1),
                "Startup second frame held");
        check(gate.updateDuringRun(34, false, 0, 0, 0, 3, true, true, 2),
                "Startup stable correct tag allowed without waiting 10 seconds");
        check(!gate.updateDuringRun(34, false, 0, 10, 0, 4, true, true, 9),
                "Startup angle check retained");
        check(!gate.updateDuringRun(42, false, 0, 0, 0, 5, true, true, 9.999),
                "Opposing tag held just before timeout");
        gate.clearCalibration();
        check(!gate.updateDuringRun(42, false, 0, 90, 0, 6, true, true, 7),
                "Seven seconds does not activate fallback");
        check(gate.updateDuringRun(42, false, Double.NaN, Double.NaN, Double.NaN,
                7, true, true, 10), "Exactly 10 seconds ignores alliance, pose and calibration");
        check(gate.updateDuringRun(99, true, 0, 90, 0, 8, true, true, 11),
                "Unknown ID and angled tag allowed after timeout");
        check(gate.updateDuringRun(-1, false, Double.NaN, Double.NaN, Double.NaN,
                9, true, true, 12), "Multiple tags need no selected target in fallback");
        check(!gate.updateDuringRun(-1, false, 0, 0, 0, 10, true, false, 13),
                "Tag disappearance immediately stops permission");
        check(!gate.updateDuringRun(34, false, 0, 0, 0, 11, false, true, 14),
                "Stale or disconnected detection blocked after timeout");
        check(gate.updateDuringRun(34, false, 0, 0, 0, Double.NaN, true, true, 15),
                "Fresh visible tag does not require camera timestamp after timeout");
        check(gate.updateDuringRun(-1, true, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN, true, true, 15),
                "Multiple fresh tags permit fallback without pose, calibration or timestamp");
        check(!gate.updateDuringRun(-1, true, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN, false, true, 15),
                "Missing timestamp never bypasses stale-result rejection");
        check(gate.updateDuringRun(45, false, 0, 90, 0, 12, true, true, 16),
                "Tag reappearance immediately restores fallback permission");
        check(!gate.updateDuringRun(34, false, 0, 0, 0, 13, true, false, 100),
                "Time alone never permits firing");
        check(!gate.updateDuringRun(34, false, 0, 0, 0, 14, true, true, Double.NaN),
                "Invalid elapsed time blocked");
        check(!gate.updateDuringRun(34, false, 0, 0, 0, 15, true, true, -1),
                "Negative elapsed time blocked");
    }

    private static void check(boolean condition, String message) {
        if (!condition) {
            throw new AssertionError(message);
        }
    }
}
