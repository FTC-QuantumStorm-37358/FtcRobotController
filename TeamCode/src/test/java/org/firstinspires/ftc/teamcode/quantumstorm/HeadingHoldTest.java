package org.firstinspires.ftc.teamcode.quantumstorm;

import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.HeadingHold;

public final class HeadingHoldTest {
    public static void main(String[] args) {
        HeadingHold hold = new HeadingHold();
        near(hold.update(0, true, 0), 0, "Capture initial heading");
        near(hold.update(5, true, 0), 0.1, "Left veer needs clockwise correction");
        near(hold.update(-5, true, 0), -0.1, "Right veer needs counterclockwise correction");
        near(hold.update(90, true, 0), 0.2, "Positive clamp");
        near(hold.update(-90, true, 0), -0.2, "Negative clamp");
        near(hold.update(1, true, 0), 0, "Heading deadband");
        near(hold.update(20, true, 0.6), 0.6, "Manual turn overrides hold");
        near(hold.update(20, true, 0), 0, "Hold new heading after manual turn");
        near(hold.update(40, false, 0), 0, "Never turn while idle");
        check(!hold.isActive(), "Idle inactive");
        near(hold.update(45, true, 0), 0.1, "Resume from idle heading");
        near(hold.update(Double.NaN, true, 0), 0, "Invalid IMU disables correction");
        check(!hold.isActive(), "Invalid IMU inactive");
        near(hold.update(100, true, 0), 0, "Recovery recaptures heading");
        near(hold.update(Double.POSITIVE_INFINITY, true, -0.4), -0.4, "Manual turn without IMU");
        near(hold.update(179, false, 0), 0, "Capture near wrap boundary");
        near(hold.update(-179, true, 0), 0.04, "Yaw wraparound clockwise");
        near(hold.update(-179, false, 0), 0, "Capture other wrap boundary");
        near(hold.update(179, true, 0), -0.04, "Yaw wraparound counterclockwise");
        check(hold.isActive(), "Translating hold active");
        System.out.println("HeadingHoldTest: all checks passed");
    }

    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-9, message + ": " + actual);
    }

    private static void check(boolean condition, String message) {
        if (!condition) {
            throw new AssertionError(message);
        }
    }
}
