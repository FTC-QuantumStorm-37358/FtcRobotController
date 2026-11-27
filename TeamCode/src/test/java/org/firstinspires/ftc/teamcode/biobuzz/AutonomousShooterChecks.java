package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration;

/** The autonomous-only RPM offset applies to table and fallback targets, never to OFF. */
public final class AutonomousShooterChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }

    private static boolean near(double a, double b) { return Math.abs(a - b) < 1e-6; }

    public static void main(String[] args) {
        double offset = Constants.AUTO_SHOOTER_RPM_OFFSET, base = Constants.SHOOTER_BASE_RPM;
        double table = new TableShooterCalibration().atPose(-12.75, -62.9, -90).rpm;
        AutonomousShooter shooter = new AutonomousShooter();

        double m = shooter.command(new RobotIO.Pose(-12.75, -62.9, Math.toRadians(-90)), .48);
        check(near(shooter.targetRpm(), table + offset) && near(m * base, shooter.targetRpm()),
                "Start-pose table target is lowered by the autonomous offset");
        m = shooter.command(null, .48);
        check(near(shooter.targetRpm(), .48 * base + offset) && near(m * base, shooter.targetRpm()),
                "Fallback multiplier is lowered by the same offset");
        check(shooter.command(null, 0) == 0 && shooter.targetRpm() == 0, "OFF stays OFF");
        check(shooter.command(null, Double.NaN) == 0 && shooter.targetRpm() == 0, "Invalid fallback stays OFF");
        check(shooter.command(null, 1) <= 1, "Offset result stays within the motor range");
        System.out.printf("Autonomous RPM offset checks passed (start pose %.0f -> %.0f RPM).%n",
                table, table + offset);
    }
}
