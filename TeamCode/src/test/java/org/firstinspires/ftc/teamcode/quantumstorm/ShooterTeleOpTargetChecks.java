package org.firstinspires.ftc.teamcode.quantumstorm;

/** Exercises the real TeleOp target hooks without initializing hardware or starting an OpMode. */
public final class ShooterTeleOpTargetChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }

    private static void near(double actual, double expected, String why) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-9,
                why + ": expected " + expected + ", got " + actual);
    }

    public static void main(String[] args) {
        // Call through the base type, as runDriverControls does. No base distance-shot
        // object exists in the manual PIDF entry, even after its own odometry is initialized.
        QuantumStormTeleOp manual = new QuantumStormShooterPidfTeleOp();
        double setting = manual.initialShooterSetting();
        near(setting, .48, "Manual entry starts at the configured 2880 RPM target");
        near(manual.updateShooterTarget(setting), setting,
                "Main-loop target update must not erase the manual setting");
        check(manual.shooterTargetReady() && manual.shooterStartAllowed(),
                "A's target gate is ready without the base automatic shot solution");
        check(manual.manualShooterAdjustment(), "Back + LB/LT must remain enabled");
        check(manual.feederRequiresShooterRunning(), "Manual feed still requires a running shooter");

        setting = manual.clampShooterSetting(setting + Constants.SHOOTER_RPM_MULTIPLIER_INCREMENT);
        for (int loop = 0; loop < 10; loop++) setting = manual.updateShooterTarget(setting);
        near(setting, .49, "A driver's speed increase survives subsequent loops");
        setting = manual.clampShooterSetting(setting - Constants.SHOOTER_RPM_MULTIPLIER_INCREMENT);
        near(manual.updateShooterTarget(setting), .48, "Driver can decrease speed again");
        near(manual.clampShooterSetting(-.01), 0, "PIDF target cannot request reverse shooting");
        near(manual.clampShooterSetting(1.01), 1, "PIDF target remains bounded");

        for (QuantumStormTeleOp automatic : new QuantumStormTeleOp[]{
                new QuantumStormTeleOp(), new QuantumStormAutoShooterTeleOp()}) {
            check(!automatic.manualShooterAdjustment(), "Automatic entries keep table-selected RPM");
            check(!automatic.shooterTargetReady(), "Automatic entries require a valid distance solution");
            check(automatic.feederRequiresShooterRunning(), "Automatic feed gate remains enabled");
        }
        System.out.println("PIDF manual target, start readiness, speed adjustment and automatic-entry isolation passed.");
    }
}
