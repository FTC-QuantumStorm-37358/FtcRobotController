package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.Collections;
import org.firstinspires.ftc.teamcode.quantumstorm.Auto.TeamOneAutonomous;
import org.firstinspires.ftc.teamcode.quantumstorm.Auto.TeamTwoAutonomous;
import org.firstinspires.ftc.teamcode.quantumstorm.Auto.TeamOneBackupAutonomous;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.AutoShooterTarget;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.ShooterPidfControl;

/** Real TeleOp hooks, gamepad edges, and output controller; no OpMode or physical hardware runs. */
public final class ShooterTeleOpTargetChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }
    private static void near(double actual, double expected, String why) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-9,
                why + ": expected " + expected + ", got " + actual);
    }
    private static Field field(String name) throws Exception {
        Field field = QuantumStormShooterPidfTeleOp.class.getDeclaredField(name);
        field.setAccessible(true);
        return field;
    }
    private static void releaseShoulders(QuantumStormTeleOp mode) {
        mode.gamepad2.left_bumper = false;
        mode.gamepad2.left_trigger = 0;
        mode.updateShooterAdjustment(mode.updateShooterTarget(0));
    }
    public static void main(String[] args) throws Exception {
        QuantumStormTeleOp latest = new QuantumStormShooterPidfTeleOp();
        latest.gamepad2 = new Gamepad();
        AutoShooterTarget target = (AutoShooterTarget) field("shooterTarget").get(latest);
        near(latest.initialShooterSetting(), .495, "Latest starts with 2970 RPM selected");
        check(!latest.shooterTargetReady(), "No initialized pose is not authorization to spin");
        near(latest.updateShooterTarget(.495), 0, "No pose blocks the requested RPM");
        target.updatePose(true, -12.75, -63.25, -90);
        near(latest.updateShooterTarget(0), .495, "Own distance solution supplies target");
        check(latest.shooterTargetReady() && latest.shooterStartAllowed(),
                "A is allowed with the latest entry's own valid solution");
        check(latest.manualShooterAdjustment() && latest.feederRequiresShooterRunning(),
                "Trim enabled, feeder requires shooter ON");

        latest.gamepad2.left_bumper = true;
        near(latest.updateShooterAdjustment(.495), .495, "Shoulder alone is locked");
        latest.gamepad2.back = true;
        near(latest.updateShooterAdjustment(.495), .495, "Back after held shoulder does not trim");
        releaseShoulders(latest);
        latest.gamepad2.left_bumper = true;
        near(latest.updateShooterAdjustment(.495), .500, "Fresh Back+LB adds .005");
        for (int i = 0; i < 10; i++) {
            near(latest.updateShooterAdjustment(latest.updateShooterTarget(0)), .500,
                    "Automatic update and held LB preserve trim without repeat");
        }
        releaseShoulders(latest);
        latest.gamepad2.left_trigger = .5f;
        near(latest.updateShooterAdjustment(.500), .500, "Trigger must pass halfway");
        latest.gamepad2.left_trigger = 1;
        near(latest.updateShooterAdjustment(.500), .495, "Fresh Back+LT subtracts .005");
        releaseShoulders(latest);
        latest.gamepad2.left_trigger = 1;
        near(latest.updateShooterAdjustment(.495), .490, "Second decrement is another .005");
        releaseShoulders(latest);
        latest.gamepad2.left_bumper = true;
        latest.gamepad2.left_trigger = 1;
        near(latest.updateShooterAdjustment(.490), .490, "Simultaneous requests cancel");
        latest.gamepad2.left_trigger = 0;
        near(latest.updateShooterAdjustment(.490), .490, "Releasing opposite button is not a new press");
        releaseShoulders(latest);
        latest.gamepad2.start = true;
        latest.gamepad2.left_bumper = true;
        near(latest.updateShooterAdjustment(.490), .490, "Assignment suppresses trim");
        latest.gamepad2.start = false;
        near(latest.updateShooterAdjustment(.490), .490, "Releasing Start cannot apply held trim");
        releaseShoulders(latest);

        target.updatePose(true, -36, -62.9, -90);
        double movedTarget = latest.updateShooterTarget(.490);
        near(movedTarget, target.getBaseMultiplier() - .005, "Moving preserves relative trim");
        check(Math.abs(movedTarget - .490) > .0001, "Distance continues adjusting target");
        // Confirm no coarse .01-grid rounding of a calculated distance target.
        check(Math.abs(movedTarget * 100 - Math.rint(movedTarget * 100)) > .001,
                "Automatic target remains smoothly interpolated");
        target.updatePose(false, -36, -62.9, -90);
        near(latest.updateShooterTarget(movedTarget), 0, "Invalid sample zeros target");
        check(!latest.shooterTargetReady(), "Invalid sample closes main-loop start/feed gate");
        target.updatePose(true, -12.75, -63.25, -90);
        near(latest.updateShooterTarget(0), .490, "Trim survives pose recovery");

        // Wire the real output hook to instrumented hardware; reject all Hub velocity commands.
        final double[] rpm = {2940}, volts = {12}, power = {Double.NaN};
        final MotorConfigurationType[] type = {new MotorConfigurationType()};
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, values) -> {
                    switch (method.getName()) {
                        case "getMotorType": return type[0];
                        case "setMotorType": type[0] = (MotorConfigurationType) values[0]; return null;
                        case "setMode": return null;
                        case "getVelocity": return rpm[0] * 6;
                        case "setPower": power[0] = (double) values[0]; return null;
                        default: throw new AssertionError("Unexpected motor call: " + method.getName());
                    }
                });
        VoltageSensor battery = (VoltageSensor) Proxy.newProxyInstance(VoltageSensor.class.getClassLoader(),
                new Class<?>[]{VoltageSensor.class}, (proxy, method, values) -> volts[0]);
        ShooterPidfControl controller = new ShooterPidfControl(motor, Collections.singletonList(battery));
        field("shooterPidf").set(latest, controller);
        check(latest.applyShooterOutput(motor, latest.updateShooterTarget(0)), "Valid output hook");
        near(controller.getTargetRpm(), 2940, "Distance plus trim reaches PIDF as RPM");
        near(power[0], .490, "Zero-error power uses trimmed target feedforward");
        volts[0] = 10.5;
        latest.applyShooterOutput(motor, latest.updateShooterTarget(0));
        near(power[0], .490 * 12 / 10.5, "Voltage scales PIDF result, including trim");
        latest.applyShooterOutput(motor, 0);
        near(power[0], 0, "B/STOP output zeros power");
        near(latest.updateShooterTarget(0), .490, "Stopping leaves target trim intact");
        volts[0] = Double.NaN;
        check(!latest.applyShooterOutput(motor, .490), "Fault returns failure to main-loop stop latch");
        near(power[0], 0, "Voltage fault stops output");

        QuantumStormTeleOp old = new QuantumStormTeleOp();
        old.gamepad2 = new Gamepad();
        check(!old.manualShooterAdjustment() && !old.shooterTargetReady(),
                "Old mode retains separate automatic-table readiness");
        check(QuantumStormTeleOp.class.getAnnotation(TeleOp.class).name().equals(Constants.OLD_TELEOP_NAME),
                "Old menu suffix");
        check(Constants.OLD_TELEOP_NAME.endsWith("-old"), "Requested old suffix");
        check(QuantumStormShooterPidfTeleOp.class.getAnnotation(TeleOp.class).name().equals(Constants.TELEOP_NAME)
                && Constants.TELEOP_NAME.endsWith("-latest 11/25"), "Requested latest menu suffix");
        for (Class<?> auto : new Class<?>[]{TeamOneAutonomous.class, TeamTwoAutonomous.class,
                TeamOneBackupAutonomous.class}) {
            check(auto.getAnnotation(Autonomous.class).preselectTeleOp().equals(Constants.TELEOP_NAME),
                    "Autonomous preselects the enabled latest entry");
        }
        System.out.println("Latest TeleOp target/readiness, relative gamepad trim, PIDF/battery output and menu checks passed.");
    }
}
