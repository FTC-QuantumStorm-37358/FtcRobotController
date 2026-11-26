package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.Collections;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/** Runs the actual FTCLib controller against instrumented motor/voltage interfaces. */
public final class ShooterPidfControlChecks {
    private static void check(boolean value, String why) { if (!value) throw new AssertionError(why); }
    private static void near(double value, double expected, String why) {
        check(Double.isFinite(value) && Math.abs(value - expected) < 1e-9, why + ": " + value);
    }
    private static final class TrackingPidf extends PIDFController {
        int resets, clears, calculations;
        // Isolate instantaneous P/F and voltage behavior here; the steady-state suite exercises I over time.
        TrackingPidf() { super(Constants.SHOOTER_PIDF_P, 0,
                Constants.SHOOTER_PIDF_D, Constants.SHOOTER_PIDF_F); }
        @Override public void reset() { super.reset(); resets++; }
        @Override public void clearTotalError() { super.clearTotalError(); clears++; }
        @Override public double calculate(double measured, double target) {
            calculations++;
            return super.calculate(measured, target);
        }
    }
    private static final class MotorProbe {
        double rpm = 3000, power = Double.NaN;
        int reads, velocityWrites;
        DcMotor.RunMode mode;
        MotorConfigurationType type = new MotorConfigurationType();
        final DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, values) -> {
                    switch (method.getName()) {
                        case "getMotorType": return type;
                        case "setMotorType": type = (MotorConfigurationType) values[0]; return null;
                        case "setMode": mode = (DcMotor.RunMode) values[0]; return null;
                        case "getVelocity":
                            check(values.length == 1 && values[0] == AngleUnit.DEGREES, "Angular encoder units");
                            reads++; return rpm * 6;
                        case "setPower":
                            check(mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER, "Only software closes velocity loop");
                            power = (double) values[0]; return null;
                        case "setVelocity": velocityWrites++; throw new AssertionError("No Hub velocity commands");
                        default: throw new AssertionError("Unexpected motor call: " + method.getName());
                    }
                });
    }
    private static VoltageSensor sensor(double[] volts) {
        return (VoltageSensor) Proxy.newProxyInstance(VoltageSensor.class.getClassLoader(),
                new Class<?>[]{VoltageSensor.class}, (proxy, method, values) -> {
                    if (method.getName().equals("getVoltage")) return volts[0];
                    throw new AssertionError("Unexpected voltage call");
                });
    }
    public static void main(String[] args) {
        MotorProbe probe = new MotorProbe();
        TrackingPidf pidf = new TrackingPidf();
        double[] volts = {12};
        ShooterPidfControl control = new ShooterPidfControl(probe.motor, Arrays.asList(sensor(volts)), pidf);
        near(probe.power, 0, "INIT power zero");
        near(probe.type.getTicksPerRev(), 28, "Correct encoder scale");
        near(Constants.SHOOTER_PIDF_TEST_DEFAULT_MULTIPLIER, .48, "Test default multiplier");
        check(control.update(.5), "Valid update");
        near(control.getCurrentRpm(), 3000, "Degrees/sec read as RPM");
        near(control.getTargetRpm(), 3000, "6000 times multiplier");
        near(probe.power, .5, "FTCLib F multiplies target RPM at zero error");
        probe.rpm = 2700; control.update(.5);
        near(probe.power, .53, "RPM feedback increases output under load");
        probe.rpm = 3300; control.update(.5);
        near(probe.power, .47, "RPM feedback reduces output above target");
        probe.rpm = 3000; volts[0] = 10.5; control.update(.5);
        near(probe.power, .5 * 12 / 10.5, "Depleted battery compensation");
        volts[0] = 14; control.update(.5);
        near(probe.power, .5 * 12 / 14, "Charged battery compensation");
        probe.rpm = 0; control.update(1);
        near(probe.power, 1, "Compensate before upper clipping");
        check(pidf.clears > 0, "Saturation clears integral accumulation");
        probe.rpm = 10000; control.update(.5);
        near(probe.power, 0, "Overspeed never reverses shooter");
        int resets = pidf.resets, calculations = pidf.calculations, reads = probe.reads;
        control.update(0);
        near(probe.power, 0, "STOP power zero");
        check(pidf.resets > resets && pidf.calculations == calculations && probe.reads == reads,
                "STOP resets PIDF without reading sensors or calculating a reverse command");
        probe.rpm = 3000; volts[0] = 12; control.update(.5);
        near(probe.power, .5, "Clean restart after reset");
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1, 1.01}) {
            check(!control.update(invalid), "Invalid target reported");
            near(probe.power, 0, "Invalid target stops");
        }
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            probe.rpm = invalid;
            check(!control.update(.5), "Invalid RPM reported");
            check(control.getStatus() == ShooterPidfControl.Status.INVALID_RPM, "RPM fault telemetry");
            near(probe.power, 0, "Invalid feedback stops");
        }
        probe.rpm = 3000;
        for (double invalid : new double[]{0, -1, 1, 20, Double.NaN, Double.POSITIVE_INFINITY}) {
            volts[0] = invalid;
            check(!control.update(.5), "Invalid battery reported");
            near(probe.power, 0, "Invalid battery stops");
            control.update(0);
            check(control.getStatus() == ShooterPidfControl.Status.INVALID_VOLTAGE, "Fault remains visible while stopped");
        }
        volts[0] = 12; check(control.update(.5), "Recover after voltage fault");
        near(probe.power, .5, "Recovery output");
        double[] bad = {Double.NaN}, other = {13};
        control = new ShooterPidfControl(probe.motor, Arrays.asList(sensor(bad), sensor(other), sensor(volts)), pidf);
        control.update(.5); near(control.getBatteryVolts(), 12, "Use minimum usable Hub voltage");
        control = new ShooterPidfControl(probe.motor, Collections.emptyList(), pidf);
        check(!control.update(.5), "Missing voltage sensor stops"); near(probe.power, 0, "Missing sensor power zero");
        control = new ShooterPidfControl(probe.motor, Arrays.asList(sensor(volts)), pidf);
        pidf.setF(Double.NaN);
        check(!control.update(.5), "Invalid computed output stops"); near(probe.power, 0, "NaN output not sent to motor");
        check(probe.velocityWrites == 0, "No setVelocity during INIT/control/STOP");
        System.out.println("FTCLib PIDF, RPM scale, load response, 12/V compensation, saturation, STOP/reset and sensor faults passed.");
    }
}
