package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.Collections;

/** Closed-loop checks with real FTCLib and a synthetic motor; not physical tuning validation. */
public final class ShooterPidfSteadyStateChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }

    private static final class Motor {
        double rpm, power, volts = 12;
        DcMotor.RunMode mode;
        MotorConfigurationType type = new MotorConfigurationType();
        final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "getMotorType": return type;
                        case "setMotorType": type = (MotorConfigurationType) args[0]; return null;
                        case "setMode": mode = (DcMotor.RunMode) args[0]; return null;
                        case "getVelocity": return rpm * 6;
                        case "setPower":
                            check(mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER, "External velocity loop only");
                            power = (double) args[0];
                            check(Double.isFinite(power) && power >= 0 && power <= 1, "Bounded motor command");
                            return null;
                        default: throw new AssertionError("Unexpected motor call: " + method.getName());
                    }
                });
        final VoltageSensor voltage = (VoltageSensor) Proxy.newProxyInstance(VoltageSensor.class.getClassLoader(),
                new Class<?>[]{VoltageSensor.class}, (proxy, method, args) -> {
                    if (method.getName().equals("getVoltage")) return volts;
                    throw new AssertionError("Unexpected voltage call");
                });

        void advance(double seconds, double rpmPerPower, double rpmLoss) {
            double equilibrium = Math.max(0, rpmPerPower * power * volts / 12 - rpmLoss);
            rpm += (equilibrium - rpm) * (1 - Math.exp(-seconds / .30));
        }
    }

    private static PIDFController pidf(ShooterPidfControl control) throws Exception {
        Field field = ShooterPidfControl.class.getDeclaredField("controller");
        field.setAccessible(true);
        return (PIDFController) field.get(control);
    }

    private static boolean tick(ShooterPidfControl control, double multiplier, double seconds) throws Exception {
        // FTCLib 2.1.1 hardcodes System.nanoTime(). Advance only its clock in this test;
        // all integration, bounds and motor-control calculations use the actual production code.
        PIDFController pidf = pidf(control);
        Field clock = PIDFController.class.getDeclaredField("lastTimeStamp");
        clock.setAccessible(true);
        if (clock.getDouble(pidf) != 0) clock.setDouble(pidf, System.nanoTime() / 1e9 - seconds);
        return control.update(multiplier);
    }

    private static double run(double multiplier, double volts, double rpmPerPower,
                              double rpmLoss, boolean integral) throws Exception {
        Motor motor = new Motor();
        motor.volts = volts;
        ShooterPidfControl control = new ShooterPidfControl(motor.device, Collections.singletonList(motor.voltage));
        if (!integral) pidf(control).setI(0);
        for (int i = 0; i < 500; i++) {
            check(tick(control, multiplier, .02), "Healthy feedback continues running");
            motor.advance(.02, rpmPerPower, rpmLoss);
        }
        return motor.rpm;
    }

    public static void main(String[] args) throws Exception {
        // This model reproduces both reported equilibria with the old P/F gains.
        check(Math.abs(run(.48, 12, 6000, 288, false) - 2700) < 1, "Reproduce 2880 -> 2700 shortfall");
        check(Math.abs(run(.53, 12, 6000, 288, false) - 3000) < 1, "Reproduce 3180 -> 3000 shortfall");
        for (double volts : new double[]{10.5, 12, 14}) for (double multiplier : new double[]{.48, .53}) {
            double actual = run(multiplier, volts, 6000, 288, true);
            check(Math.abs(actual - multiplier * 6000) < 10,
                    "Close persistent RPM error at " + volts + " V: actual " + actual);
        }
        // A different motor/load model checks that this is feedback, not a hardcoded +180 RPM.
        check(Math.abs(run(.53, 12, 5500, 150, true) - 3180) < 10, "Adapt to another load");

        Motor motor = new Motor();
        motor.rpm = 3000;
        ShooterPidfControl control = new ShooterPidfControl(motor.device, Collections.singletonList(motor.voltage));
        tick(control, .53, .02);
        double firstPower = motor.power;
        tick(control, .53, .1);
        check(motor.power > firstPower + .003, "Persistent 180 RPM error adds power over time");
        for (int i = 0; i < 400; i++) {
            tick(control, .53, i % 2 == 0 ? .01 : .03);
            check(Math.abs(control.getIntegralPower()) <= .15000001, "Integral correction stays bounded");
        }
        check(Math.abs(control.getIntegralPower() - .15) < 1e-8, "FTCLib integral bounds use RPM-seconds");
        motor.rpm = 3180;
        tick(control, .53, .02);
        check(control.getIntegralPower() > .149, "Keep the learned load correction at zero error");
        motor.rpm = 3360;
        for (int i = 0; i < 500; i++) tick(control, .53, .02);
        check(Math.abs(control.getIntegralPower() + .15) < 1e-8, "Overspeed unwinds I to the negative bound");

        motor.rpm = 0;
        tick(control, .53, .02);
        check(Math.abs(control.getIntegralPower()) < 1e-9, "Spin-up clears accumulated I outside the zone");
        motor.rpm = 3000;
        for (int i = 0; i < 100; i++) tick(control, .53, .02);
        check(control.getIntegralPower() > .05, "Relearn near the target");
        motor.volts = 6;
        tick(control, .53, .02);
        check(motor.power == 1, "Low voltage saturates the final power command");
        motor.volts = 12; motor.rpm = 3180;
        tick(control, .53, .02);
        check(Math.abs(control.getIntegralPower()) < 1e-9, "Saturation clears integral for recovery");

        motor.rpm = 3000;
        for (int i = 0; i < 100; i++) tick(control, .53, .02);
        control.update(0);
        check(motor.power == 0 && control.getIntegralPower() == 0, "STOP immediately clears power and I");
        tick(control, .53, 20);
        check(Math.abs(control.getIntegralPower()) < 1e-9, "Stopped time is not integrated on restart");
        tick(control, .53, .1);
        check(control.getIntegralPower() > 0, "I resumes on following active samples");
        motor.volts = Double.NaN;
        check(!tick(control, .53, .02) && motor.power == 0 && control.getIntegralPower() == 0,
                "Voltage fault clears learned correction and stops output");
        motor.volts = 12; motor.rpm = 3180;
        tick(control, .53, .02);
        check(Math.abs(motor.power - .53) < 1e-9, "Fault recovery starts without stale integral");
        System.out.println("Steady RPM recovery, voltage/load variation, bounded I, spin-up, saturation, STOP and fault reset passed.");
    }
}
