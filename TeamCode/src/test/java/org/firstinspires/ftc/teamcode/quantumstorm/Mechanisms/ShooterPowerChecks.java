package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import java.lang.reflect.Proxy;
import java.util.Collections;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Real autonomous mechanism + FTCLib controller, instrumented motor/encoder/battery/feeder. */
public final class ShooterPowerChecks {
    private static void check(boolean condition, String why) {
        if (!condition) throw new AssertionError(why);
    }
    private static void near(double actual, double expected, String why) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-8,
                why + ": expected " + expected + ", got " + actual);
    }
    public static void main(String[] args) {
        double[] rpm = {2970}, volts = {12}, power = {Double.NaN}, feedPower = {Double.NaN};
        int[] reads = {0}, voltageReads = {0}, powerCalls = {0};
        boolean[] readThrows = {false};
        DcMotor.RunMode[] mode = {null};
        MotorConfigurationType originalType = new MotorConfigurationType();
        originalType.setTicksPerRev(2786);
        originalType.setGearing(99.5);
        originalType.setMaxRPM(60);
        MotorConfigurationType[] motorType = {originalType};
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, values) -> {
                    switch (method.getName()) {
                        case "getMotorType": return motorType[0];
                        case "setMotorType": motorType[0] = (MotorConfigurationType) values[0]; return null;
                        case "setMode": mode[0] = (DcMotor.RunMode) values[0]; return null;
                        case "setPower": power[0] = (double) values[0]; powerCalls[0]++; return null;
                        case "setVelocity": throw new AssertionError("Autonomous must never command Hub velocity");
                        case "getVelocity":
                            check(values.length == 1 && values[0] == AngleUnit.DEGREES, "Read degrees/sec");
                            reads[0]++;
                            if (readThrows[0]) throw new IllegalStateException("Disconnected encoder");
                            return rpm[0] * 6;
                        case "setDirection": case "setZeroPowerBehavior": return null;
                        default: throw new AssertionError("Unexpected motor call " + method.getName());
                    }
                });
        CRServo feeder = (CRServo) Proxy.newProxyInstance(CRServo.class.getClassLoader(),
                new Class<?>[]{CRServo.class}, (proxy, method, values) -> {
                    if (method.getName().equals("setPower")) feedPower[0] = (double) values[0];
                    return null;
                });
        VoltageSensor battery = (VoltageSensor) Proxy.newProxyInstance(VoltageSensor.class.getClassLoader(),
                new Class<?>[]{VoltageSensor.class}, (proxy, method, values) -> { voltageReads[0]++; return volts[0]; });
        Shooter shooter = new Shooter(motor, feeder, Collections.singletonList(battery));
        check(mode[0] == DcMotor.RunMode.RUN_WITHOUT_ENCODER, "Only software PIDF controls motor output");
        near(motorType[0].getTicksPerRev(), 28, "Shooter encoder corrected to 28 counts/rev");
        check(motorType[0] != originalType && originalType.getTicksPerRev() == 2786, "Other profiles unchanged");
        near(power[0], 0, "INIT stops flywheel"); near(feedPower[0], 0, "INIT stops feeder");
        shooter.spinUp();
        near(shooter.getTargetRpm(), 2970, "Default spin-up is 0.495 / 2970 RPM");
        near(shooter.getMeasuredRpm(), 2970, "Readback converts degrees/sec to RPM");
        int initialReads = reads[0], initialVoltageReads = voltageReads[0], initialWrites = powerCalls[0];
        for (double voltage : new double[]{12, 13.05, 10.5, 14}) {
            volts[0] = voltage;
            check(shooter.apply(.495, 0), "Healthy command");
            near(shooter.getTargetRpm(), 2970, "Battery never changes target RPM");
            near(power[0], .495 * 12 / voltage, "Battery compensation on every unchanged-target loop");
        }
        check(reads[0] == initialReads + 4 && voltageReads[0] == initialVoltageReads + 4
                && powerCalls[0] == initialWrites + 4, "Each loop reads sensors and writes updated power");
        volts[0] = 12; rpm[0] = 2700;
        shooter.apply(.495, 0);
        check(power[0] > .495, "RPM shortfall raises power at the same target");
        shooter.stop(); rpm[0] = 3600;
        shooter.apply(.495, 0);
        check(power[0] < .495, "Overspeed reduces power");
        shooter.stop(); rpm[0] = 2970;
        shooter.apply(.495, 1);
        check(feedPower[0] > 0 && shooter.isFeederPulseActive(), "Authorized healthy shot feeds");
        volts[0] = Double.NaN;
        check(!shooter.apply(.495, 1), "Invalid battery reported to adapter");
        near(power[0], 0, "Battery fault stops flywheel"); near(feedPower[0], 0, "Battery fault suppresses feed");
        check(!shooter.isFeederPulseActive(), "Fault is not recorded as a real feed pulse");
        shooter.feed(); near(feedPower[0], 0, "Feed helper cannot bypass fault");
        volts[0] = 12;
        check(shooter.apply(.495, 0), "Next healthy authorized loop can recover");
        near(power[0], .495, "Recovery starts with cleared integral");
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1, 1.01}) {
            check(!shooter.apply(invalid, 1), "Invalid target rejected");
            near(power[0], 0, "Bad target stops power"); near(feedPower[0], 0, "Bad target suppresses feed");
        }
        rpm[0] = Double.NaN;
        check(!shooter.apply(.495, 1), "Invalid encoder reading stops");
        near(feedPower[0], 0, "RPM fault suppresses feed");
        rpm[0] = 2970; shooter.apply(.495, 0); readThrows[0] = true;
        try { shooter.apply(.495, 1); throw new AssertionError("Expected sensor exception"); }
        catch (IllegalStateException expected) { }
        near(power[0], 0, "Sensor exception attempts motor stop"); near(feedPower[0], 0, "Sensor exception attempts feed stop");
        readThrows[0] = false;
        shooter.spinUp();
        int beforeStopReads = reads[0];
        shooter.stop(); shooter.feed();
        near(shooter.getTargetRpm(), 0, "STOP clears target");
        near(power[0], 0, "STOP zeros motor power"); near(feedPower[0], 0, "Stopped shooter cannot feed");
        check(reads[0] == beforeStopReads, "STOP does not depend on a sensor read");
        shooter = new Shooter(motor, feeder, Collections.emptyList());
        check(!shooter.apply(.495, 1), "Missing voltage sensor blocks autonomous shooter");
        near(power[0], 0, "Missing voltage power zero"); near(feedPower[0], 0, "Missing voltage feed zero");
        System.out.println("Autonomous PIDF assembly, 2970 RPM, live RPM/voltage updates, fault feed guard and STOP passed.");
    }
}
