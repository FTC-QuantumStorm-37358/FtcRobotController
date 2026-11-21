package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/** Software RPM feedback for the separate manual shooter test; the Hub supplies raw motor power. */
public final class ShooterPidfControl {
    public enum Status { STOPPED, RUNNING, INVALID_TARGET, INVALID_RPM, INVALID_VOLTAGE, INVALID_OUTPUT }

    private final DcMotorEx motor;
    private final Iterable<VoltageSensor> voltageSensors;
    private final PIDFController controller;
    private Status status = Status.STOPPED;
    private double currentRpm = Double.NaN, targetRpm, batteryVolts = Double.NaN;
    private double rawPower, compensatedPower, appliedPower;

    public ShooterPidfControl(DcMotorEx motor, Iterable<VoltageSensor> voltageSensors) {
        this(motor, voltageSensors, new PIDFController(Constants.SHOOTER_PIDF_P,
                Constants.SHOOTER_PIDF_I, Constants.SHOOTER_PIDF_D,
                Constants.SHOOTER_PIDF_F));
    }

    ShooterPidfControl(DcMotorEx motor, Iterable<VoltageSensor> voltageSensors, PIDFController controller) {
        this.motor = motor;
        this.voltageSensors = voltageSensors;
        this.controller = controller;
        ShooterMotorConfiguration.apply(motor);
        // Encoder readings remain available; disable the Hub's competing velocity feedback loop.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    /** Call once each active TeleOp loop. Returns false on a fault so the caller also stops feeding. */
    public boolean update(double multiplier) {
        if (!Double.isFinite(multiplier) || multiplier < 0 || multiplier > 1) {
            return fault(Status.INVALID_TARGET);
        }
        if (multiplier == 0) {
            stop();
            return true;
        }
        targetRpm = Constants.SHOOTER_BASE_RPM * multiplier;
        currentRpm = motor.getVelocity(AngleUnit.DEGREES) / 6.0;
        if (!Double.isFinite(currentRpm)) return fault(Status.INVALID_RPM);
        batteryVolts = readBatteryVoltage();
        if (!Double.isFinite(batteryVolts)) return fault(Status.INVALID_VOLTAGE);

        rawPower = controller.calculate(currentRpm, targetRpm);
        compensatedPower = rawPower * Constants.SHOOTER_PIDF_REFERENCE_VOLTS / batteryVolts;
        if (!Double.isFinite(compensatedPower)) return fault(Status.INVALID_OUTPUT);
        // This entry shoots forward only. Excess speed reduces power to zero rather than reversing.
        appliedPower = Math.max(0, Math.min(1, compensatedPower));
        if (compensatedPower <= 0 || compensatedPower >= 1) controller.clearTotalError();
        motor.setPower(appliedPower);
        status = Status.RUNNING;
        return true;
    }

    private double readBatteryVoltage() {
        double minimum = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double value = sensor.getVoltage();
            // Both REV Hubs share the battery. Ignore absent/broken readings, including implausibly low ones.
            if (Double.isFinite(value) && value >= 6 && value <= 16.8) minimum = Math.min(minimum, value);
        }
        return minimum;
    }

    private boolean fault(Status failure) {
        stop();
        status = failure;
        return false;
    }

    /** STOP never reads sensors or calculates PID error; no reverse/braking pulse is generated. */
    public void stop() {
        motor.setPower(0);
        controller.reset();
        targetRpm = rawPower = compensatedPower = appliedPower = 0;
        if (status == Status.RUNNING) status = Status.STOPPED;
        // Retain a fault message while stopped; a successful fresh start clears it.
    }

    public Status getStatus() { return status; }
    public double getCurrentRpm() { return currentRpm; }
    public double getTargetRpm() { return targetRpm; }
    public double getBatteryVolts() { return batteryVolts; }
    public double getRawPower() { return rawPower; }
    public double getCompensatedPower() { return compensatedPower; }
    public double getAppliedPower() { return appliedPower; }
}
