package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.ftc.SafeShutdown;

/** Autonomous flywheel RPM feedback and paced feed, shared by both FTC hardware adapters. */
public class Shooter {
    private final DcMotorEx shooter;
    private final ShooterPidfControl control;
    private final CRServo feeder;
    private boolean feederPulseActive;
    private final org.firstinspires.ftc.teamcode.biobuzz.FeedCadence cadence =
            new org.firstinspires.ftc.teamcode.biobuzz.FeedCadence(
                    Constants.SPINDEXER_FEED_PULSE_SEC, Constants.SPINDEXER_SHOT_GAP_SEC);

    private double now() { return System.nanoTime() / 1e9; }

    public Shooter(HardwareMap hardwareMap) {
        this(hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR),
                hardwareMap.get(CRServo.class, Constants.SHOOTER_SERVO), hardwareMap.voltageSensor);
    }

    public Shooter(DcMotorEx shooter, CRServo feeder, Iterable<VoltageSensor> voltageSensors) {
        this.shooter = shooter;
        this.feeder = feeder;
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setDirection(CRServo.Direction.REVERSE);
        control = new ShooterPidfControl(shooter, voltageSensors);
        stop();
    }

    /** Reference-position spin-up; active autonomous loops supply their distance-derived multiplier. */
    public void spinUp() { apply(Constants.SHOOTER_AUTO_START_MULTIPLIER, 0); }

    /**
     * RobotIO.shooterPower is a fraction of SHOOTER_BASE_RPM, not motor power.
     * Call every loop, even when target RPM is unchanged, to update feedback and battery compensation.
     */
    public boolean apply(double rpmMultiplier, double feedPower) {
        boolean healthy;
        try {
            healthy = control.update(rpmMultiplier);
        } catch (RuntimeException failure) {
            // Attempt both stops even if a hardware sensor/motor call throws.
            stop();
            throw failure;
        }
        double demand = healthy && control.getTargetRpm() > 0 ? feedPower : 0;
        double feederCommand = cadence.apply(demand, now());
        feederPulseActive = feederCommand > 0;
        feeder.setPower(feederCommand * Constants.SHOOTER_SERVO_POWER);
        return healthy;
    }

    public double getTargetRpm() { return control.getTargetRpm(); }
    public double getMeasuredRpm() { return shooter.getVelocity(AngleUnit.DEGREES) / 6.0; }
    public ShooterPidfControl.Status getStatus() { return control.getStatus(); }
    public boolean isFeederPulseActive() { return feederPulseActive; }

    public void feed() {
        // A helper must not bypass the same fault/zero-target guard used by apply().
        double command = cadence.apply(control.getStatus() == ShooterPidfControl.Status.RUNNING
                && control.getTargetRpm() > 0 ? 1 : 0, now());
        feederPulseActive = command > 0;
        feeder.setPower(command * Constants.SHOOTER_SERVO_POWER);
    }

    public void stopFeed() {
        cadence.apply(0, now());
        feederPulseActive = false;
        feeder.setPower(0);
    }

    public void stop() {
        cadence.apply(0, now());
        feederPulseActive = false;
        SafeShutdown.run(control::stop, () -> feeder.setPower(0));
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter PIDF state", control.getStatus());
        telemetry.addData("PIDF sampled / target RPM", "%.0f / %.0f",
                control.getCurrentRpm(), control.getTargetRpm());
        telemetry.addData("Shooter battery (V)", "%.2f", control.getBatteryVolts());
        telemetry.addData("PIDF / compensated / applied power", "%.3f / %.3f / %.3f",
                control.getRawPower(), control.getCompensatedPower(), control.getAppliedPower());
        telemetry.addData("PIDF integral power / limit", "%.3f / %.2f",
                control.getIntegralPower(), Constants.SHOOTER_PIDF_MAX_INTEGRAL_POWER);
    }
}
