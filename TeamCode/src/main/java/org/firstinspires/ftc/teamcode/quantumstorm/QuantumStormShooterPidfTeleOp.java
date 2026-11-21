package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.ShooterPidfControl;

/** Manual RPM test for software PIDF; no odometry or automatic distance target is required. */
@TeleOp(name = "Biobuzz-Quantum Storm New Shooter Power Control", group = "Quantum Storm Test")
public class QuantumStormShooterPidfTeleOp extends QuantumStormTeleOp {
    private ShooterPidfControl shooterPidf;

    @Override protected double initialShooterSetting() {
        return Constants.SHOOTER_PIDF_TEST_DEFAULT_MULTIPLIER;
    }

    @Override protected String shooterControlLabel() { return "FTCLib PIDF / voltage-compensated power"; }

    @Override protected void configureShooterMotor(DcMotorEx motor) {
        shooterPidf = new ShooterPidfControl(motor, hardwareMap.voltageSensor);
    }

    @Override protected boolean applyShooterOutput(DcMotorEx motor, double multiplier) {
        return shooterPidf.update(multiplier);
    }

    @Override protected boolean feederRequiresShooterRunning() { return true; }

    @Override protected double clampShooterSetting(double setting) {
        return Math.max(0, super.clampShooterSetting(setting));
    }

    @Override protected void addShooterTargetTelemetry() {
        telemetry.addLine("PIDF TEST: A starts, B stops; BACK + LB/LT adjusts target by 60 RPM");
        telemetry.addLine("Provisional gains: tune P/F on hardware; I/D initially zero");
        telemetry.addData("Shooter PIDF state", shooterPidf.getStatus());
        telemetry.addData("PIDF sampled / target RPM", "%.0f / %.0f",
                shooterPidf.getCurrentRpm(), shooterPidf.getTargetRpm());
        telemetry.addData("Battery voltage during control (V)", "%.2f", shooterPidf.getBatteryVolts());
        telemetry.addData("PIDF / compensated / applied power", "%.3f / %.3f / %.3f",
                shooterPidf.getRawPower(), shooterPidf.getCompensatedPower(), shooterPidf.getAppliedPower());
        telemetry.addData("PIDF P / I / D / F", "%.6f / %.6f / %.6f / %.6f",
                Constants.SHOOTER_PIDF_P, Constants.SHOOTER_PIDF_I,
                Constants.SHOOTER_PIDF_D, Constants.SHOOTER_PIDF_F);
        telemetry.addLine("Feeder requires shooter ON; after a sensor fault, release A then press it to retry");
    }
}
