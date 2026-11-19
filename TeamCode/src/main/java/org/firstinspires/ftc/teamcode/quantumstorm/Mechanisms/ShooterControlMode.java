package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/** Explicit API and run-mode selection for the two otherwise identical TeleOps. */
public enum ShooterControlMode {
    POWER(Constants.SHOOTER_POWER,Constants.SHOOTER_POWER_INCREMENT),
    VELOCITY(Constants.SHOOTER_RPM_MULTIPLIER,Constants.SHOOTER_RPM_MULTIPLIER_INCREMENT);

    public final double initialSetting,increment;

    ShooterControlMode(double initialSetting,double increment) {
        this.initialSetting=initialSetting;
        this.increment=increment;
    }

    public void configure(DcMotorEx motor) {
        ShooterMotorConfiguration.apply(motor);
        motor.setMode(this==VELOCITY?DcMotor.RunMode.RUN_USING_ENCODER:DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setOutput(motor,0);
    }

    /** Signed power fraction or signed RPM fraction; zero stops through the selected API. */
    public void setOutput(DcMotorEx motor,double setting) {
        double bounded=Double.isFinite(setting)?Math.max(-1,Math.min(1,setting)):0;
        if(this==VELOCITY) {
            // One RPM is six degrees per second; SDK converts using configured motor type.
            motor.setVelocity(Constants.SHOOTER_BASE_RPM*bounded*6.0,AngleUnit.DEGREES);
        } else {
            motor.setPower(bounded);
        }
    }
}
