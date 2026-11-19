package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/** Calibrated encoder velocity control for the default shooter TeleOp. */
public final class ShooterVelocityControl {
    private ShooterVelocityControl() { }

    public static void configure(DcMotorEx motor) {
        ShooterMotorConfiguration.apply(motor);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setOutput(motor,0);
    }

    /** Signed RPM fraction; zero stops through the velocity API. */
    public static void setOutput(DcMotorEx motor,double rpmMultiplier) {
        double bounded=Double.isFinite(rpmMultiplier)?Math.max(-1,Math.min(1,rpmMultiplier)):0;
        // One RPM is six degrees per second; SDK converts using configured motor type.
        motor.setVelocity(Constants.SHOOTER_BASE_RPM*bounded*6.0,AngleUnit.DEGREES);
    }
}
