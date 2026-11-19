package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/** Encoder scale for the robot's confirmed goBILDA 5203 1:1 shooter motor. */
public final class ShooterMotorConfiguration {
    private ShooterMotorConfiguration() { }

    public static void apply(DcMotorEx motor) {
        // Clone the profile: other motors may share the configured type object.
        MotorConfigurationType type=motor.getMotorType().clone();
        type.setTicksPerRev(Constants.SHOOTER_ENCODER_TICKS_PER_REV);
        type.setGearing(Constants.SHOOTER_GEAR_RATIO);
        type.setMaxRPM(Constants.SHOOTER_BASE_RPM);
        motor.setMotorType(type);
    }
}
