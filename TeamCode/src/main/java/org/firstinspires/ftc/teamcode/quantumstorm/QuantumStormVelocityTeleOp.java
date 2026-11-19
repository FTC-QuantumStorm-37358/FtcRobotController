package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.ShooterControlMode;

/** Full two-driver TeleOp with encoder-regulated shooter RPM for comparison testing. */
@TeleOp(name = Constants.VELOCITY_TELEOP_NAME, group = "Quantum Storm")
public class QuantumStormVelocityTeleOp extends QuantumStormTeleOp {
    @Override
    protected ShooterControlMode getShooterControlMode() {
        return ShooterControlMode.VELOCITY;
    }
}
