package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Shooter flywheel motor + the CR servo that feeds balls into it.
 * Same hardware and directions as QuantumStormTeleOp.
 */
public class Shooter {

    private final DcMotor shooter;
    private final CRServo feeder;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotor.class, Constants.SHOOTER_MOTOR);
        feeder = hardwareMap.get(CRServo.class, Constants.SHOOTER_SERVO);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setDirection(CRServo.Direction.REVERSE);

        stop();
    }

    public void spinUp() {
        shooter.setPower(Constants.SHOOTER_POWER);
    }

    public void feed() {
        feeder.setPower(Constants.SHOOTER_SERVO_POWER);
    }

    public void stopFeed() {
        feeder.setPower(0);
    }

    public void stop() {
        shooter.setPower(0);
        feeder.setPower(0);
    }
}
