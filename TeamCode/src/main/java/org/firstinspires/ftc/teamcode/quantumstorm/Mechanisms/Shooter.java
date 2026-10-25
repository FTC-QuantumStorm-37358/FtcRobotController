package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.ftc.SafeShutdown;

/**
 * Single shooter flywheel motor + the CR servo that feeds balls into it.
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
        apply(Constants.SHOOTER_POWER,0);
    }

    /** Shared positive demand scaled to the current signed TeleOp flywheel calibration. */
    public void apply(double power,double feedPower) {
        double scale=Constants.SHOOTER_POWER>0?Math.max(0,Math.min(1,power/Constants.SHOOTER_POWER)):0;
        shooter.setPower(Constants.SHOOTER_1_POWER*scale);
        feeder.setPower(scale>0?Math.max(-1,Math.min(1,feedPower))*Constants.SHOOTER_SERVO_POWER:0);
    }

    public void feed() {
        feeder.setPower(Constants.SHOOTER_SERVO_POWER);
    }

    public void stopFeed() {
        feeder.setPower(0);
    }

    public void stop() {
        SafeShutdown.run(()->shooter.setPower(0),()->feeder.setPower(0));
    }
}
