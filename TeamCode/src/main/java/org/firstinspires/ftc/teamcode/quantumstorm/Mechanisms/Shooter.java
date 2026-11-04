package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.ftc.SafeShutdown;

/**
 * Single shooter flywheel motor + the CR servo that feeds balls into it.
 * Same hardware and directions as QuantumStormTeleOp.
 */
public class Shooter {

    private final DcMotorEx shooter;
    private double targetRpm;
    private final CRServo feeder;
    private final org.firstinspires.ftc.teamcode.biobuzz.FeedCadence cadence=new org.firstinspires.ftc.teamcode.biobuzz.FeedCadence(Constants.SPINDEXER_FEED_PULSE_SEC,Constants.SPINDEXER_SHOT_GAP_SEC);
    private double now(){return System.nanoTime()/1e9;}

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR);
        feeder = hardwareMap.get(CRServo.class, Constants.SHOOTER_SERVO);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feeder.setDirection(CRServo.Direction.REVERSE);

        stop();
    }

    public void spinUp() {
        apply(Constants.SHOOTER_RPM_MULTIPLIER,0);
    }

    /** RobotIO.shooterPower is interpreted here as a fraction of SHOOTER_BASE_RPM. */
    public void apply(double rpmMultiplier,double feedPower) {
        double scale=Double.isFinite(rpmMultiplier)?Math.max(0,Math.min(1,rpmMultiplier)):0;
        targetRpm=Constants.SHOOTER_BASE_RPM*scale;
        shooter.setVelocity(targetRpm*6.0,AngleUnit.DEGREES);
        feeder.setPower(cadence.apply(scale>0?feedPower:0,now())*Constants.SHOOTER_SERVO_POWER);
    }

    public double getTargetRpm() { return targetRpm; }

    public double getMeasuredRpm() { return shooter.getVelocity(AngleUnit.DEGREES)/6.0; }

    public void feed() {
        feeder.setPower(cadence.apply(1,now())*Constants.SHOOTER_SERVO_POWER);
    }

    public void stopFeed() {
        cadence.apply(0,now());
        feeder.setPower(0);
    }

    public void stop() {
        targetRpm=0;
        cadence.apply(0,now());
        SafeShutdown.run(()->shooter.setVelocity(0),()->feeder.setPower(0));
    }
}
