package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Intake motor + left/right intake CR servos.
 * Same hardware and directions as QuantumStormTeleOp.
 */
public class Intake {

    private final DcMotor intake;
    private final CRServo rightServo;
    private final CRServo leftServo;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);
        rightServo = hardwareMap.get(CRServo.class, Constants.RIGHT_INTAKE_SERVO);
        leftServo = hardwareMap.get(CRServo.class, Constants.LEFT_INTAKE_SERVO);

        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightServo.setDirection(CRServo.Direction.FORWARD);
        leftServo.setDirection(CRServo.Direction.REVERSE);

        stop();
    }

    public void start() {
        setPower(Constants.INTAKE_POWER);
    }

    public void stop() {
        setPower(0);
    }

    private void setPower(double power) {
        intake.setPower(power);
        rightServo.setPower(power);
        leftServo.setPower(power);
    }
}
