package org.firstinspires.ftc.teamcode.quantumstorm;

public class Constants {

    // =========================================================
    // DEVICE NAMES
    // Must match the active Robot Configuration exactly (case-sensitive)
    // =========================================================

    // Drive motors
    public final static String FRONT_LEFT_DRIVE = "left_front_drive";
    public final static String FRONT_RIGHT_DRIVE = "right_front_drive";
    public final static String BACK_LEFT_DRIVE = "left_back_drive";
    public final static String BACK_RIGHT_DRIVE = "right_back_drive";

    // Mechanism motors
    public final static String INTAKE_MOTOR = "intake";
    public final static String SHOOTER_MOTOR = "shooter";

    // CR servos
    public final static String RIGHT_INTAKE_SERVO = "right_intake_servo";
    public final static String LEFT_INTAKE_SERVO = "left_intake_servo";
    public final static String SHOOTER_SERVO = "shooter_servo";

    // =========================================================
    // POWER SETTINGS
    // =========================================================

    public final static double INTAKE_POWER = 1.0;
    public final static double SHOOTER_POWER = 0.60;
    public final static double SHOOTER_POWER_INCREMENT = 0.05;
    public final static double SHOOTER_SERVO_POWER = 1.0;

    // Power used by MecanumDriveTest when spinning one wheel at a time
    public final static double DRIVE_TEST_POWER = 0.3;

}
