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

    // Shooter 1 keeps the original "shooter" port name; shooter 2 is new
    public final static String SHOOTER_MOTOR = "shooter";
    public final static String SHOOTER_2_MOTOR = "shooter2";

    // CR servos
    public final static String RIGHT_INTAKE_SERVO = "right_intake_servo";
    public final static String LEFT_INTAKE_SERVO = "left_intake_servo";
    public final static String SHOOTER_SERVO = "shooter_servo";

    // =========================================================
    // POWER SETTINGS
    // =========================================================

    public final static double INTAKE_POWER = 1.0;

    // Used by the autonomous Shooter mechanism
    public final static double SHOOTER_POWER = 0.60;

    // TeleOp starting powers. Shooter 1 runs negative because its motor
    // is mounted facing the opposite way from shooter 2.
    public final static double SHOOTER_1_POWER = -0.60;
    public final static double SHOOTER_2_POWER = 0.60;

    public final static double SHOOTER_POWER_INCREMENT = 0.05;

    // Clamp bounds for the adjustable TeleOp shooter powers
    public final static double SHOOTER_POWER_MIN = -1.0;
    public final static double SHOOTER_POWER_MAX = 1.0;

    public final static double SHOOTER_SERVO_POWER = 1.0;

    // A trigger is analog; past this it counts as pressed
    public final static double TRIGGER_THRESHOLD = 0.5;

    // Power used by MecanumDriveTest when spinning one wheel at a time
    public final static double DRIVE_TEST_POWER = 0.3;

}
