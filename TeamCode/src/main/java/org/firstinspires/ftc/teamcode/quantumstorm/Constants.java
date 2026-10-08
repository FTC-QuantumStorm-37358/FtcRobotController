package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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

    // Limelight camera + the servo that tilts it
    public final static String LIMELIGHT = "limelight";
    public final static String LIMELIGHT_TILT_SERVO = "limelight_tilt_servo";

    // =========================================================
    // POWER SETTINGS
    // =========================================================

    public final static double INTAKE_POWER = 1.0;
    public final static double SHOOTER_POWER = 0.60;
    public final static double SHOOTER_SERVO_POWER = 1.0;

    // Power used by MecanumDriveTest when spinning one wheel at a time
    public final static double DRIVE_TEST_POWER = 0.3;

    // =========================================================
    // LIMELIGHT
    // =========================================================

    // Tilt servo positions (0..1). TODO: find with "Servo Position Helper"
    public final static double LIMELIGHT_TILT_UP = 0.3;     // look at the hive AprilTag
    public final static double LIMELIGHT_TILT_DOWN = 0.7;   // look at balls on the floor

    // Pipeline numbers as set up in the Limelight web UI
    public final static int LIMELIGHT_APRILTAG_PIPELINE = 0;
    public final static int LIMELIGHT_BALL_PIPELINE = 1;

    // =========================================================
    // PINPOINT ODOMETRY
    // =========================================================

    public final static String PINPOINT = "odometry_1";   // I2C device name in Robot Configuration

    // Pod offsets from the robot's center (tracking point), in mm.
    // X pod (forward-counting): sideways distance, LEFT of center = positive.
    // Y pod (strafe-counting):  forward distance, FORWARD of center = positive.
    // TODO: measure on the robot
    public final static double PINPOINT_X_POD_OFFSET_MM = 0;
    public final static double PINPOINT_Y_POD_OFFSET_MM = 0;

    // Robot uses the goBILDA 4-Bar Odometry Pack (2 pods + 1 Pinpoint computer)
    public final static GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    // X pod must count UP driving forward, Y pod must count UP strafing LEFT.
    // Flip to REVERSED if the "Test: Pinpoint Pose" OpMode shows the wrong sign.
    public final static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public final static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // =========================================================
    // FIELD LOCATIONS
    // Team field frame: origin at center; inches; walls at +/-72.
    // Viewed from the audience: +X is right (Blue), +Y is away (rear).
    // Heading is CCW: 0 = +X, 90 = +Y, 180 = -X, -90 = -Y.
    // This is our chosen frame, not a coordinate convention mandated by FIRST.
    // Source: simulator/dist/model.js (rendered field), not the simulator's
    // provisional Java Garden waypoint, which is on the wrong alliance half.
    // Browser conversion: field X = browser x, field Y = -browser z.
    // Landmark coordinates match the simulator; verify the physical field.
    // *_POSE values are ROBOT CENTER destinations, not field-element centers.
    // Approach/start poses assume an 18 x 18 in robot with front intake/shooter.
    // Shooting distance and pickup offsets are provisional and must be tuned.
    // These destinations do not define obstacle-avoiding routes.
    // Seed Pinpoint with the matching start pose and verify pod offsets/signs.
    // =========================================================

    // Field landmarks (heading unused). Flowers are shared, not alliance-owned.
    public final static Pose2D REAR_FLOWER_CENTER     = fieldPose(-24, 69, 0);
    public final static Pose2D LEFT_FLOWER_CENTER     = fieldPose(-69, -24, 0);
    public final static Pose2D AUDIENCE_FLOWER_CENTER = fieldPose(24, -69, 0);
    public final static Pose2D RIGHT_FLOWER_CENTER    = fieldPose(69, 24, 0);
    public final static Pose2D RED_GARDEN_CENTER  = fieldPose(-60.5, -70.8, 0);
    public final static Pose2D BLUE_GARDEN_CENTER = fieldPose(60.5, 70.8, 0);
    // Loading Zones: approximately 11 in deep by 23 in wide.
    // Red: X [-72,-61], Y [24.5,47.5]; Blue is rotated 180 degrees.
    public final static Pose2D RED_LOADING_ZONE_CENTER  = fieldPose(-66.5, 36, 0);
    public final static Pose2D BLUE_LOADING_ZONE_CENTER = fieldPose(66.5, -36, 0);
    // One Hive per alliance, each with two Cells; pivots are 25.5 in apart.
    // Pivot position is unchanged by tipping. Robot shooting side changes.
    public final static Pose2D RED_HIVE_PIVOT  = fieldPose(-12.75, 0, 0);
    public final static Pose2D BLUE_HIVE_PIVOT = fieldPose(12.75, 0, 0);
    // Projected centers of the upward-facing Cell floors in the rendered model:
    // local z = +/-15.45, local height .4, tilt +/-30 deg, pivot height 43.95.
    public final static double SIM_HIVE_CELL_OFFSET_IN = 15.45*Math.cos(Math.PI/6)-0.4*Math.sin(Math.PI/6);
    public final static Pose2D RED_HIVE_NORMAL_CELL_CENTER = fieldPose(-12.75, SIM_HIVE_CELL_OFFSET_IN, 0);
    public final static Pose2D RED_HIVE_FLIPPED_CELL_CENTER = fieldPose(-12.75, -SIM_HIVE_CELL_OFFSET_IN, 0);
    public final static Pose2D BLUE_HIVE_NORMAL_CELL_CENTER = fieldPose(12.75, -SIM_HIVE_CELL_OFFSET_IN, 0);
    public final static Pose2D BLUE_HIVE_FLIPPED_CELL_CENTER = fieldPose(12.75, SIM_HIVE_CELL_OFFSET_IN, 0);

    // Proposed starting spots; not prescribed by Red/Blue 1/2 assignment.
    // Rear of an 18 in robot touches its alliance wall, outside Loading Zone.
    public final static Pose2D RED_1_START_POSE  = fieldPose(-63, -12, 0);
    public final static Pose2D RED_2_START_POSE  = fieldPose(-63, -60, 0);
    public final static Pose2D BLUE_1_START_POSE = fieldPose(63, 12, 180);
    public final static Pose2D BLUE_2_START_POSE = fieldPose(63, 60, 180);

    // Normal = rendered simulator's initial up Cell: Red +Y, Blue -Y.
    // This differs from the manual's initial up-Cell orientation; use the
    // appropriate measured state/poses on a competition field.
    // Robot poses are 60 in from the pivot, leaving room for an 18 in chassis.
    // Do not copy the simulator's 63.945 in pose: its chassis is asymmetric.
    public final static Pose2D RED_1_HIVE1_POSE  = fieldPose(-12.75, 60, -90);
    public final static Pose2D RED_2_HIVE1_POSE  = RED_1_HIVE1_POSE;
    public final static Pose2D BLUE_1_HIVE1_POSE = fieldPose(12.75, -60, 90);
    public final static Pose2D BLUE_2_HIVE1_POSE = BLUE_1_HIVE1_POSE;

    // After a tip, approach the newly upward-facing Cell from the other side.
    public final static Pose2D RED_1_HIVE1_FLIPPED_POSE  = fieldPose(-12.75, -60, 90);
    public final static Pose2D RED_2_HIVE1_FLIPPED_POSE  = RED_1_HIVE1_FLIPPED_POSE;
    public final static Pose2D BLUE_1_HIVE1_FLIPPED_POSE = fieldPose(12.75, 60, -90);
    public final static Pose2D BLUE_2_HIVE1_FLIPPED_POSE = BLUE_1_HIVE1_FLIPPED_POSE;

    // Garden: face front intake toward the audience/rear wall respectively.
    public final static Pose2D RED_GARDEN_POSE  = fieldPose(-60.5, -60.8, -90);
    public final static Pose2D BLUE_GARDEN_POSE = fieldPose(60.5, 60.8, 90);
    // Select one of the two Flowers on our AUTO half; approach its bottom.
    public final static Pose2D RED_FLOWER_POSE  = fieldPose(-24, 57.45, 90);
    public final static Pose2D BLUE_FLOWER_POSE = fieldPose(24, -57.45, -90);
    // Park with part of the footprint in our Loading Zone and a 1 in wall gap.
    // Wall contact is unnecessary for PARK and would forfeit end-of-AUTO LEAVE.
    public final static Pose2D RED_PARK_POSE  = fieldPose(-62, 36, 0);
    public final static Pose2D BLUE_PARK_POSE = fieldPose(62, -36, 180);

    // =========================================================
    // AUTONOMOUS ROUTINE
    // =========================================================

    public final static int AUTO_PRELOAD_BALLS = 4;                // Pollen loaded before the match
    public final static int AUTO_BALLS_TO_SHOOT = 4;               // shoot only when holding this many
    public final static double AUTO_PARK_TIME_SEC = 26.0;          // after this, go park (30 s auto, leaves time to drive)
    public final static int AUTO_MAX_RETRIES = 3;                  // recoverable errors before giving up and parking
    public final static double AUTO_APRILTAG_TIMEOUT_SEC = 3.0;    // no tag / not aligned -> ERROR
    public final static double AUTO_SHOOTER_SPINUP_SEC = 1.0;      // flywheel spin-up before feeding
    public final static double AUTO_FEED_TIME_SEC = 2.0;           // feed time to empty the indexer
    public final static double AUTO_AIM_TOLERANCE_DEG = 2.0;       // AprilTag tx within this = aligned
    public final static double AUTO_AIM_TURN_KP = 0.02;            // turn power per degree of tag offset
    public final static double AUTO_PICKUP_TIMEOUT_SEC = 3.0;      // time spent intaking at garden/flower
    public final static double AUTO_GROUND_SEARCH_TIMEOUT_SEC = 6.0;
    public final static double AUTO_GROUND_DRIVE_POWER = 0.3;      // forward speed toward a seen ball
    public final static double AUTO_GROUND_SEARCH_TURN_POWER = 0.2; // spin speed when no ball seen
    public final static double AUTO_GROUND_TURN_KP = 0.02;         // turn power per degree of ball offset

    // =========================================================
    // AUTONOMOUS MOVE-TO-POSE TUNING (used by Navigator)
    // =========================================================

    public final static double MOVE_POSITION_TOLERANCE_IN = 1.0;   // "arrived" when this close
    public final static double MOVE_HEADING_TOLERANCE_DEG = 2.0;
    public final static double MOVE_KP_TRANSLATION = 0.05;         // power per inch of error
    public final static double MOVE_KP_HEADING = 0.02;             // power per degree of error
    public final static double MOVE_MAX_POWER = 0.6;
    public final static double MOVE_TIMEOUT_SEC = 5.0;             // give up -> ERROR state

    private static Pose2D fieldPose(double xInches, double yInches, double headingDegrees) {
        return new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees);
    }
}
