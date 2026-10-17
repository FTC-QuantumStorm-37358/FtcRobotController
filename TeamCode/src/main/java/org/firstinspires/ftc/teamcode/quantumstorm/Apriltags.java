package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;

import java.util.List;

@Autonomous(name = "Blue Hive: Move Under Top Cell")
public class Apriltags extends LinearOpMode {
    private static final int APRILTAG_PIPELINE = 0; // Change to your AprilTag pipeline number.
    private static final int FIRST_BLUE_HIVE_TAG_ID = 38;
    private static final int LAST_BLUE_HIVE_TAG_ID = 45;

    // Robot stops when the target is this close to directly above its center.
    private static final double CENTER_TOLERANCE_IN = 2.0;
    private static final double DRIVE_GAIN = 0.025;
    private static final double MAX_DRIVE_POWER = 0.18;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Looking for blue HIVE tags 38-45.");
        telemetry.addLine("Aim the Limelight upward so it sees the CELL underside.");
        telemetry.addLine("Press Start to move slowly underneath a visible tag.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                LLResultTypes.FiducialResult target = null;

                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (tags != null) {
                        for (LLResultTypes.FiducialResult tag : tags) {
                            int id = tag.getFiducialId();
                            // The blue BIOBUZZ HIVE uses IDs 38-41 and 42-45.
                            // Since its tags are on the underside, a tag seen from below
                            // is the upward-facing CELL. A downward-facing CELL is ignored
                            // because its tag faces away from the Limelight.
                            if (id >= FIRST_BLUE_HIVE_TAG_ID && id <= LAST_BLUE_HIVE_TAG_ID) {
                                target = tag;
                                telemetry.addData("Blue HIVE tag", "%d", id);
                                telemetry.addData("Tag angle", "left/right %.1f° | up/down %.1f°",
                                        tag.getTargetXDegrees(), tag.getTargetYDegrees());
                                break;
                            }
                        }
                    }
                }

                if (target == null) {
                    // Do not move toward a hidden/bottom CELL or any non-blue tag.
                    drive.stop();
                    telemetry.addLine("No visible top blue CELL tag (38-45); holding still.");
                } else {
                    Pose3D targetPose = target.getTargetPoseRobotSpace();
                    if (targetPose == null || targetPose.getPosition() == null) {
                        drive.stop();
                        telemetry.addLine("Tag visible, but 3D pose is missing; holding still.");
                    } else {
                        // Limelight robot-space position is converted to inches.
                        // FTC robot coordinates: +X forward and +Y left.
                        Position targetPosition = targetPose.getPosition().toUnit(DistanceUnit.INCH);
                        double forwardErrorIn = targetPosition.x;
                        double leftErrorIn = targetPosition.y;

                        if (Math.abs(forwardErrorIn) <= CENTER_TOLERANCE_IN
                                && Math.abs(leftErrorIn) <= CENTER_TOLERANCE_IN) {
                            drive.stop();
                            telemetry.addLine("Centered under the visible top CELL tag.");
                        } else {
                            double forward = clamp(forwardErrorIn * DRIVE_GAIN,
                                    -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
                            // MecanumDrive uses positive X to strafe right; target +Y is left.
                            double strafeRight = clamp(-leftErrorIn * DRIVE_GAIN,
                                    -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
                            drive.drive(forward, strafeRight, 0);
                            telemetry.addData("Action", "Moving under tag %d", target.getFiducialId());
                        }

                        telemetry.addData("Offset from tag", "forward %.1f in | left %.1f in",
                                forwardErrorIn, leftErrorIn);
                    }
                }

                telemetry.update();
                sleep(50);
            }
        } finally {
            drive.stop();
            limelight.stop();
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
