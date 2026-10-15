package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;

import java.util.List;

/** Uses the Limelight 3A color pipeline to approach a visible blue or yellow target. */
@Autonomous(name = "Approach Blue or Yellow", group = "Vision")
public class PollenAndAprilTagDetection extends LinearOpMode {

    // This must match the Limelight device name in Robot Configuration.
    private static final String LIMELIGHT_NAME = "limelight 3A";
    private static final double APPROACH_POWER = 0.12;
    private static final double STRAFE_GAIN = 0.008;
    private static final double MAX_STRAFE_POWER = 0.12;
    private static final double CENTER_TOLERANCE_DEGREES = 2.0;
    private static final double STOP_TARGET_AREA_PERCENT = 12.0;

    @Override
    public void runOpMode() {
        // Set up the drive motors and the intake motor.
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        DcMotor intake = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        // Limelight3A is the hardware type; it is not a WebcamName.
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        // Choose the color pipeline number from Constants, then start the camera.
        limelight.pipelineSwitch(Constants.LIMELIGHT_COLOR_PIPELINE);
        limelight.start();

        telemetry.addLine("Approaching the largest target from the Limelight color pipeline.");
        telemetry.addLine("Set that pipeline to detect blue or yellow objects.");
        telemetry.addLine("Press Start to begin; stop the OpMode from the Driver Station.");
        telemetry.update();
        waitForStart();

        try {
            // Keep checking for a target until the OpMode is stopped.
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                LLResultTypes.ColorResult target = findLargestColorTarget(result);

                if (target == null) {
                    // No target means stop the robot and intake.
                    drive.stop();
                    intake.setPower(0);
                    telemetry.addLine("No color target visible; stopped.");
                } else {
                    // Limelight reports horizontal angle in degrees and target area in percent.
                    double angle = target.getTargetXDegrees();
                    double area = target.getTargetArea();

                    if (area >= STOP_TARGET_AREA_PERCENT) {
                        // A large target is close enough, so stop driving.
                        drive.stop();
                        intake.setPower(Constants.INTAKE_POWER);
                        telemetry.addLine("Target is close; stopped and intaking.");
                    } else {
                        // Move forward and steer toward the target's center.
                        double strafe = Math.abs(angle) <= CENTER_TOLERANCE_DEGREES
                                ? 0.0
                                : clamp(angle * STRAFE_GAIN,
                                        -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
                        drive.drive(APPROACH_POWER, strafe, 0);
                        intake.setPower(Constants.INTAKE_POWER);
                        telemetry.addData("Action", "Approaching target");
                    }

                    telemetry.addData("Target angle", "%.1f degrees", angle);
                    telemetry.addData("Target area", "%.1f%%", area);
                }

                telemetry.update();
                sleep(50);
            }
        } finally {
            // Stop everything when this OpMode ends.
            intake.setPower(0);
            drive.stop();
            limelight.stop();
        }
    }

    private LLResultTypes.ColorResult findLargestColorTarget(LLResult result) {
        // Invalid camera data means no target was found.
        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
        if (colorTargets == null || colorTargets.isEmpty()) {
            return null;
        }

        LLResultTypes.ColorResult largest = null;
        // Keep the target that takes up the most camera view.
        for (LLResultTypes.ColorResult target : colorTargets) {
            if (largest == null || target.getTargetArea() > largest.getTargetArea()) {
                largest = target;
            }
        }
        return largest;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /*
     * Coding Translations
     *
     * LIMELIGHT_NAME = the Limelight's name in the Robot Configuration.
     * pipelineSwitch(...) = tells the Limelight which saved pipeline to use.
     * getLatestResult() = gets the newest information from the camera.
     * getColorResults() = gets the color targets found by that pipeline.
     * getTargetXDegrees() = tells how far the target is left or right.
     * getTargetArea() = tells how much of the camera view the target fills.
     * drive(...) = moves the robot; intake.setPower(...) runs the intake motor.
     * findLargestColorTarget(...) = returns the biggest visible color target.
     * clamp(...) = keeps the steering power within the allowed range.
     */
}
