package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.AutoShooterTarget;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose;

/**
 * Autonomous adapter for the same distance curve and reference calibration as the latest TeleOp.
 *
 * <p>The table receives the Pinpoint field pose in inches and returns the
 * shooter RPM needed for that distance.  If a pose is not yet available, an
 * autonomous keeps its configured tested multiplier instead of cancelling a
 * timed shot.</p>
 */
final class AutonomousShooter {
    private final AutoShooterTarget calibration;
    private double targetRpm=Double.NaN;

    AutonomousShooter(Pose reference){
        calibration=new AutoShooterTarget(reference.x,reference.y,Math.toDegrees(reference.heading));
    }

    double command(Pose pose,double fallbackMultiplier){
        double fallback=Double.isFinite(fallbackMultiplier)
                ?Math.max(0,Math.min(1,fallbackMultiplier)):0;
        if(fallback==0||pose==null||!pose.finite()){
            targetRpm=fallback*Constants.SHOOTER_BASE_RPM;
            return fallback;
        }
        calibration.updatePose(true,pose.x,pose.y,Math.toDegrees(pose.heading));
        if(!calibration.ready()){
            targetRpm=fallback*Constants.SHOOTER_BASE_RPM;
            return fallback;
        }
        // The configured multiplier is the speed at the reference pose, not a raw power command.
        // Default is 0.495; preserve explicit calibration overrides and zero as a disable command.
        double multiplier=Math.max(0,Math.min(1,fallback+(calibration.getBaseMultiplier()
                -Constants.SHOOTER_AUTO_START_MULTIPLIER)));
        targetRpm=multiplier*Constants.SHOOTER_BASE_RPM;
        return multiplier;
    }

    double targetRpm(){return targetRpm;}
}
