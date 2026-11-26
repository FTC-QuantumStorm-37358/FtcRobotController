package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose;

/**
 * Autonomous-facing adapter for the calibrated shooter RPM table.
 *
 * <p>The table receives the Pinpoint field pose in inches and returns the
 * shooter RPM needed for that distance.  If a pose is not yet available, an
 * autonomous keeps its configured tested multiplier instead of cancelling a
 * timed shot.</p>
 */
final class AutonomousShooter {
    private final TableShooterCalibration calibration=new TableShooterCalibration();
    private double targetRpm=Double.NaN;

    double command(Pose pose,double fallbackMultiplier){
        double fallback=Double.isFinite(fallbackMultiplier)
                ?Math.max(0,Math.min(1,fallbackMultiplier)):0;
        if(pose==null||!pose.finite()){
            targetRpm=fallback*Constants.SHOOTER_BASE_RPM;
            return fallback;
        }
        TableShooterCalibration.Solution solution=calibration.atPose(
                pose.x,pose.y,Math.toDegrees(pose.heading));
        if(!solution.ready()||!Double.isFinite(solution.rpm)||solution.rpm<=0){
            targetRpm=fallback*Constants.SHOOTER_BASE_RPM;
            return fallback;
        }
        targetRpm=solution.rpm;
        return Math.max(0,Math.min(1,solution.rpm/Constants.SHOOTER_BASE_RPM));
    }

    double targetRpm(){return targetRpm;}
}
