package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.biobuzz.FieldLayout;
import org.firstinspires.ftc.teamcode.biobuzz.HiveVision;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Target;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Vision;
import java.util.ArrayList;
import java.util.List;

/**
 * Limelight 3A on a tilt servo. One pipeline runs at a time:
 *  - tiltUp():   AprilTag pipeline, used before shooting at the Hive
 *  - tiltDown(): ball pipeline, used to find balls on the floor
 *
 * If the camera or servo is missing from the Robot Configuration, this does not
 * crash INIT: isConnected() returns false and every query reports "nothing seen",
 * so the autonomous self-check can send the robot to ERROR -> PARK instead.
 */
public class LimelightCamera {

    private final Limelight3A limelight;   // null if not in the configuration
    private final Servo tiltServo;         // null if not in the configuration
    private HiveVision hiveVision=new HiveVision(new FieldLayout(),0);
    private boolean redHive=true,expectedFlipped;
    private double frameTimestamp=Double.NaN,frameCapturedAt,tiltChangedAt;
    private int requestedPipeline=-1;
    public void selectHive(boolean red,boolean flipped) {
        if(red!=redHive){redHive=red;hiveVision=new HiveVision(new FieldLayout(),red?0:1);}
        expectedFlipped=flipped;
    }

    public LimelightCamera(HardwareMap hardwareMap) {
        limelight = hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT);
        tiltServo = hardwareMap.tryGet(Servo.class, Constants.LIMELIGHT_TILT_SERVO);
        if (limelight != null) {
            limelight.start();
        }
        tiltUp();
    }

    public boolean isConnected() {
        return limelight != null && tiltServo != null && limelight.isConnected();
    }

    public void tiltUp() {
        setTiltAndPipeline(Constants.LIMELIGHT_TILT_UP, Constants.LIMELIGHT_APRILTAG_PIPELINE);
    }

    public void tiltDown() {
        setTiltAndPipeline(Constants.LIMELIGHT_TILT_DOWN, Constants.LIMELIGHT_BALL_PIPELINE);
    }

    // ---- AprilTag pipeline ----

    /** True only for a fresh, stable upward Cell on our alliance's shooting side. */
    public boolean isAprilTagVisible() {
        return aim()!=null;
    }

    /** Horizontal angle to the seen AprilTag in degrees (right = positive), or 0 if none. */
    public double getAprilTagAngle() {
        Target target=aim();
        return target==null?0:target.tx;
    }

    /**
     * Null means unknown/in motion. Both directions require calibrated heights
     * over distinct fresh frames; seeing the far tag's ID alone proves no tip.
     */
    public Boolean getHiveState() {
        updateHive();int state=hiveVision.state(now());return state<0?null:state==1;
    }
    public boolean isHiveFlipped() { return Boolean.TRUE.equals(getHiveState()); }
    private static double now(){return System.nanoTime()/1e9;}
    private Target aim(){updateHive();return hiveVision.aim(now(),expectedFlipped);}
    private void updateHive() {
        double time=now();LLResult result=resultFor(Constants.LIMELIGHT_APRILTAG_PIPELINE);
        if(result==null){hiveVision.update(Vision.empty(0,time),time);return;}
        double age=(result.getStaleness()+result.getCaptureLatency()+result.getTargetingLatency())/1000;
        if(result.getTimestamp()!=frameTimestamp){frameTimestamp=result.getTimestamp();frameCapturedAt=time-age;}
        List<Target> targets=new ArrayList<>();
        for(FiducialResult tag:result.getFiducialResults()) {
            double height=Double.NaN;
            if(Constants.LIMELIGHT_HIVE_HEIGHT_CALIBRATED&&tag.getTargetPoseRobotSpace()!=null){
                Position p=tag.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
                int axis=Constants.LIMELIGHT_TAG_HEIGHT_AXIS;
                height=(axis==0?p.x:axis==1?p.y:p.z)+Constants.LIMELIGHT_ROBOT_ORIGIN_HEIGHT_IN;
            }
            targets.add(new Target(tag.getFiducialId(),tag.getTargetXDegrees(),tag.getTargetYDegrees(),height));
        }
        hiveVision.update(new Vision(true,0,frameCapturedAt,targets.toArray(new Target[0])),time);
    }

    // ---- Ball pipeline ----

    /** True if the ball pipeline currently sees a ball. */
    public boolean isBallVisible() {
        return resultFor(Constants.LIMELIGHT_BALL_PIPELINE) != null;
    }

    /** Horizontal angle to the seen ball in degrees (right = positive), or 0 if none. */
    public double getBallAngle() {
        LLResult result = resultFor(Constants.LIMELIGHT_BALL_PIPELINE);
        return result != null ? result.getTx() : 0;
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    // ---- helpers ----

    private void setTiltAndPipeline(double tilt, int pipeline) {
        if(requestedPipeline!=pipeline){
            requestedPipeline=pipeline;tiltChangedAt=now();frameTimestamp=Double.NaN;
            hiveVision.update(Vision.empty(0,now()),now());
        }
        if (tiltServo != null) {
            tiltServo.setPosition(tilt);
        }
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    /** Latest valid result from the given pipeline, or null (right after a switch results are from the old one). */
    private LLResult resultFor(int pipeline) {
        if (limelight == null) {
            return null;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getPipelineIndex() != pipeline
                || now()-tiltChangedAt<.20
                || result.getStaleness()+result.getCaptureLatency()+result.getTargetingLatency()>250) {
            return null;
        }
        return result;
    }
}
