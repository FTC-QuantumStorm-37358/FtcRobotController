package org.firstinspires.ftc.teamcode.biobuzz.ftc;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.biobuzz.FieldLayout;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose;

/** Hardware calibration only; the autonomous algorithm stays in robot/shared. */
public class FtcHardwareConfig {
    public boolean calibrated=false;
    public final FieldLayout field=new FieldLayout();
    public final Pose[] start={field.hive[0],field.hive[1]};
    public String frontLeft="left_front_drive",frontRight="right_front_drive",backLeft="left_back_drive",backRight="right_back_drive";
    public String intake="intake",shooter="shooter",leftIntakeServo="left_intake_servo",rightIntakeServo="right_intake_servo",feeder="shooter_servo";
    public String pinpoint="odometry_1",limelight="limelight",cameraTilt="limelight_tilt_servo";
    public double podXOffsetMm=0,podYOffsetMm=0,tiltUp=.3,tiltDown=.7;
    public boolean reverseXPod=false,reverseYPod=false;
    public int tagPipeline=0,ballPipeline=1;
    /** Confirm Limelight camera pose/extrinsics at tiltUp before using 3D heights. */
    public boolean hiveHeightCalibrated=false;
    /** Robot-space vertical axis: 2 for Z-up. Set only after verifying the pose convention. */
    public int tagHeightAxis=2;
    public double robotOriginHeightInches=0;
    public double shooterPower=.395;
    /** Configure four occupied-slot signals, or override createBallCounter with your actual sensing mechanism. */
    public String[] ballSlotNames={};
    public boolean occupiedSignalLow=true;
    public interface BallCounter { int count(); }
    public BallCounter createBallCounter(HardwareMap map) {
        if(ballSlotNames.length!=4)throw new IllegalStateException("Configure ballSlotNames or provide a sensor-backed BallCounter; the old SoftwareIndexer assumes counts.");
        final DigitalChannel[] slots=new DigitalChannel[4];
        for(int i=0;i<4;i++){slots[i]=map.get(DigitalChannel.class,ballSlotNames[i]);slots[i].setMode(DigitalChannel.Mode.INPUT);}
        return ()->{int count=0;for(DigitalChannel slot:slots)if(slot.getState()!=occupiedSignalLow)count++;return count;};
    }
    public void validate() {
        if(!calibrated)throw new IllegalStateException("Configure measured field poses, tag-to-face IDs, pod offsets/directions, camera tilt and ball counter in FtcHardwareConfig, then mark calibrated=true.");
        if(!hiveHeightCalibrated||tagHeightAxis<0||tagHeightAxis>2||!Double.isFinite(robotOriginHeightInches))
            throw new IllegalStateException("Calibrate robot-space tag heights at tiltUp, including camera pose, vertical axis and origin height.");
        if(shooterPower<0||shooterPower>1||!Double.isFinite(shooterPower))throw new IllegalArgumentException("Shooter power");
        for(Pose[] poses:new Pose[][]{start,field.hive,field.farHive,field.garden,field.flower})for(Pose p:poses)
            if(p==null||!p.finite()||!field.allowed(p))throw new IllegalArgumentException("Invalid field calibration pose");
        for(int[][] groups:new int[][][]{field.nearTags,field.farTags})for(int[] ids:groups)
            if(ids.length==0)throw new IllegalArgumentException("Missing hive AprilTag IDs");
    }
}
