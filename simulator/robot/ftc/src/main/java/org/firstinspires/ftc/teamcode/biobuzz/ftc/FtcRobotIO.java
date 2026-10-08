package org.firstinspires.ftc.teamcode.biobuzz.ftc;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO;

/** FTC SDK adapter. No socket, desktop Java or browser dependency is deployed to the robot. */
public final class FtcRobotIO implements RobotIO,AutoCloseable {
    private final FtcHardwareConfig config;
    private final DcMotor fl,fr,bl,br,intake,shooter;
    private final CRServo leftIntake,rightIntake,feeder;
    private final GoBildaPinpointDriver pinpoint;
    private final Limelight3A limelight;
    private final Servo cameraTilt;
    private final FtcHardwareConfig.BallCounter counter;
    private final long epoch=System.nanoTime();
    private int pipeline=-1;
    public FtcRobotIO(HardwareMap map,FtcHardwareConfig config,int alliance) {
        config.validate();this.config=config;counter=config.createBallCounter(map);
        fl=map.get(DcMotor.class,config.frontLeft);fr=map.get(DcMotor.class,config.frontRight);
        bl=map.get(DcMotor.class,config.backLeft);br=map.get(DcMotor.class,config.backRight);
        intake=map.get(DcMotor.class,config.intake);shooter=map.get(DcMotor.class,config.shooter);
        leftIntake=map.get(CRServo.class,config.leftIntakeServo);rightIntake=map.get(CRServo.class,config.rightIntakeServo);feeder=map.get(CRServo.class,config.feeder);
        pinpoint=map.get(GoBildaPinpointDriver.class,config.pinpoint);limelight=map.get(Limelight3A.class,config.limelight);cameraTilt=map.get(Servo.class,config.cameraTilt);
        fl.setDirection(DcMotor.Direction.REVERSE);bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);br.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);shooter.setDirection(DcMotor.Direction.FORWARD);
        for(DcMotor motor:new DcMotor[]{fl,fr,bl,br,intake})motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        for(DcMotor motor:new DcMotor[]{fl,fr,bl,br,intake,shooter})motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftIntake.setDirection(CRServo.Direction.REVERSE);rightIntake.setDirection(CRServo.Direction.FORWARD);feeder.setDirection(CRServo.Direction.REVERSE);
        apply(new Outputs());
        pinpoint.setOffsets(config.podXOffsetMm,config.podYOffsetMm,DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(config.reverseXPod?GoBildaPinpointDriver.EncoderDirection.REVERSED:GoBildaPinpointDriver.EncoderDirection.FORWARD,
            config.reverseYPod?GoBildaPinpointDriver.EncoderDirection.REVERSED:GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.recalibrateIMU();Pose p=config.start[alliance];pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,p.x,p.y,AngleUnit.RADIANS,p.heading));
        limelight.start();
    }
    public double nowSeconds() { return (System.nanoTime()-epoch)/1e9; }
    public Sensors readSensors() {
        pinpoint.update();Pose2D p=pinpoint.getPosition();double now=nowSeconds();
        Pose pose=new Pose(p.getX(DistanceUnit.INCH),p.getY(DistanceUnit.INCH),p.getHeading(AngleUnit.RADIANS));
        boolean valid=pinpoint.getDeviceStatus().name().equals("READY");
        LLResult result=limelight.getLatestResult();Vision vision=Vision.empty(pipeline,now);
        int actual=pipeline==0?config.tagPipeline:config.ballPipeline;
        if(result!=null&&result.isValid()&&result.getPipelineIndex()==actual){
            List<Target> targets=new ArrayList<Target>();
            if(pipeline==0){List<LLResultTypes.FiducialResult> tags=result.getFiducialResults();if(tags!=null)for(LLResultTypes.FiducialResult t:tags)
                targets.add(new Target(t.getFiducialId(),t.getTargetXDegrees(),t.getTargetYDegrees()));}
            else targets.add(new Target(-1,result.getTx(),result.getTy()));
            double age=Math.max(0,result.getStaleness()+result.getCaptureLatency()+result.getTargetingLatency())/1000;
            vision=new Vision(!targets.isEmpty(),pipeline,now-age,targets.toArray(new Target[0]));
        }
        return new Sensors(pose,now,valid,limelight.isConnected(),counter.count(),vision);
    }
    public void apply(Outputs o) {
        double[] w=o.wheelPowers();
        fl.setPower(w[0]);fr.setPower(w[1]);bl.setPower(w[2]);br.setPower(w[3]);
        intake.setPower(o.intakePower);leftIntake.setPower(o.intakePower);rightIntake.setPower(o.intakePower);
        shooter.setPower(o.shooterPower);feeder.setPower(o.feederPower);
        if(pipeline!=o.pipeline){pipeline=o.pipeline;cameraTilt.setPosition(pipeline==0?config.tiltUp:config.tiltDown);limelight.pipelineSwitch(pipeline==0?config.tagPipeline:config.ballPipeline);}
    }
    public void close() { try{apply(new Outputs());}finally{limelight.stop();} }
}
