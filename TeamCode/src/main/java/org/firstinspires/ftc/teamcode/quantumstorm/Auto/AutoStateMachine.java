package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.FieldLayout;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;

/** Camera-free hardware wrapper around the simulator's shared controller.
 * Calibrate the simulator start, Pinpoint offsets/directions, simulation waypoints
 * and shooter power on hardware. Unknown inventory stays -1, never assumed full.
 */
public final class AutoStateMachine {
    private final LinearOpMode opMode;
    private final AutoStartPosition position;
    private final MecanumDrive drive;
    private final PinpointOdometry odometry;
    private final Shooter shooter;
    private final Intake intake;
    private final ElapsedTime clock=new ElapsedTime();
    private final org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine routine;

    public AutoStateMachine(LinearOpMode opMode,AutoStartPosition position) {
        this.opMode=opMode;this.position=position;
        int alliance=position==AutoStartPosition.RED_1||position==AutoStartPosition.RED_2?0:1;
        FieldLayout field=new FieldLayout();Pose start=field.hive[alliance];
        drive=new MecanumDrive(opMode.hardwareMap);
        odometry=new PinpointOdometry(opMode.hardwareMap,new Pose2D(DistanceUnit.INCH,start.x,start.y,AngleUnit.RADIANS,start.heading));
        shooter=new Shooter(opMode.hardwareMap);intake=new Intake(opMode.hardwareMap);
        RobotIO io=new RobotIO(){
            public double nowSeconds(){return clock.seconds();}
            public Sensors readSensors(){
                odometry.update();double now=clock.seconds();
                Pose pose=new Pose(odometry.getX(),odometry.getY(),odometry.getPose().getHeading(AngleUnit.RADIANS));
                return new Sensors(pose,now,odometry.getStatus().name().equals("READY"),false,-1,Vision.empty(0,now));
            }
            public void apply(Outputs o){
                double[] wheels=o.wheelPowers();drive.setPowers(wheels[0],wheels[1],wheels[2],wheels[3]);
                if(o.shooterPower>0)shooter.spinUp();else shooter.stop();
                if(o.feederPower>0)shooter.feed();else shooter.stopFeed();
                if(o.intakePower>0)intake.start();else intake.stop();
            }
        };
        routine=new org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine(io,field,false);
        routine.timedFeedSeconds=Constants.AUTO_FEED_TIME_SEC;
        routine.timedPickupSeconds=Constants.AUTO_PICKUP_TIMEOUT_SEC;
        opMode.telemetry.addData("Autonomous",position.label);
        opMode.telemetry.addData("Required simulator start","X %.3f, Y %.3f in, heading %.1f deg",start.x,start.y,Math.toDegrees(start.heading));
        opMode.telemetry.addLine("Team One/Two use the SAME simulator start; not the old side-wall starts.");
        opMode.telemetry.addLine("Camera-free: Limelight and camera tilt servo are not used.");
        opMode.telemetry.addLine("Routine: shoot -> Garden -> shoot -> Flower -> shoot -> park at 26 s.");
        opMode.telemetry.addLine("Inventory unknown: timed pickup/feed attempts; Hive tilt unverified.");
        opMode.telemetry.addLine("Calibrate start, odometry, waypoints and shooter before testing.");
        opMode.telemetry.update();
    }

    public void run(){
        int alliance=position==AutoStartPosition.RED_1||position==AutoStartPosition.RED_2?0:1;
        clock.reset();routine.start(alliance,Constants.SHOOTER_POWER);
        try {
            // Bounded, stationary wait; included in the match's 26/30-second clock.
            while(opMode.opModeIsActive()&&clock.seconds()<2){
                odometry.update();
                if(odometry.getStatus().name().equals("READY")||odometry.getStatus().name().startsWith("FAULT"))break;
                opMode.idle();
            }
            while(opMode.opModeIsActive()&&routine.isRunning()){
                routine.tick();
                opMode.telemetry.addData("Autonomous",position.label);
                opMode.telemetry.addData("Routine",routine.status());
                opMode.telemetry.addData("AprilTags","Not required (camera-free routine)");
                opMode.telemetry.addData("Balls","Unknown (timed attempts)");
                odometry.addTelemetry(opMode.telemetry);opMode.telemetry.update();
                opMode.idle();
            }
        }finally{routine.stop();drive.stop();shooter.stop();intake.stop();}
    }
}
