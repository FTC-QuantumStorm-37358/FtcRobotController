package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.FieldLayout;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;
import org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine.Routine;
import org.firstinspires.ftc.teamcode.biobuzz.ftc.SafeShutdown;
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
        this(opMode,position,Routine.FULL);
    }
    public AutoStateMachine(LinearOpMode opMode,AutoStartPosition position,Routine selectedRoutine) {
        this.opMode=opMode;this.position=position;
        int alliance=position==AutoStartPosition.RED_1||position==AutoStartPosition.RED_2?0:1;
        FieldLayout field=new FieldLayout();Pose start=field.hive[alliance];
        initStage("Initializing drive motors");
        drive=new MecanumDrive(opMode.hardwareMap);
        initStage("Initializing odometry_1: verify Pinpoint wiring/configuration");
        odometry=new PinpointOdometry(opMode.hardwareMap,new Pose2D(DistanceUnit.INCH,start.x,start.y,AngleUnit.RADIANS,start.heading));
        initStage("Initializing shooter and intake");
        shooter=new Shooter(opMode.hardwareMap);intake=new Intake(opMode.hardwareMap);
        RobotIO io=new RobotIO(){
            private boolean stopped;
            public boolean stopRequested(){return opMode.isStopRequested()||Thread.currentThread().isInterrupted();}
            public double nowSeconds(){return clock.seconds();}
            public Sensors readSensors(){
                odometry.update();double now=clock.seconds();
                Pose pose=new Pose(odometry.getX(),odometry.getY(),odometry.getPose().getHeading(AngleUnit.RADIANS));
                return new Sensors(pose,now,odometry.isValid(),false,-1,Vision.empty(0,now));
            }
            public void apply(Outputs o){
                if(stopRequested()||o.forward==0&&o.strafeRight==0&&o.turnClockwise==0&&o.shooterPower==0&&o.feederPower==0&&o.intakePower==0){
                    if(!stopped){stopped=true;SafeShutdown.run(drive::stop,shooter::stop,intake::stop);}return;
                }
                stopped=false;
                double[] wheels=o.wheelPowers();drive.setPowers(wheels[0],wheels[1],wheels[2],wheels[3]);
                if(o.shooterPower>0)shooter.spinUp();else shooter.stop();
                if(o.feederPower>0)shooter.feed();else shooter.stopFeed();
                if(o.intakePower>0)intake.start();else intake.stop();
            }
        };
        routine=new org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine(io,field,false,selectedRoutine);
        routine.timedFeedSeconds=Constants.AUTO_FEED_TIME_SEC;
        routine.timedPickupSeconds=Constants.AUTO_PICKUP_TIMEOUT_SEC;
        opMode.telemetry.addData("Autonomous",position.label);
        opMode.telemetry.addData("Required simulator start","X %.3f, Y %.3f in, heading %.1f deg",start.x,start.y,Math.toDegrees(start.heading));
        opMode.telemetry.addLine("Team One/Two use the SAME simulator start; not the old side-wall starts.");
        opMode.telemetry.addLine("Camera-free: Limelight and camera tilt servo are not used.");
        opMode.telemetry.addLine(selectedRoutine==Routine.SINGLE_TILT
            ?"SINGLE TILT: opening four-ball shot attempt -> park immediately. No pickups."
            :"Routine: shoot -> Garden -> shoot -> Flower -> shoot -> park at 26 s.");
        if(selectedRoutine==Routine.SINGLE_TILT)opMode.telemetry.addLine("Assumes starting Hive cell is UP; partner handles second tilt. Tilt is not detected.");
        opMode.telemetry.addLine("Inventory unknown: timed pickup/feed attempts; Hive tilt unverified.");
        opMode.telemetry.addLine("Every shot cycle runs its full timer, even with an empty indexer.");
        opMode.telemetry.addLine("Calibrate start, odometry, waypoints and shooter before testing.");
        opMode.telemetry.update();
    }

    private void initStage(String message){
        if(opMode.isStopRequested()||Thread.currentThread().isInterrupted())throw new java.util.concurrent.CancellationException("INIT cancelled");
        opMode.telemetry.addData("Startup",message);opMode.telemetry.update();
        com.qualcomm.robotcore.util.RobotLog.ii("BioBuzzInit",message);
    }

    public void run(){
        int alliance=position==AutoStartPosition.RED_1||position==AutoStartPosition.RED_2?0:1;
        if(opMode.isStopRequested())return;
        clock.reset();
        try {
            routine.start(alliance,Constants.SHOOTER_POWER);
            // Bounded, stationary wait; included in the match's 26/30-second clock.
            while(opMode.opModeIsActive()&&clock.seconds()<2){
                odometry.update();
                opMode.telemetry.addData("Startup","Waiting for Pinpoint: %s",odometry.getStatus());opMode.telemetry.update();
                if(odometry.getStatus().name().equals("READY")||odometry.getStatus().name().startsWith("FAULT"))break;
                opMode.idle();
            }
            while(opMode.opModeIsActive()&&routine.isRunning()){
                routine.tick();
                opMode.telemetry.addData("Autonomous",position.label);
                opMode.telemetry.addData("Routine",routine.status());
                opMode.telemetry.addData("AprilTags","Not required (camera-free routine)");
                opMode.telemetry.addData("Navigation","Fixed simulator route; no route search");
                opMode.telemetry.addData("Balls","Unknown (timed attempts)");
                odometry.addTelemetry(opMode.telemetry);opMode.telemetry.update();
                opMode.idle();
            }
        }catch(java.util.concurrent.CancellationException e){
            // STOP is a normal exit, not an autonomous fault.
        }catch(RuntimeException e){
            com.qualcomm.robotcore.util.RobotLog.ee("BioBuzzAuto",e,"Autonomous hardware/controller fault");
            opMode.telemetry.addData("Autonomous stopped",e.getMessage());opMode.telemetry.update();
        }finally{routine.stop();}
    }
}
