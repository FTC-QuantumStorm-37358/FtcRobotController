package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.FieldLayout;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO;
import org.firstinspires.ftc.teamcode.biobuzz.RobotController;
import org.firstinspires.ftc.teamcode.biobuzz.TeamOnePollenAuto;
import org.firstinspires.ftc.teamcode.biobuzz.TeamTwoHiveAuto;
import org.firstinspires.ftc.teamcode.quantumstorm.Vision.TeamOneLimelight;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;
import org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine.Routine;
import org.firstinspires.ftc.teamcode.biobuzz.ftc.SafeShutdown;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PollenInventory;

/** Team One recovers pollen; Team Two gates opposite-wall shots on live own-Hive tags.
 * Calibrate start, Pinpoint offsets/directions, waypoints and shooter power.
 * Unknown inventory uses bounded attempts, never invented ball counts.
 */
public final class AutoStateMachine implements AutoCloseable {
    private final LinearOpMode opMode;
    private final AutoStartPosition position;
    private final MecanumDrive drive;
    private final PinpointOdometry odometry;
    private final Shooter shooter;
    private final Intake intake;
    private final ElapsedTime clock=new ElapsedTime();
    private final RobotController routine;
    private final TeamOneLimelight camera;
    private final PollenInventory inventory;
    private boolean closed;
    private final boolean robotRelative;
    private final org.firstinspires.ftc.teamcode.biobuzz.MirroredHiveTags mirroredTags;

    /** Canonical frame is re-anchored at the same physical role start on either alliance. */
    public AutoStateMachine(LinearOpMode opMode,boolean teamTwo){
        this(opMode,teamTwo?AutoStartPosition.RED_2:AutoStartPosition.RED_1,teamTwo?Routine.TEAM_TWO:Routine.TEAM_ONE,true);
    }
    private String label(){return robotRelative?(position==AutoStartPosition.RED_2?"Team 2 Auto":"Team 1 Auto")+" (robot-relative)":position.label;}

    public AutoStateMachine(LinearOpMode opMode,AutoStartPosition position) {
        this(opMode,position,position==AutoStartPosition.RED_2||position==AutoStartPosition.BLUE_2?Routine.TEAM_TWO:Routine.TEAM_ONE);
    }
    public AutoStateMachine(LinearOpMode opMode,AutoStartPosition position,Routine selectedRoutine) {
        this(opMode,position,selectedRoutine,false);
    }
    private AutoStateMachine(LinearOpMode opMode,AutoStartPosition position,Routine selectedRoutine,boolean robotRelative) {
        this.opMode=opMode;this.position=position;this.robotRelative=robotRelative;
        int alliance=position==AutoStartPosition.RED_1||position==AutoStartPosition.RED_2?0:1;
        FieldLayout field=new FieldLayout();Pose start=field.roleStart(alliance,selectedRoutine==Routine.TEAM_TWO);
        mirroredTags=robotRelative&&selectedRoutine==Routine.TEAM_TWO?new org.firstinspires.ftc.teamcode.biobuzz.MirroredHiveTags(field):null;
        initStage("Initializing drive motors");
        drive=new MecanumDrive(opMode.hardwareMap);
        initStage("Initializing odometry_1: verify Pinpoint wiring/configuration");
        odometry=new PinpointOdometry(opMode.hardwareMap,new Pose2D(DistanceUnit.INCH,start.x,start.y,AngleUnit.RADIANS,start.heading));
        initStage("Initializing shooter and intake; OFF until START");
        shooter=new Shooter(opMode.hardwareMap);intake=new Intake(opMode.hardwareMap);
        field.upperTagHeight=selectedRoutine==Routine.TEAM_TWO?Constants.TEAM_TWO_UPPER_TAG_HEIGHT_IN:Constants.TEAM_ONE_UPPER_TAG_HEIGHT_IN;
        field.lowerTagHeight=selectedRoutine==Routine.TEAM_TWO?Constants.TEAM_TWO_LOWER_TAG_HEIGHT_IN:Constants.TEAM_ONE_LOWER_TAG_HEIGHT_IN;
        initStage(selectedRoutine==Routine.TEAM_TWO?"Initializing Limelight: own raised far Cell required before feeding":"Initializing optional Limelight for Team One pollen recovery");
        camera=selectedRoutine==Routine.FULL?null:new TeamOneLimelight(opMode.hardwareMap,clock::seconds,selectedRoutine==Routine.TEAM_TWO);
        inventory=selectedRoutine==Routine.FULL?null:new PollenInventory(opMode.hardwareMap);
        RobotIO io=new RobotIO(){
            private boolean stopped;
            public boolean stopRequested(){return opMode.isStopRequested()||Thread.currentThread().isInterrupted();}
            public double nowSeconds(){return clock.seconds();}
            public Sensors readSensors(){
                odometry.update();double now=clock.seconds();
                Pose pose=new Pose(odometry.getX(),odometry.getY(),odometry.getPose().getHeading(AngleUnit.RADIANS));
                Vision vision=camera==null?Vision.empty(0,now):camera.read(now);
                if(mirroredTags!=null){
                    Pose shot=field.teamTwoWallHive[0];
                    boolean atShot=pose.distance(shot)<=1&&Math.abs(Pose.angle(pose.heading-shot.heading))<=Math.toRadians(2);
                    vision=mirroredTags.normalize(vision,now,atShot);
                }
                PollenInventory.Sample count=inventory==null?null:inventory.read();
                return new Sensors(pose,now,odometry.isValid(),camera!=null&&camera.connected(),count==null?-1:count.total,count==null?-1:count.pollen,vision);
            }
            public void apply(Outputs o){
                if(stopRequested()||o.forward==0&&o.strafeRight==0&&o.turnClockwise==0&&o.shooterPower==0&&o.feederPower==0&&o.intakePower==0){
                    if(!stopped){stopped=true;SafeShutdown.run(drive::stop,shooter::stop,intake::stop);}
                    // Stop actuators BEFORE any camera SDK call, including a pipeline transition.
                    if(camera!=null&&!stopRequested())camera.select(o.pipeline);
                    return;
                }
                stopped=false;
                double[] wheels=o.wheelPowers(Constants.AUTO_FORWARD_POWER_SIGN);drive.setPowers(wheels[0],wheels[1],wheels[2],wheels[3]);
                shooter.apply(o.shooterPower,o.feederPower);intake.setPower(o.intakePower*Constants.INTAKE_POWER);
                if(camera!=null&&!stopRequested())camera.select(o.pipeline);
            }
        };
        if(selectedRoutine==Routine.TEAM_ONE){
            TeamOnePollenAuto one=new TeamOnePollenAuto(io,field);
            one.spinUpSeconds=Constants.AUTO_SHOOTER_SPINUP_SEC;one.feedSeconds=Constants.AUTO_FEED_TIME_SEC;
            one.observationHeadingOffsetRadians=Math.toRadians(Constants.TEAM_ONE_TAG_VIEW_TURN_DEG);
            routine=one;
        }else if(selectedRoutine==Routine.TEAM_TWO){
            TeamTwoHiveAuto two=new TeamTwoHiveAuto(io,field);
            two.spinUpSeconds=Constants.AUTO_SHOOTER_SPINUP_SEC;two.feedSeconds=Constants.AUTO_FEED_TIME_SEC;
            routine=two;
        }else{
            org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine two=new org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine(io,field,false,selectedRoutine,false,false);
            two.parkingToleranceInches=2.0;two.waitUntilParkTime=true;routine=two;
        }
        opMode.telemetry.addData("Autonomous",label());
        if(robotRelative)opMode.telemetry.addLine("No alliance selection: place at the matching Team 1/2 start on either side. Coordinates use a local route reference, not absolute field color.");
        opMode.telemetry.addData(robotRelative?"Local route reference":"Required start","X %.3f, Y %.3f in, heading %.1f deg",start.x,start.y,Math.toDegrees(start.heading));
        opMode.telemetry.addData("Autonomous role",selectedRoutine);
        opMode.telemetry.addData("Drive forward power sign",Constants.AUTO_FORWARD_POWER_SIGN);
        opMode.telemetry.addLine("Start FRONT toward wall, rear shooter toward Hive. Retreat BACKWARD into the field after shooting.");
        opMode.telemetry.addLine(camera==null?"Legacy camera-free mode":camera.status());
        opMode.telemetry.addLine(selectedRoutine==Routine.TEAM_TWO?"Team 2: opposite wall -> VERIFY raised far Cell -> feed -> nearest Flower -> wall -> VERIFY -> feed -> park by 26 s.":"Team 1: preloads -> Garden pollen intake -> start-side-half park by 26 s.");
        opMode.telemetry.addLine("LIVE shooter/feed/intake after START. Load four pollen; clear the robot and firing path.");
        opMode.telemetry.addLine(inventory==null?"Inventory unknown: timed attempts do not invent ball counts.":inventory.status());
        opMode.telemetry.addLine(selectedRoutine==Routine.TEAM_TWO?"Pipeline 0: raised own far Cell required before AND during feed; no any-tag fallback.":camera==null?"No extra pickup; wait then park at 26 s; stop at 30 s.":"Team One: immediate preloads; yellow-only Limelight for Garden pickup; park by 26 s.");
        if(selectedRoutine==Routine.TEAM_ONE){Pose park=field.teamOneStartSidePark[alliance];opMode.telemetry.addData("Team One start-side-half park","X %.1f, Y %.1f in; nearest starting wall",park.x,park.y);}
        if(selectedRoutine==Routine.TEAM_TWO){Pose shot=field.teamTwoWallHive[alliance];opMode.telemetry.addData("Team Two wall shot","X %.2f, Y %.2f in, heading %.1f deg",shot.x,shot.y,Math.toDegrees(shot.heading));opMode.telemetry.addLine("Limelight must see the Hive behind the rear shooter while the front faces the wall.");}
        opMode.telemetry.addLine("Parking: within 2 inches -> all outputs OFF; no final heading correction.");
        opMode.telemetry.addLine("Calibrate start, odometry and drive speed; physical arrival times may differ.");
        opMode.telemetry.update();
    }

    private void initStage(String message){
        if(opMode.isStopRequested()||Thread.currentThread().isInterrupted())throw new java.util.concurrent.CancellationException("INIT cancelled");
        opMode.telemetry.addData("Startup",message);opMode.telemetry.update();
        com.qualcomm.robotcore.util.RobotLog.ii("BioBuzzInit",message);
    }

    private String lastError(){
        if(routine instanceof TeamOnePollenAuto)return ((TeamOnePollenAuto)routine).lastError();
        if(routine instanceof TeamTwoHiveAuto)return ((TeamTwoHiveAuto)routine).lastError();
        return ((org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine)routine).lastError();
    }

    public void run(){
        int alliance=position==AutoStartPosition.RED_1||position==AutoStartPosition.RED_2?0:1;
        if(opMode.isStopRequested())return;
        clock.reset();
        try {
            if(camera!=null)camera.reset();
            if(mirroredTags!=null)mirroredTags.reset();
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
                opMode.telemetry.addData("Autonomous",label());
                opMode.telemetry.addData("Routine",routine.status());
                if(mirroredTags!=null)opMode.telemetry.addData("Hive tag family",mirroredTags.observedFamily()<0?"Waiting for one unambiguous raised Hive at shot pose":mirroredTags.observedFamily()==0?"Red family observed automatically":"Blue family observed automatically");
                opMode.telemetry.addData("State",routine.getState());
                opMode.telemetry.addData("Last error",lastError().isEmpty()?"None":lastError());
                opMode.telemetry.addData("AprilTags",camera==null?"Not required (legacy mode)":camera.status());
                opMode.telemetry.addData("Navigation",routine instanceof TeamTwoHiveAuto?"Opposite-wall shots; fixed route; fresh own-tag gate":camera==null?"Fixed simulator route; no route search":"Camera pollen approach + collision-checked start-side-half park");
                opMode.telemetry.addData("Mechanisms",inventory==null?"LIVE hardware outputs; unknown inventory":inventory.status());
                odometry.addTelemetry(opMode.telemetry);opMode.telemetry.update();
                opMode.idle();
            }
        }catch(java.util.concurrent.CancellationException e){
            // STOP is a normal exit, not an autonomous fault.
        }catch(RuntimeException e){
            com.qualcomm.robotcore.util.RobotLog.ee("BioBuzzAuto",e,"Autonomous hardware/controller fault");
            opMode.telemetry.addData("Autonomous stopped",e.getMessage());opMode.telemetry.update();
        }finally{close();}
    }
    /** Also called when STOP is pressed during INIT before run() ever starts. */
    public void close(){
        if(closed)return;closed=true;
        SafeShutdown.run(routine::stop,drive::stop,shooter::stop,intake::stop,()->{if(camera!=null)camera.close();});
    }
}
