package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.biobuzz.FieldLayout;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;
import org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine.Routine;
import org.firstinspires.ftc.teamcode.biobuzz.TimedDriveReplay;
import org.firstinspires.ftc.teamcode.biobuzz.ftc.SafeShutdown;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.Shooter;

/** Odometry-free, drive-only playback of a simulator recording. No sensor hardware is mapped. */
public final class AutoStateMachine {
    private final LinearOpMode opMode;
    private final AutoStartPosition position;
    private final ElapsedTime clock=new ElapsedTime();
    private final TimedDriveReplay replay;
    public AutoStateMachine(LinearOpMode opMode,AutoStartPosition position){this(opMode,position,Routine.FULL);}
    public AutoStateMachine(LinearOpMode opMode,AutoStartPosition position,Routine selectedRoutine){
        this.opMode=opMode;this.position=position;
        initStage("Initializing drive motors; timed replay, no odometry");
        final MecanumDrive drive=new MecanumDrive(opMode.hardwareMap);
        initStage("Initializing and disabling shooter / feeder / intake");
        final Shooter shooter=new Shooter(opMode.hardwareMap);final Intake intake=new Intake(opMode.hardwareMap);
        RobotIO io=new RobotIO(){
            private boolean stopped;
            public boolean stopRequested(){return opMode.isStopRequested()||Thread.currentThread().isInterrupted();}
            public double nowSeconds(){return clock.seconds();}
            public Sensors readSensors(){throw new UnsupportedOperationException("Timed playback must never read sensors");}
            public void apply(Outputs o){
                if(stopRequested()||o.forward==0&&o.strafeRight==0&&o.turnClockwise==0){
                    if(!stopped){stopped=true;SafeShutdown.run(drive::stop,shooter::stop,intake::stop);}return;
                }
                stopped=false;double[] w=o.wheelPowers();drive.setPowers(w[0],w[1],w[2],w[3]);
                // Independent hardware interlock: playback cannot activate scoring mechanisms.
                shooter.stop();intake.stop();
            }
        };
        replay=new TimedDriveReplay(io,selectedRoutine);
        int alliance=alliance();Pose start=new FieldLayout().hive[alliance];
        opMode.telemetry.addData("Autonomous",position.label);
        opMode.telemetry.addLine("TIMED REPLAY: odometry_1 is NOT required, initialized or read.");
        opMode.telemetry.addLine("Limelight, AprilTags and ball count are NOT used. Mechanisms remain OFF.");
        opMode.telemetry.addData("Required physical start","X %.3f, Y %.3f in, heading %.1f deg",start.x,start.y,Math.toDegrees(start.heading));
        opMode.telemetry.addLine("Team One/Two share their alliance start. Place and align the robot manually.");
        opMode.telemetry.addData("Recorded duration","%.2f seconds; original simulator pauses retained",replay.duration());
        opMode.telemetry.addLine("OPEN LOOP: cannot correct slipping, wrong speed or obstacles. Clear field; keep STOP ready.");
        opMode.telemetry.update();
    }
    private int alliance(){return position==AutoStartPosition.RED_1||position==AutoStartPosition.RED_2?0:1;}
    private void initStage(String message){
        if(opMode.isStopRequested()||Thread.currentThread().isInterrupted())throw new java.util.concurrent.CancellationException("INIT cancelled");
        opMode.telemetry.addData("Startup",message);opMode.telemetry.update();
    }
    public void run(){
        if(opMode.isStopRequested())return;clock.reset();
        try{
            replay.start(alliance());
            while(opMode.opModeIsActive()&&replay.isRunning()){
                replay.tick();opMode.telemetry.addData("Autonomous",position.label);
                opMode.telemetry.addData("Routine",replay.status());opMode.telemetry.addData("State",replay.getState());
                opMode.telemetry.addData("Last error",replay.lastError().isEmpty()?"None":replay.lastError());
                opMode.telemetry.addData("Odometry / vision / inventory","NOT USED (timer playback)");
                opMode.telemetry.addData("Mechanisms","DISABLED");opMode.telemetry.update();opMode.idle();
            }
        }catch(java.util.concurrent.CancellationException ignored){/* STOP is normal. */}
        catch(RuntimeException e){
            com.qualcomm.robotcore.util.RobotLog.ee("BioBuzzReplay",e,"Timed autonomous hardware fault");
            opMode.telemetry.addData("Autonomous stopped",e.getMessage());opMode.telemetry.update();
        }finally{replay.stop();}
    }
}
