package org.firstinspires.ftc.teamcode.biobuzz;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

public final class TimedDriveChecks {
    private static final class IO implements RobotIO {
        double time;boolean stop;int reads;Outputs out=new Outputs();
        public double nowSeconds(){return time;}
        public boolean stopRequested(){return stop;}
        public Sensors readSensors(){reads++;throw new AssertionError("Timed replay must not read odometry, vision or inventory");}
        public void apply(Outputs o){out=o.copy();}
    }
    private static void check(boolean b,String message){if(!b)throw new AssertionError(message);}
    private static void zero(IO io){for(double v:io.out.wheelPowers())check(v==0,"All drive motors must stop");check(io.out.shooterPower==0&&io.out.feederPower==0&&io.out.intakePower==0,"Mechanisms must remain OFF");}
    public static void main(String[] args){
        for(AutoStateMachine.Routine mode:AutoStateMachine.Routine.values())for(int alliance=0;alliance<2;alliance++){
            IO io=new IO();TimedDriveReplay replay=new TimedDriveReplay(io,mode);replay.start(alliance);
            boolean moved=false,sawPark=false;
            for(int i=0;i<1501&&replay.isRunning();i++){
                io.time=i*.02;replay.tick();
                check(io.out.shooterPower==0&&io.out.feederPower==0&&io.out.intakePower==0,"Recorded outputs cannot activate mechanisms");
                moved|=Math.abs(io.out.forward)+Math.abs(io.out.strafeRight)+Math.abs(io.out.turnClockwise)>0;
                sawPark|=replay.getState()==AutoStateMachine.State.PARK;
            }
            check(moved&&sawPark&&!replay.isRunning(),"Recorded routine must finish in parking");check(io.reads==0,"No sensor reads");zero(io);
            io.time=0;replay.start(alliance);
            for(int i=0;i<1500;i++){
                io.time=i*.02;replay.tick();
                if(Math.abs(io.out.forward)+Math.abs(io.out.strafeRight)+Math.abs(io.out.turnClockwise)>0)break;
            }
            check(Math.abs(io.out.forward)+Math.abs(io.out.strafeRight)+Math.abs(io.out.turnClockwise)>0,"STOP test must interrupt active drive");
            io.stop=true;replay.tick();check(!replay.isRunning(),"Manual STOP while moving");zero(io);
            io.stop=false;io.time=0;replay.start(alliance);io.time=30;replay.tick();check(!replay.isRunning(),"30-second hard deadline");zero(io);
            io.time=0;replay.start(alliance);io.time=.3;replay.tick();check(!replay.isRunning(),"Delayed control loop cannot leave old drive command running");zero(io);
            io.time=1;replay.start(alliance);io.time=.5;replay.tick();check(!replay.isRunning(),"Backward clock");zero(io);
            io.time=0;replay.start(alliance);io.time=Double.NaN;replay.tick();check(!replay.isRunning(),"Invalid clock");zero(io);
        }
        System.out.println("Timed replay: zero sensor reads, full/single routines, both alliances, mechanism interlock, STOP, delayed loop and hard deadlines passed.");
    }
}
