package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Decision/safety fixtures only; Cannon tests independently verify real shots and pickup. */
public final class RecoveryChecks {
    private static final class IO implements RobotIO {
        double time,age;int count=4,pipeline,reads;boolean connected=true,valid=true,odom=true,stop;
        Pose pose=new FieldLayout().hive[0];Target[] targets=up();Outputs out=new Outputs();
        public double nowSeconds(){return time;}
        public boolean stopRequested(){return stop;}
        public Sensors readSensors(){reads++;return new Sensors(pose,time,odom,connected,count,new Vision(valid,pipeline,time-age,targets));}
        public void apply(Outputs outputs){out=outputs.copy();}
    }
    private static Target[] up(){return new Target[]{new Target(30,0,0,52.02),new Target(31,0,0,52.02)};}
    private static Target[] down(){return new Target[]{new Target(30,0,0,36.57),new Target(31,0,0,36.57)};}
    private static Target ball(double tx,double area,String color){return new Target(-1,tx,-10,Double.NaN,area,color);}
    private static void check(boolean condition,String reason){if(!condition)throw new AssertionError(reason);}
    private static void step(IO io,LimelightRecoveryAuto auto){io.time+=.04;io.pipeline=io.out.pipeline;auto.tick();}
    private static void stopped(IO io,String why){check(io.out.forward==0&&io.out.strafeRight==0&&io.out.turnClockwise==0&&io.out.shooterPower==0&&io.out.feederPower==0&&io.out.intakePower==0,why);}
    private static LimelightRecoveryAuto searching(IO io){
        LimelightRecoveryAuto auto=new LimelightRecoveryAuto(io,new FieldLayout());auto.start(0,.395);
        for(int tick=0;tick<100&&io.out.feederPower==0;tick++)step(io,auto);
        check(io.out.feederPower==1,"Own stable Hive tags enable opening feed");
        io.count=0;step(io,auto);check(auto.getState()==LimelightRecoveryAuto.State.OBSERVE_TILT,"Measured empty indexer moves to observation, not pickup");
        io.pose=auto.destination();step(io,auto);check(auto.getState()==LimelightRecoveryAuto.State.WAIT_TILT,"Observe before recovery");
        io.time+=2;step(io,auto);check(auto.getState()==LimelightRecoveryAuto.State.WAIT_TILT,"A timer cannot invent a Hive tilt");
        io.targets=new Target[]{new Target(38,0,0,36.57),new Target(39,0,0,36.57)};
        for(int tick=0;tick<8;tick++)step(io,auto);
        check(auto.getState()==LimelightRecoveryAuto.State.WAIT_TILT,"Opponent tags cannot confirm own Hive tilt");
        io.targets=down();for(int tick=0;tick<8&&auto.getState()==LimelightRecoveryAuto.State.WAIT_TILT;tick++)step(io,auto);
        check(auto.getState()==LimelightRecoveryAuto.State.CLEAR_FENCE,"Fresh own-tag height confirms opposite Cell up");
        io.pose=auto.destination();step(io,auto);check(auto.getState()==LimelightRecoveryAuto.State.SEARCH_POLLEN,"Switch to front color search after tilt without forcing a turn away from nearby nectar");
        check(io.out.pipeline==1,"State changes must retain color pipeline selection");return auto;
    }
    public static void main(String[] args){
        IO io=new IO();io.valid=false;LimelightRecoveryAuto auto=new LimelightRecoveryAuto(io,new FieldLayout());auto.start(0,.395);
        for(int tick=0;tick<80;tick++){step(io,auto);check(io.out.feederPower==0,"Missing tags never authorize shots");}
        io.stop=true;int reads=io.reads;auto.tick();check(!auto.isRunning()&&io.reads==reads,"STOP cancels before sensor reads");stopped(io,"STOP zeros everything");
        io=new IO();auto=searching(io);
        io.targets=new Target[]{ball(-15,.5,"yellow"),ball(4,2,"red"),ball(20,40,"blue")};
        step(io,auto);step(io,auto);
        check(auto.getState()==LimelightRecoveryAuto.State.APPROACH_POLLEN&&io.out.forward>=.25&&io.out.strafeRight>0,"Approach alliance nectar as well as pollen, ignoring larger opponent nectar");
        check(io.out.pipeline==1&&io.out.intakePower==1,"Keep color camera and intake active on approach");
        io.age=.5;step(io,auto);check(io.out.forward==0&&io.out.strafeRight==0&&io.out.intakePower==0,"Stale color data stops approach immediately");
        io.age=0;io.targets=new Target[]{ball(0,2,"yellow")};step(io,auto);step(io,auto);
        io.connected=false;step(io,auto);stopped(io,"Camera disconnect immediately cancels all actuators");
        io.time=26;auto.tick();check(auto.getState()==LimelightRecoveryAuto.State.PARK,"26-second parking does not depend on camera recovery");
        io.time=30;auto.tick();check(!auto.isRunning(),"30-second hard cutoff");stopped(io,"Hard cutoff zeros actuators");
        io=new IO();auto=searching(io);io.targets=new Target[]{ball(0,2,"yellow")};step(io,auto);step(io,auto);
        io.count=1;step(io,auto);check(auto.getState()==LimelightRecoveryAuto.State.SEARCH_POLLEN,"First measured pickup searches for more balls instead of parking");stopped(io,"Pickup transition cancels all mechanisms");
        step(io,auto);step(io,auto);check(io.out.intakePower==1&&io.out.feederPower==0,"Continue collecting without firing recovered balls");
        io.count=4;step(io,auto);check(auto.getState()==LimelightRecoveryAuto.State.PARK,"Full measured inventory starts parking");stopped(io,"Full inventory cancels collection");
        io=new IO();auto=searching(io);io.targets=new Target[]{ball(0,.5,"yellow")};io.pose=new Pose(-62.5,30,Math.PI);
        step(io,auto);step(io,auto);check(io.out.forward==0&&io.out.intakePower==0,"Blocked observed target cannot drive through fence");
        io.odom=false;step(io,auto);check(!auto.isRunning(),"Invalid odometry safely stops recovery");stopped(io,"Odometry fault cancels actuators");
        System.out.println("Recovery: own-tag tilt confirmation, pollen/alliance-nectar selection, continued pickup/full capacity, stale/lost camera, STOP, collision guard and 26/30-second deadlines passed.");
    }
}
