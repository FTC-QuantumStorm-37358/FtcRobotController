package org.firstinspires.ftc.teamcode.biobuzz;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Sensor faults and FTC-style clock behavior, independent of a browser or SDK stubs. */
public final class ControllerChecks {
    private static final class IO implements RobotIO {
        double time,readDuration=0,stale=0,height=52.02;int count=4,pipeline=0,id=30;boolean connected=true,odom=true,visible=true;
        Pose pose;Outputs out=new Outputs();
        public double nowSeconds(){return time;}
        public Sensors readSensors(){time+=readDuration;return new Sensors(pose,time-stale,odom,connected,count,
            new Vision(visible,pipeline,time,new Target[]{new Target(id,0,0,height),new Target(id+1,0,0,height)}));}
        public void apply(Outputs o){out=o.copy();}
    }
    private static void check(boolean condition,String message){if(!condition)throw new AssertionError(message);}
    private static void stopped(IO io,String name){check(io.out.forward==0&&io.out.strafeRight==0&&io.out.turnClockwise==0&&io.out.feederPower==0&&io.out.shooterPower==0&&io.out.intakePower==0,name+" must stop all actuators");}
    private static AutoStateMachine make(IO io){FieldLayout f=new FieldLayout();io.pose=f.hive[0];return new AutoStateMachine(io,f);}
    public static void main(String[] args){
        IO io=new IO();io.readDuration=.001;AutoStateMachine auto=make(io);auto.start(0,.4);
        for(int i=0;i<70;i++){io.time+=.02;auto.tick();}
        check(auto.isRunning(),"Reading a real sensor advances wall time; fresh data must not be rejected as future data");
        check(io.out.feederPower==1,"Centered fresh tags plus spinup must enable feed");
        check(io.count==4,"Controller must not overwrite measured ball counts");
        io.time=5;auto.tick();check(auto.status().contains("Balls stuck"),"Full measured count after feeding must report a jam");stopped(io,"Jam");
        auto.stop();stopped(io,"OpMode stop");
        io=new IO();auto=make(io);auto.start(0,.4);io.stale=1;auto.tick();check(!auto.isRunning(),"Stale odometry must stop");stopped(io,"Stale sensor");
        io=new IO();auto=make(io);auto.start(0,.4);io.count=-1;auto.tick();check(!auto.isRunning(),"Unknown ball count must stop");stopped(io,"Unknown inventory");
        io=new IO();auto=make(io);auto.start(0,.4);io.connected=false;auto.tick();check(!auto.isRunning(),"Disconnected camera must stop");stopped(io,"Camera disconnect");
        for(int bad:new int[]{1,2}){
            io=new IO();auto=make(io);auto.start(0,.4);if(bad==1)io.pipeline=1;else io.id=38;
            for(int i=0;i<120;i++){io.time+=.02;auto.tick();check(io.out.feederPower==0,"Wrong pipeline or alliance must never authorize a shot");}
            io.time=26;auto.tick();check(auto.getState()==AutoStateMachine.State.PARK,"26-second guard must enter park");check(io.out.feederPower==0&&io.out.shooterPower==0,"Park stops shooter");
            check(!auto.isRunning(),"26-second deadline stops run");stopped(io,"Parking deadline");
            io.time=30;auto.tick();check(!auto.isRunning(),"Cannot resume after parking deadline");stopped(io,"Match end");
        }
        io=new IO();auto=make(io);auto.start(0,.4);io.time=.1;auto.tick();io.time=-1;auto.tick();check(!auto.isRunning(),"Backward clock must stop");stopped(io,"Clock fault");
        Outputs mix=new Outputs();mix.forward=.8;mix.strafeRight=.8;mix.turnClockwise=.8;double[] wheels=mix.wheelPowers();check(Math.abs(wheels[0]-1)<1e-9,"Combined mecanum command must normalize all wheels together");for(double w:wheels)check(Math.abs(w)<=1,"Wheel power bounds");
        FieldLayout f=new FieldLayout();Navigator nav=new Navigator(f);check(nav.plan(new Pose(-45,40,Math.PI/2),new Pose(45,-40,Math.PI/2),Math.PI/2).size()>1,"Route must avoid frame supports");
        check(nav.plan(f.hive[0],new Pose(100,100,0),0)==null,"Outside-field route must be rejected");
        for(int alliance=0;alliance<2;alliance++){
            io=new IO();io.id=f.nearTags[alliance][0];io.pose=f.hive[alliance];
            auto=new AutoStateMachine(io,f);auto.start(alliance,.4);
            for(int i=0;i<70;i++){io.time+=.02;auto.tick();}
            check(io.out.feederPower==1,"Initial confirmed own Cell feeds");
            io.count=2;io.height=f.upperTagHeight-.5;io.time+=.04;auto.tick();
            check(io.out.feederPower==0,"Height motion pauses feeding immediately");
            io.id=f.farTags[alliance][0];io.height=f.upperTagHeight;
            for(int i=0;i<4;i++){io.time+=.04;auto.tick();}
            check(auto.getState()==AutoStateMachine.State.MOVING&&auto.destination()==f.farHive[alliance],"Confirmed tip repositions to far Cell");
            check(io.out.feederPower==0,"Reposition stops feeder");
            io.pose=f.farHive[alliance];
            for(int i=0;i<70;i++){io.time+=.02;auto.tick();}
            check(io.out.feederPower==1,"Partial measured load resumes at confirmed far Cell");
            io.id=f.nearTags[alliance][0];
            for(int i=0;i<4;i++){io.time+=.04;auto.tick();}
            check(auto.getState()==AutoStateMachine.State.MOVING&&auto.destination()==f.hive[alliance],"Second tip returns to normal Cell");
            check(io.out.feederPower==0,"Second reposition stops feeder");
        }
        System.out.println("Shared Java clock, sensor freshness, pipeline/alliance gates, jam, duration and collision routing passed.");
    }
}
