package org.firstinspires.ftc.teamcode.biobuzz;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Sensor faults and FTC-style clock behavior, independent of a browser or SDK stubs. */
public final class ControllerChecks {
    private static final class IO implements RobotIO {
        double time,readDuration=0,stale=0,height=52.02;int count=4,pipeline=0,id=30;boolean connected=true,odom=true,visible=true;
        Pose pose;Outputs out=new Outputs();boolean requestStop,throwRead;int reads,applies;
        public boolean stopRequested(){return requestStop||Thread.currentThread().isInterrupted();}
        public double nowSeconds(){return time;}
        public Sensors readSensors(){reads++;if(throwRead)throw new IllegalStateException("Transient test I2C read");time+=readDuration;return new Sensors(pose,time-stale,odom,connected,count,
            new Vision(visible,pipeline,time,new Target[]{new Target(id,0,0,height),new Target(id+1,0,0,height)}));}
        public void apply(Outputs o){applies++;out=o.copy();}
    }
    private static void check(boolean condition,String message){if(!condition)throw new AssertionError(message);}
    private static void stopped(IO io,String name){check(io.out.forward==0&&io.out.strafeRight==0&&io.out.turnClockwise==0&&io.out.feederPower==0&&io.out.shooterPower==0&&io.out.intakePower==0,name+" must stop all actuators");}
    private static AutoStateMachine make(IO io){FieldLayout f=new FieldLayout();io.pose=f.hive[0];return new AutoStateMachine(io,f);}
    private static void step(IO io,AutoStateMachine auto){io.time+=.02;auto.tick();}
    /** Script measured counts/poses for decision tests; physical pickup is tested separately in Cannon. */
    private static AutoStateMachine prepareFinalWait(IO io){
        FieldLayout f=new FieldLayout();AutoStateMachine auto=make(io);auto.start(0,.395);
        for(int i=0;i<70;i++)step(io,auto);
        io.count=0;step(io,auto);io.pose=f.garden[0];step(io,auto);
        io.count=4;step(io,auto);io.id=f.farTags[0][0];io.pose=f.farHive[0];
        for(int i=0;i<70;i++)step(io,auto);
        io.count=0;step(io,auto);io.pose=f.flower[0];step(io,auto);
        io.count=4;step(io,auto);io.pose=f.farHive[0];
        for(int i=0;i<70;i++)step(io,auto);
        io.time=23;io.count=0;auto.tick();
        check(auto.getState()==AutoStateMachine.State.WAIT_TILT,"Final batch must await a measured tilt");
        for(int i=0;i<8;i++)step(io,auto);
        check(auto.getState()==AutoStateMachine.State.WAIT_TILT,"No tilt may be inferred from an empty queue or a timer");
        io.id=f.nearTags[0][0];for(int i=0;i<10;i++)step(io,auto);
        check(auto.getState()==AutoStateMachine.State.WAITING,"Confirmed Flower tilt enters stationary waiting");
        return auto;
    }
    public static void main(String[] args){
        for(int side=0;side<2;side++)for(AutoStateMachine.Routine role:new AutoStateMachine.Routine[]{AutoStateMachine.Routine.TEAM_ONE,AutoStateMachine.Routine.TEAM_TWO}){
            FieldLayout layout=new FieldLayout();IO waitingIO=new IO();waitingIO.pose=layout.roleStart(side,role==AutoStateMachine.Routine.TEAM_TWO);waitingIO.count=-1;waitingIO.connected=false;
            AutoStateMachine waitingAuto=new AutoStateMachine(waitingIO,layout,false,role,false,false);
            waitingAuto.parkingToleranceInches=2;waitingAuto.waitUntilParkTime=true;waitingAuto.start(side,.55);
            boolean visitedGarden=false;
            for(int tick=0;tick<1200&&waitingAuto.getState()!=AutoStateMachine.State.WAITING;tick++){
                if(waitingAuto.destination()!=null)waitingIO.pose=waitingAuto.destination();
                step(waitingIO,waitingAuto);
                if(waitingAuto.status().contains("INTAKE · GARDEN"))visitedGarden=true;
                check(waitingAuto.getState()!=AutoStateMachine.State.PARK,"Hardware cannot park before its timed trigger");
            }
            check(waitingAuto.getState()==AutoStateMachine.State.WAITING&&waitingIO.time<26,"Finished role waits until parking time");
            check(visitedGarden==(role==AutoStateMachine.Routine.TEAM_TWO),"Timed wait preserves required Garden pickup");
            stopped(waitingIO,"Entry to stationary wait");
            for(int tick=0;tick<20;tick++){step(waitingIO,waitingAuto);stopped(waitingIO,"All outputs OFF while waiting");}
            waitingIO.time=25.98;waitingAuto.tick();check(waitingAuto.getState()==AutoStateMachine.State.WAITING,"Still waiting just before 26 seconds");stopped(waitingIO,"Pre-park wait");
            waitingIO.time=26;waitingAuto.tick();check(waitingAuto.getState()==AutoStateMachine.State.PARK&&waitingAuto.isRunning(),"26 seconds starts parking, not shutdown");
            check(Math.hypot(waitingIO.out.forward,waitingIO.out.strafeRight)>0,"Parking actually drives at 26 seconds");
            Pose goal=waitingAuto.destination();waitingIO.pose=new Pose(goal.x,goal.y,goal.heading+Math.PI/2);step(waitingIO,waitingAuto);
            check(!waitingAuto.isRunning(),"Timed parking still stops on position without heading correction");stopped(waitingIO,"Timed parking arrival");
            waitingIO.pose=layout.hive[side];step(waitingIO,waitingAuto);stopped(waitingIO,"Timed parking stays latched");
            waitingAuto.start(side,.55);waitingIO.time+=30;waitingAuto.tick();check(!waitingAuto.isRunning(),"Waiting cannot extend the 30-second hard deadline");stopped(waitingIO,"Timed parking deadline");
        }
        // Real-robot parking stops on position, even with heading error/noisy sensors.
        for(int side=0;side<2;side++)for(AutoStateMachine.Routine role:AutoStateMachine.Routine.values()){
            FieldLayout layout=new FieldLayout();IO parkedIO=new IO();parkedIO.pose=layout.hive[side];parkedIO.count=-1;parkedIO.connected=false;
            AutoStateMachine parkedAuto=new AutoStateMachine(parkedIO,layout,false,role,false,false);
            parkedAuto.parkingToleranceInches=2;parkedAuto.start(side,.55);parkedIO.time=26;parkedAuto.tick();
            Pose goal=parkedAuto.destination();check(goal!=null&&parkedAuto.isRunning(),"Distant robot still drives toward parking");
            double inward=side==0?1:-1;
            parkedIO.pose=new Pose(goal.x+inward*2.1,goal.y,goal.heading);step(parkedIO,parkedAuto);
            check(parkedAuto.isRunning(),"Outside parking tolerance must not falsely finish");
            parkedIO.pose=new Pose(goal.x+inward*1.5,goal.y,goal.heading+Math.PI/2);step(parkedIO,parkedAuto);
            check(!parkedAuto.isRunning()&&parkedAuto.getState()==AutoStateMachine.State.PARK,"Position acceptance ignores final heading error");
            check(parkedAuto.status().contains("Parked: all outputs OFF"),"Arrival is visible in telemetry");stopped(parkedIO,"Parking arrival same tick");
            int arrivalReads=parkedIO.reads;
            parkedIO.pose=layout.hive[side];parkedIO.odom=false;parkedIO.throwRead=true;
            for(int repeat=0;repeat<20;repeat++){step(parkedIO,parkedAuto);stopped(parkedIO,"Latched parking despite sensor drift");}
            check(parkedIO.reads==arrivalReads&&!parkedAuto.isRunning(),"Parked controller cannot rearm itself or chase drift");
            parkedAuto.start(side,.55);check(parkedAuto.isRunning()&&!parkedAuto.status().contains("Parked: all outputs OFF"),"Explicit new START clears arrival latch");
            parkedAuto.stop();stopped(parkedIO,"Restart STOP");
        }
        FieldLayout starts=new FieldLayout();
        for(int side=0;side<2;side++){
            check(starts.roleStart(side,false)==starts.hive[side],"Team One matches browser Hive start");
            Pose start=starts.roleStart(side,true),expected=side==0?FieldLayout.fromBrowser(-62.9,-6,Math.PI):FieldLayout.fromBrowser(62.9,6,0);
            check(start.distance(expected)<1e-9&&Math.abs(Pose.angle(start.heading-expected.heading))<1e-9,"Team Two matches browser side-wall start");
            check(starts.allowed(start),"Hardware role start fits field");
        }
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
            check(auto.isRunning(),"Parking movement must remain enabled at 26 seconds");
            io.time=30;auto.tick();check(!auto.isRunning(),"30-second deadline stops run");stopped(io,"Match end");
        }
        io=new IO();auto=make(io);auto.start(0,.4);io.time=.1;auto.tick();io.time=-1;auto.tick();check(!auto.isRunning(),"Backward clock must stop");stopped(io,"Clock fault");
        Outputs mix=new Outputs();mix.forward=.8;mix.strafeRight=.8;mix.turnClockwise=.8;double[] wheels=mix.wheelPowers();check(Math.abs(wheels[0]-1)<1e-9,"Combined mecanum command must normalize all wheels together");for(double w:wheels)check(Math.abs(w)<=1,"Wheel power bounds");
        FieldLayout f=new FieldLayout();Navigator nav=new Navigator(f);check(nav.plan(new Pose(-45,40,Math.PI/2),new Pose(45,-40,Math.PI/2),Math.PI/2).size()>1,"Route must avoid frame supports");
        check(nav.plan(f.hive[0],new Pose(100,100,0),0)==null,"Outside-field route must be rejected");
        for(int alliance=0;alliance<2;alliance++){
            io=new IO();io.pose=f.hive[alliance];auto=new AutoStateMachine(io,f);auto.start(alliance,.4);
            io.time=25.98;auto.tick();check(auto.getState()!=AutoStateMachine.State.PARK,"Do not park early");
            io.time=26;auto.tick();check(auto.getState()==AutoStateMachine.State.PARK&&auto.isRunning(),"26 seconds starts parking, not shutdown");
            check(Math.hypot(io.out.forward,io.out.strafeRight)>0,"Parking must command driving at 26 seconds");
            check(io.out.shooterPower==0&&io.out.feederPower==0&&io.out.intakePower==0,"Parking cancels scoring immediately");
            check(auto.destination().heading==io.pose.heading,"Parking preserves current heading");
            for(int angle=0;angle<360;angle+=5)check(f.allowed(f.parkingTarget(new Pose(0,0,Math.toRadians(angle)),alliance)),"Parking target must fit at every heading");
            io.time=30;auto.tick();check(!auto.isRunning(),"Stalled parking must stop at 30 seconds");stopped(io,"Stalled parking deadline");
        }
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
        io=new IO();auto=prepareFinalWait(io);
        for(int i=0;i<50;i++){step(io,auto);check(auto.getState()==AutoStateMachine.State.WAITING,"Do not restart pickup after final tilt");stopped(io,"Waiting after Flower tilt");}
        io.time=25.98;auto.tick();stopped(io,"Wait until parking start");
        io.time=26;auto.tick();check(auto.getState()==AutoStateMachine.State.PARK,"Completed routine begins parking at 26");check(io.out.intakePower==0,"Parking leaves intake off");
        io.time=30;auto.tick();stopped(io,"Completed routine deadline");
        // Camera-free mode accepts absent vision and unknown inventory without inventing counts.
        for(int alliance=0;alliance<2;alliance++){
            io=new IO();io.connected=false;io.visible=false;io.count=-1;io.pose=f.hive[alliance];
            auto=new AutoStateMachine(io,f,false);auto.start(alliance,.4);
            for(int i=0;i<70;i++)step(io,auto);
            check(auto.isRunning()&&io.out.feederPower==1,"Camera-free fixed-pose shot needs no Limelight");
            check(io.count==-1,"Timed attempts must not manufacture inventory");
            io.time=26;auto.tick();check(auto.getState()==AutoStateMachine.State.PARK,"Camera-free park starts at 26 seconds");
            check(io.out.feederPower==0&&io.out.intakePower==0,"Camera-free parking cancels mechanisms");
            io.time=30;auto.tick();check(!auto.isRunning(),"Camera-free hard deadline");stopped(io,"Camera-free deadline");
            auto.start(alliance,.4);io.odom=false;auto.tick();check(!auto.isRunning(),"Camera-free still requires valid odometry");stopped(io,"Camera-free odometry fault");
        }
        // Missing tags plus partial pickup must not strand the route at Garden.
        for(int alliance=0;alliance<2;alliance++){
            io=new IO();io.connected=false;io.visible=false;io.count=-1;io.pose=f.hive[alliance];
            auto=new AutoStateMachine(io,f,false);auto.start(alliance,.4);
            for(int i=0;i<170;i++)step(io,auto);
            check(auto.getState()==AutoStateMachine.State.MOVING&&auto.destination()==f.garden[alliance],"Preloads lead to Garden without tags");
            Pose garden=f.garden[alliance];io.pose=new Pose(garden.x,garden.y+(alliance==0?2:-2),garden.heading);step(io,auto);
            check(auto.getState()==AutoStateMachine.State.INTAKE,"Garden approach accepts a two-inch arrival error");
            io.count=2;io.time+=3.1;auto.tick();
            check(auto.getState()==AutoStateMachine.State.MOVING&&auto.destination()==f.farHive[alliance],"Partial Garden pickup continues to Hive");
            check(io.count==2,"Partial Garden count is not fabricated");
            io.pose=f.farHive[alliance];for(int i=0;i<70;i++)step(io,auto);
            check(io.out.feederPower==1,"Partial Garden batch shoots without tags");
            io.count=0;step(io,auto);check(io.out.feederPower==1,"Empty count must not shorten the timed Garden shot");
            io.time+=2.1;auto.tick();check(auto.destination()==f.flower[alliance],"Timed Garden shot leads to Flower");
            io.pose=f.flower[alliance];step(io,auto);io.count=1;io.time+=3.1;auto.tick();
            check(auto.destination()==f.farHive[alliance],"Partial Flower pickup returns to Hive");
            io.pose=f.farHive[alliance];for(int i=0;i<70;i++)step(io,auto);
            check(io.out.feederPower==1,"Flower batch shoots without tags");
            io.count=0;step(io,auto);check(io.out.feederPower==1,"Empty count must not shorten the timed Flower shot");
            io.time+=2.1;auto.tick();check(auto.getState()==AutoStateMachine.State.WAITING,"Only completed three-batch route waits for park");
            io.time=26;auto.tick();check(auto.getState()==AutoStateMachine.State.PARK,"Partial batches preserve park trigger");
        }
        // Slower measured progress must not trip a simulator-speed deadline.
        io=new IO();io.connected=false;io.visible=false;io.count=-1;io.pose=f.hive[0];auto=new AutoStateMachine(io,f,false);auto.start(0,.4);
        for(int i=0;i<170;i++)step(io,auto);
        for(int i=0;i<140;i++){io.pose=new Pose(io.pose.x,io.pose.y-.1,io.pose.heading);io.time+=.1;auto.tick();}
        check(auto.getState()==AutoStateMachine.State.MOVING,"Slow but progressing Garden approach continues");
        io.time+=3.1;auto.tick();check(auto.status().contains("Movement stalled toward Garden"),"A genuinely stalled approach still faults");stopped(io,"Garden motion stall");
        auto.stop();stopped(io,"Manual stop after slow approach");
        io=new IO();auto=make(io);auto.start(0,.4);io.requestStop=true;int reads=io.reads;
        auto.tick();check(!auto.isRunning()&&io.reads==reads,"STOP must exit before another hardware read");stopped(io,"Requested STOP");
        int applies=io.applies;auto.stop();auto.stop();check(io.applies==applies,"Repeated stop must not repeat hardware writes");
        io=new IO();auto=make(io);auto.start(0,.4);Thread.currentThread().interrupt();
        try{auto.tick();check(!auto.isRunning(),"SDK thread interruption must stop controller");stopped(io,"Interrupted controller");}
        finally{Thread.interrupted();}
        final int[] polls={0};Navigator cancelNav=new Navigator(f,()->++polls[0]>40);
        try{cancelNav.plan(new Pose(-45,40,Math.PI/2),new Pose(45,-40,Math.PI/2),Math.PI/2);throw new AssertionError("Route search must check STOP inside its work");}
        catch(java.util.concurrent.CancellationException expected){check(polls[0]<50,"Navigation cancellation must be prompt");}
        check(!f.allowed(new Pose(Double.NaN,0,0)),"NaN geometry cannot be navigable");
        check(!f.clear(new Pose(1e30,0,0),f.hive[0],0),"Corrupt coordinates must not create an enormous sampling loop");
        check(nav.plan(new Pose(1e30,0,0),f.hive[0],0)==null,"Corrupt route start is rejected");
        for(double corrupt:new double[]{Double.NaN,Double.POSITIVE_INFINITY,1e30}){
            check(Pose.validatedOdometry(0,0,corrupt)==null,"Corrupt raw heading is rejected before normalization");
            check(Pose.validatedOdometry(corrupt,0,0)==null,"Corrupt raw X is rejected");
        }
        Pose wrapped=Pose.validatedOdometry(1,2,10000*Math.PI);
        check(wrapped!=null&&Math.abs(wrapped.heading)<=Math.PI,"Large valid heading wraps in constant time");
        // Model a slow Hub: each clock poll consumes 10 ms, forcing many slices.
        final long[] slowClock={0};final boolean[] cancelSlow={false};
        Navigator sliced=new Navigator(f,()->cancelSlow[0],()->{slowClock[0]+=10000000L;return slowClock[0];});
        Pose slowStart=new Pose(-45,40,Math.PI/2),slowGoal=new Pose(45,-40,Math.PI/2);Outputs planned=null;int pending=0;
        for(int i=0;i<10000;i++){
            planned=sliced.move(slowStart,slowGoal);
            if(!sliced.isPlanning())break;
            pending++;check(planned!=null&&planned.forward==0&&planned.strafeRight==0&&planned.turnClockwise==0,"Pending planning must hold zero drive");
        }
        check(pending>25&&slowClock[0]>250000000L,"Slow planning must span well beyond the old cutoff");
        check(!sliced.isPlanning()&&planned!=null&&Math.hypot(planned.forward,planned.strafeRight)>0,"Slow planning must eventually produce a usable route, not WAITING");
        sliced.reset();sliced.move(slowStart,slowGoal);cancelSlow[0]=true;
        try{sliced.move(slowStart,slowGoal);throw new AssertionError("STOP must cancel pending planning");}
        catch(java.util.concurrent.CancellationException expected){ }
        // Fixed camera-free navigation must never even consult a planner clock.
        for(int team=0;team<2;team++){
            final boolean[] stopFixed={false};Navigator fixed=new Navigator(f,()->stopFixed[0],()->{throw new AssertionError("Fixed routine must not enter route search");});
            fixed.useScriptedRoute(true,team);
            Outputs firstMove=fixed.move(f.hive[team],f.garden[team]);
            check(firstMove!=null&&!fixed.isPlanning(),"Fixed Garden route starts immediately without planning");
            Pose previous=f.hive[team];
            if(!f.canRotate(previous,f.garden[team].heading)){
                // Rear-facing start must first leave the fence before turning to intake.
                for(Pose waypoint:fixed.route()){check(f.clear(previous,waypoint,previous.heading),"Fence exit must be collision-checked");previous=waypoint;}
                check(f.canRotate(previous,f.garden[team].heading),"Fence exit provides turn clearance");
                fixed.move(previous,f.garden[team]);
                previous=new Pose(previous.x,previous.y,f.garden[team].heading);
                fixed.move(previous,f.garden[team]);
            }
            for(Pose waypoint:fixed.route()){check(f.clear(previous,waypoint,f.garden[team].heading),"Every fixed Garden corridor segment must be collision-checked");previous=waypoint;}
            check(previous==f.garden[team],"Fixed corridor ends at Garden");
            fixed.reset();check(fixed.move(f.garden[team],f.farHive[team])!=null,"Fixed Garden return has a safe turn approach");
            stopFixed[0]=true;try{fixed.move(f.garden[team],f.farHive[team]);throw new AssertionError("Fixed route must still honor STOP");}
            catch(java.util.concurrent.CancellationException expected){ }
        }
        // All three cycles still run with a permanently empty, measured indexer.
        for(int team=0;team<2;team++){
            io=new IO();io.count=0;io.connected=false;io.visible=false;io.pose=f.hive[team];auto=new AutoStateMachine(io,f,false);auto.start(team,.4);
            for(int batch=0;batch<3;batch++){
                io.pose=batch==0?f.hive[team]:f.farHive[team];
                for(int i=0;i<70;i++)step(io,auto);
                check(auto.getState()==AutoStateMachine.State.SHOOTING&&io.out.shooterPower>0&&io.out.feederPower==1,"Empty batch "+batch+" must spin and feed");
                check(io.count==0,"Empty shot attempt must preserve measured zero");
                for(int i=0;i<100;i++)step(io,auto);
                if(batch<2){
                    check(auto.getState()==AutoStateMachine.State.MOVING,"Timed empty shot advances to pickup");
                    io.pose=batch==0?f.garden[team]:f.flower[team];step(io,auto);io.time+=3.1;auto.tick();
                    check(auto.destination()==f.farHive[team],"Empty pickup still returns to shooting pose");
                }
            }
            check(auto.getState()==AutoStateMachine.State.WAITING||auto.getState()==AutoStateMachine.State.MOVING,
                "Three empty shot cycles finish routine or start a bounded leftover search");
            check(io.out.feederPower==0&&io.out.shooterPower==0,"Optional pickup never adds another shot");
            io.time=25;auto.tick();check(auto.getState()==AutoStateMachine.State.WAITING,"Optional pickup ends by 25 seconds");
            stopped(io,"Extra pickup cutoff");
            io.time=26;auto.tick();check(auto.getState()==AutoStateMachine.State.PARK,"Empty routine preserves park trigger");
            io.time=30;auto.tick();stopped(io,"Empty routine hard deadline");
        }
        io=new IO();io.count=4;io.connected=false;io.visible=false;io.pose=f.hive[0];auto=new AutoStateMachine(io,f,false);auto.start(0,.4);
        for(int i=0;i<170;i++)step(io,auto);
        check(auto.destination()==f.garden[0],"An unchanged full count cannot extend the fixed feed cycle indefinitely");
        for(int team=0;team<2;team++)for(int scenario=0;scenario<4;scenario++){
            io=new IO();io.pose=f.farHive[team];io.count=0;io.connected=false;
            auto=new AutoStateMachine(io,f,false,AutoStateMachine.Routine.TEAM_TWO,false,scenario!=3);auto.start(team,.4);
            for(int i=0;i<1000&&auto.getState()!=AutoStateMachine.State.PRE_PARK;i++){
                if(auto.destination()!=null)io.pose=auto.destination();
                step(io,auto);
            }
            check(auto.getState()==AutoStateMachine.State.PRE_PARK,"Check optional pickup after main routine");
            if(scenario==0)io.time=23; // Too late to add approach + pickup + parking.
            if(scenario==1)io.count=4; // No inventory space.
            step(io,auto);
            if(scenario<2||scenario==3)check(auto.getState()==AutoStateMachine.State.PARK,"Late/full/disabled robot skips optional pickup");
            else{
                check(auto.getState()==AutoStateMachine.State.MOVING&&auto.status().contains("Extra Flower"),"Early empty robot attempts nearby pollen");
                io.time=25;auto.tick();check(auto.getState()==AutoStateMachine.State.PARK,"Extra pickup yields to parking at its cutoff");
                check(io.out.intakePower==0&&io.out.feederPower==0&&io.out.shooterPower==0,"Parking cancels optional mechanisms");
            }
            io.time=30;auto.tick();check(!auto.isRunning(),"Optional pickup cannot extend autonomous");stopped(io,"Optional pickup deadline");
        }
        for(int side=0;side<2;side++)for(AutoStateMachine.Routine role:new AutoStateMachine.Routine[]{AutoStateMachine.Routine.TEAM_ONE,AutoStateMachine.Routine.TEAM_TWO}){
            io=new IO();io.pose=f.roleStart(side,role==AutoStateMachine.Routine.TEAM_TWO);io.count=-1;io.connected=false;
            auto=new AutoStateMachine(io,f,false,role,false,false);auto.start(side,.4);
            boolean gardenVisited=false,parked=false;
            for(int i=0;i<1000&&auto.isRunning();i++){
                if(auto.destination()!=null)io.pose=auto.destination();
                AutoStateMachine.State beforeTick=auto.getState();
                step(io,auto);
                if(role==AutoStateMachine.Routine.TEAM_ONE&&beforeTick==AutoStateMachine.State.SHOOTING&&auto.getState()!=beforeTick){
                    check(auto.getState()==AutoStateMachine.State.PARK,"Team One goes directly from shooting to parking");
                    check(auto.destination().distance(f.parkingTarget(io.pose,side))<1e-9,"Team One targets park, not an intermediate Hive waypoint");
                    stopped(io,"Shot-to-park transition cancels mechanisms");
                }
                if(auto.status().contains("INTAKE · GARDEN"))gardenVisited=true;
                check(!auto.status().contains("Extra Flower")&&!auto.status().contains("Extra pollen sweep"),"Hardware never attempts final Flower pickup");
                if(auto.getState()==AutoStateMachine.State.PARK){parked=true;check(io.out.intakePower==0,"Parking disables intake");break;}
            }
            check(parked&&io.time<26,"Hardware parks when role completes, without optional collection");
            check(gardenVisited==(role==AutoStateMachine.Routine.TEAM_TWO),"Disabling final collection preserves Team Two's required Garden pickup");
            auto.stop();stopped(io,"Hardware role STOP");
        }
        for(AutoStateMachine.Routine mode:AutoStateMachine.Routine.values()){
            io=new IO();io.pose=f.hive[0];io.count=4;io.connected=false;
            auto=new AutoStateMachine(io,f,false,mode,true);auto.start(0,1);
            for(int i=0;i<100;i++){step(io,auto);check(io.out.shooterPower==0&&io.out.feederPower==0&&io.out.intakePower==0,"Dry-run shot dwell must never power a mechanism");}
            check(auto.status().contains("MOVEMENT ONLY"),"Dry run must be visible in telemetry");
            AutoStateMachine.State heldState=auto.getState();io.odom=false;step(io,auto);
            check(auto.isRunning()&&auto.getState()==AutoStateMachine.State.SAFETY_HOLD,"Bad odometry holds motors OFF without ending dry run");stopped(io,"Dry-run safety hold");
            io.time+=.5;step(io,auto);io.odom=true;step(io,auto);
            check(auto.isRunning()&&auto.getState()==heldState,"Recovered sensor resumes the interrupted pretend-shoot state");
            io.throwRead=true;step(io,auto);check(auto.isRunning()&&auto.getState()==AutoStateMachine.State.SAFETY_HOLD,"Transient read exception must not exit dry run");stopped(io,"Read exception hold");
            io.throwRead=false;step(io,auto);check(auto.isRunning()&&auto.getState()==heldState,"Read recovery resumes without skipping dwell");
            io.requestStop=true;step(io,auto);check(!auto.isRunning(),"Dry run honors immediate STOP");stopped(io,"Dry-run STOP");
            io.requestStop=false;auto.start(0,1);io.time+=30;auto.tick();check(!auto.isRunning(),"Dry run hard deadline");stopped(io,"Dry-run deadline");
            auto.start(0,1);io.odom=false;step(io,auto);check(auto.isRunning(),"Dry run stays alive with invalid odometry but no drive");stopped(io,"Dry-run odometry fault");
            io.time+=26;auto.tick();check(auto.isRunning()&&auto.getState()==AutoStateMachine.State.PARK,"Parking state starts at deadline even during sensor hold");stopped(io,"Invalid-odometry parking hold");
            io.time+=4;auto.tick();check(!auto.isRunning(),"An unrecovered sensor still cannot extend autonomous beyond 30 seconds");stopped(io,"Sensor hold deadline");
        }
        System.out.println("Shared Java full/partner routines, empty shot cycles, fixed routes, STOP and odometry checks passed.");
    }
}
