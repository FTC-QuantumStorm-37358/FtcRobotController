package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** QuantumStorm's autonomous sequence, independent of FTC SDK, browser, or wall-clock timers. */
public final class AutoStateMachine {
    public static final double PARK_START_SECONDS=26.0;
    public static final double AUTO_END_SECONDS=30.0;
    public enum State { INIT, MOVING, SHOOTING, INTAKE, ERROR, WAIT_TILT, WAITING, PARK, STOPPED }
    public enum Routine { FULL, SINGLE_TILT }
    private enum ShootStep { CHECK_BALLS, AIM, FIRE }
    private enum Source { GARDEN, FLOWER }
    private final RobotIO io;
    private final FieldLayout field;
    private final Navigator navigator;
    private final boolean useVision;
    private final Routine routine;
    /** Timed attempts when inventory is unavailable; never manufacture a ball count. */
    public double timedFeedSeconds=2.0,timedPickupSeconds=3.0;
    private HiveVision hiveVision;
    private State state=State.STOPPED, afterMove;
    private ShootStep shootStep;
    private Source source;
    private Pose destination;
    private Pose currentPose;
    private String destinationName="",error="";
    private boolean running,gardenDone,flowerDone,flipped,recoverable;
    private boolean flowerTiltConfirmed;
    private int finalShotHiveState;
    private int alliance,retries;
    private double startAt,stateAt,lastNow,power;
    private double shooterAt,moveDeadline;
    private Pose progressPose;
    private double progressAt;
    private Outputs last=new Outputs();
    public AutoStateMachine(RobotIO io,FieldLayout field) { this(io,field,true); }
    public AutoStateMachine(RobotIO io,FieldLayout field,boolean useVision) {
        this(io,field,useVision,Routine.FULL);
    }
    public AutoStateMachine(RobotIO io,FieldLayout field,boolean useVision,Routine routine) {
        if(routine==null)throw new IllegalArgumentException("Routine is required");
        this.routine=routine;
        this.io=io;this.field=field;this.useVision=useVision;this.navigator=new Navigator(field,io::stopRequested);
    }
    public void start(int alliance,double power) {
        if(alliance<0||alliance>1||!Double.isFinite(power)||power<0||power>1)throw new IllegalArgumentException("Invalid alliance or shooter power");
        this.alliance=alliance;this.power=power;startAt=lastNow=io.nowSeconds();running=true;retries=0;
        hiveVision=useVision?new HiveVision(field,alliance):null;
        navigator.useScriptedRoute(!useVision,alliance);
        gardenDone=flowerDone=flipped=flowerTiltConfirmed=false;finalShotHiveState=-1;error="";destination=null;enter(State.INIT);io.apply(new Outputs());
        shooterAt=Double.NaN;
    }
    public void stop() { boolean alreadyStopped=!running&&state==State.STOPPED;running=false;state=State.STOPPED;last=new Outputs();if(!alreadyStopped)io.apply(last); }
    public boolean isRunning() { return running; }
    public State getState() { return state; }
    public Outputs getOutputs() { return last.copy(); }
    public Pose destination() { return running&&(state==State.MOVING||state==State.PARK)?destination:null; }
    public java.util.List<Pose> route() { return running&&(state==State.MOVING||state==State.PARK)?navigator.route():new java.util.ArrayList<Pose>(); }
    public double elapsed() { return Math.max(0,io.nowSeconds()-startAt); }
    public String status() {
        return state+(state==State.MOVING?" → "+destinationName:state==State.SHOOTING?" · "+shootStep:state==State.INTAKE?" · "+source:"")
            +String.format(java.util.Locale.ROOT," · %.1f / 30 s",elapsed())
            +(routine==Routine.SINGLE_TILT?" · Single tilt: park after opening shot cycle":" · park starts at 26 s")
            +(!useVision?" · Camera-free: fixed route, no tag detection, Hive tilt unverified":"")
            +(navigator.isPlanning()?" · Planning route (STOP available)":"")
            +(flowerTiltConfirmed?" · Flower tilt confirmed":"")+(error.isEmpty()?"":" · "+error);
    }
    private void enter(State state) { this.state=state;stateAt=io.nowSeconds();navigator.reset();if(state==State.SHOOTING)shootStep=ShootStep.CHECK_BALLS; }
    private void moveThen(Pose target,String name,State after) {
        destination=target;destinationName=name;afterMove=after;enter(State.MOVING);
        Pose current=currentPose;
        progressPose=current;progressAt=io.nowSeconds();
        // Never run a synchronous search just to estimate a deadline.
        double distance=current.distance(target)*1.6;
        moveDeadline=Math.max(3,distance/Math.max(1,25*field.drivePower)+5);
    }
    private Pose hive() { return flipped?field.farHive[alliance]:field.hive[alliance]; }
    private void pickup() {
        if(routine==Routine.SINGLE_TILT){beginParking();return;}
        if(!gardenDone){source=Source.GARDEN;moveThen(field.garden[alliance],"Garden",State.INTAKE);}
        else if(!flowerDone){source=Source.FLOWER;moveThen(field.flower[alliance],"Flower",State.INTAKE);}
        else{finalShotHiveState=flipped?1:0;enter(useVision?State.WAIT_TILT:State.WAITING);}
    }
    private void beginParking(){destination=field.parkingTarget(currentPose,alliance);enter(State.PARK);}
    private void finishPickup() {
        if(source==Source.GARDEN)gardenDone=true;if(source==Source.FLOWER)flowerDone=true;
        if(!useVision){
            // Scripted far-side return, matching the default simulation sequence.
            // This is an assumption, not a detected/confirmed Hive tip.
            flipped=true;moveThen(field.farHive[alliance],"Hive scripted far face",State.SHOOTING);return;
        }
        // Move to the closer observation side; vision, never proximity, decides
        // whether that Cell is up and authorizes feeding after arrival.
        Pose observation=currentPose.distance(field.hive[alliance])<=currentPose.distance(field.farHive[alliance])
            ?field.hive[alliance]:field.farHive[alliance];
        moveThen(observation,"Hive",State.SHOOTING);
    }
    private void fail(String reason,boolean recoverable) { error=reason;this.recoverable=recoverable;enter(State.ERROR); }
    private Target tag(Vision vision,double now) {
        return hiveVision.aim(now,flipped);
    }
    public void tick() {
        if(io.stopRequested()){stop();return;}
        Outputs out=new Outputs();double now=io.nowSeconds();
        if(!running){last=out;io.apply(out);return;}
        Sensors sensors=io.readSensors();now=io.nowSeconds();
        if(io.stopRequested()){stop();return;}
        if(!Double.isFinite(now)||now<lastNow){fail("Clock moved backward",false);running=false;last=out;io.apply(out);return;}lastNow=now;
        if(elapsed()>=AUTO_END_SECONDS){running=false;error="30-second autonomous limit; all actuators stopped";last=out;io.apply(out);return;}
        if(!sensors.odometryValid||!sensors.pose.finite()||now-sensors.sampledAt<0||now-sensors.sampledAt>.25){
            fail("Odometry unavailable or stale",false);running=false;last=out;io.apply(out);return;
        }
        if(sensors.ballCount<-1||sensors.ballCount>4||(useVision&&(sensors.ballCount<0||!sensors.cameraConnected))){
            fail(sensors.ballCount<0||sensors.ballCount>4?"Ball-count sensor unavailable":"Camera disconnected",false);running=false;last=out;io.apply(out);return;
        }
        if(useVision)hiveVision.update(sensors.vision,now);
        currentPose=sensors.pose;
        // The final four seconds belong exclusively to parking, regardless of
        // which scoring or pickup state was active before the transition.
        if(state!=State.PARK&&elapsed()>=PARK_START_SECONDS){
            beginParking();
        }
        double age=now-stateAt;
        State before=state;
        try {
            switch(state) {
                case INIT:moveThen(hive(),"Hive",State.SHOOTING);break;
                case MOVING:
                    // Pickup needs a usable approach, not sub-inch shooting
                    // accuracy. Stop before intaking; never loosen shot alignment.
                    if(!useVision&&afterMove==State.INTAKE&&source==Source.GARDEN&&sensors.pose.distance(destination)<=3
                            &&Math.abs(Pose.angle(destination.heading-sensors.pose.heading))<=Math.toRadians(8)){
                        enter(State.INTAKE);break;
                    }
                    Outputs drive=navigator.move(sensors.pose,destination);
                    if(drive==null)enter(afterMove);else out=drive;
                    if(state==State.MOVING){
                        if(navigator.isPlanning()){
                            progressPose=sensors.pose;progressAt=now;stateAt=now;
                        }else if(!useVision){
                            // Real wheel speeds need not match the simulator's
                            // 25 in/s. Keep navigating while odometry shows progress,
                            // but stop a genuinely stalled robot after three seconds.
                            if(sensors.pose.distance(progressPose)>.25||Math.abs(Pose.angle(sensors.pose.heading-progressPose.heading))>Math.toRadians(3)){
                                progressPose=sensors.pose;progressAt=now;
                            }
                            if(now-progressAt>3)fail("Movement stalled toward "+destinationName,false);
                        }else if(age>moveDeadline)fail("Destination timeout: "+destinationName,false);
                    }
                    break;
                case SHOOTING:
                    if(!useVision){
                        // Fixed routine: always execute this bounded shot attempt,
                        // even with zero/unknown balls. Never infer shots or inventory.
                        out.shooterPower=power;
                        if(Double.isNaN(shooterAt))shooterAt=now;
                        if(shootStep==ShootStep.CHECK_BALLS){shootStep=ShootStep.AIM;stateAt=now;age=0;}
                        if(shootStep==ShootStep.AIM&&now-shooterAt>=1){shootStep=ShootStep.FIRE;stateAt=now;age=0;}
                        if(shootStep==ShootStep.FIRE){
                            if(age>=timedFeedSeconds){shooterAt=Double.NaN;pickup();break;}
                            out.feederPower=1;
                        }
                        break;
                    }
                    int observed=hiveVision.state(now);
                    if(observed>=0){
                        flipped=observed==1;
                        if(destination!=hive()){moveThen(hive(),flipped?"Hive far face":"Hive normal face",State.SHOOTING);break;}
                    }
                    if(shootStep==ShootStep.CHECK_BALLS){
                        if(sensors.ballCount==0){pickup();break;}shootStep=ShootStep.AIM;stateAt=now;age=0;
                    }
                    out.shooterPower=power;
                    if(Double.isNaN(shooterAt))shooterAt=now;
                    if(shootStep==ShootStep.AIM){
                        Target t=tag(sensors.vision,now);
                        if(t!=null&&Math.abs(t.tx)<2&&now-shooterAt>=1){shootStep=ShootStep.FIRE;stateAt=now;age=0;}
                        else if(age>3){fail(t==null?"AprilTag not found":"AprilTag alignment timeout",true);break;}
                        else if(t!=null)out.turnClockwise=FieldLayout.clip(t.tx*.02,.6);
                    }
                    if(shootStep==ShootStep.FIRE){
                        if(sensors.ballCount==0){out.shooterPower=0;shooterAt=Double.NaN;pickup();break;}
                        Target fireTarget=tag(sensors.vision,now);
                        if(age>3){fail(fireTarget==null?"Hive state or AprilTag lost while feeding":"Balls stuck in indexer",true);break;}
                        if(fireTarget!=null&&Math.abs(fireTarget.tx)<2)out.feederPower=1;
                    }
                    break;
                case INTAKE:
                    if(sensors.ballCount>=4){finishPickup();break;}
                    // Garden intake already runs on approach; reserve the longer
                    // dwell for the Flower's release and sweep.
                    if(!useVision&&sensors.ballCount<0&&age>=(source==Source.GARDEN?Math.min(1,timedPickupSeconds):timedPickupSeconds)){finishPickup();break;}
                    if(!useVision&&age>=timedPickupSeconds){
                        // Preserve the measured count, including zero. A partial
                        // pickup does not cancel the remaining Garden/Flower route.
                        error="Pickup attempt ended with "+sensors.ballCount+" measured balls; continuing routine";
                        finishPickup();break;
                    }
                    if(age>4){fail("Pickup did not fill four slots",false);break;}
                    out.pipeline=1;out.intakePower=1;
                    // Touch at a corner, then strafe clear of the outlet. Allow
                    // staged releases to clear before sweeping them through the
                    // front intake; only the measured count completes pickup.
                    if(source==Source.FLOWER&&age<.12)out.forward=.3;
                    else if(source==Source.FLOWER&&age<.24)out.strafeRight=-.4;
                    else if(source==Source.FLOWER&&age<1.24)out.strafeRight=.2;
                    else if(source==Source.FLOWER&&age<2.24)out.forward=0;
                    else if(source==Source.FLOWER)out.forward=.65;
                    else if(!useVision){
                        // Finish the short, collision-checked Garden approach
                        // while intaking, then leave on the bounded pickup timer.
                        Outputs approach=navigator.move(sensors.pose,destination);
                        if(approach!=null){out.forward=approach.forward;out.strafeRight=approach.strafeRight;out.turnClockwise=approach.turnClockwise;}
                    }
                    else if(sensors.vision.fresh(now,1)&&sensors.vision.targets.length>0){
                        Target t=sensors.vision.targets[0];out.forward=Math.abs(t.tx)<20?.3:0;out.turnClockwise=FieldLayout.clip(t.tx*.02,.3);
                    }else out.turnClockwise=.2;
                    break;
                case ERROR:
                    if(recoverable&&retries++<3&&destination!=null)enter(State.MOVING);
                    else if(routine==Routine.SINGLE_TILT)beginParking();
                    else enter(State.WAITING);break;
                case WAIT_TILT:
                    int settled=hiveVision.state(now);
                    if(settled>=0&&settled!=finalShotHiveState){
                        flowerTiltConfirmed=true;flipped=settled==1;enter(State.WAITING);
                    }else if(age>5)fail("Flower Hive tilt not confirmed",false);
                    break;
                case WAITING:break; // Hold with all actuators off until parking starts at 26 seconds.
                case PARK:
                    Outputs park=navigator.move(sensors.pose,destination);
                    if(park==null)running=false;else out=park;break;
                default:break;
            }
        }catch(java.util.concurrent.CancellationException e){
            stop();return;
        }catch(IllegalStateException e){
            if(state==State.PARK)running=false;
            else fail(e.getMessage(),true);
        }
        // State transitions cancel every actuator from the previous state in this same tick.
        if(!running||state==State.ERROR||state!=before)out=new Outputs();
        // Start intake while approaching a pickup spot, before the chassis pushes the balls away.
        if(running&&state==State.MOVING&&afterMove==State.INTAKE&&sensors.ballCount<4&&sensors.pose.distance(destination)<24){out.intakePower=1;out.pipeline=1;}
        if(running&&state==State.MOVING&&afterMove==State.SHOOTING){out.shooterPower=power;if(Double.isNaN(shooterAt))shooterAt=now;}
        if(io.stopRequested()){stop();return;}
        last=out;io.apply(out);
    }
}
