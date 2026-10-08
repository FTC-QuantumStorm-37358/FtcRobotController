package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** QuantumStorm's autonomous sequence, independent of FTC SDK, browser, or wall-clock timers. */
public final class AutoStateMachine {
    public static final double PARK_DEADLINE_SECONDS=26.0;
    public enum State { INIT, MOVING, SHOOTING, INTAKE, ERROR, PARK, STOPPED }
    private enum ShootStep { CHECK_BALLS, AIM, FIRE }
    private enum Source { GARDEN, FLOWER }
    private final RobotIO io;
    private final FieldLayout field;
    private final Navigator navigator;
    private HiveVision hiveVision;
    private State state=State.STOPPED, afterMove;
    private ShootStep shootStep;
    private Source source;
    private Pose destination;
    private Pose currentPose;
    private String destinationName="",error="";
    private boolean running,gardenDone,flowerDone,flipped,recoverable;
    private int alliance,retries;
    private double startAt,stateAt,lastNow,power;
    private double shooterAt,moveDeadline;
    private double nextParkBudgetCheck;
    private Outputs last=new Outputs();
    public AutoStateMachine(RobotIO io,FieldLayout field) { this.io=io;this.field=field;this.navigator=new Navigator(field); }
    public void start(int alliance,double power) {
        if(alliance<0||alliance>1||!Double.isFinite(power)||power<0||power>1)throw new IllegalArgumentException("Invalid alliance or shooter power");
        this.alliance=alliance;this.power=power;startAt=lastNow=io.nowSeconds();running=true;retries=0;
        hiveVision=new HiveVision(field,alliance);
        gardenDone=flowerDone=flipped=false;error="";destination=null;enter(State.INIT);io.apply(new Outputs());
        shooterAt=Double.NaN;nextParkBudgetCheck=0;
    }
    public void stop() { running=false;state=State.STOPPED;last=new Outputs();io.apply(last); }
    public boolean isRunning() { return running; }
    public State getState() { return state; }
    public Outputs getOutputs() { return last.copy(); }
    public Pose destination() { return running&&(state==State.MOVING||state==State.PARK)?destination:null; }
    public java.util.List<Pose> route() { return running&&(state==State.MOVING||state==State.PARK)?navigator.route():new java.util.ArrayList<Pose>(); }
    public double elapsed() { return Math.max(0,io.nowSeconds()-startAt); }
    public String status() {
        return state+(state==State.MOVING?" → "+destinationName:state==State.SHOOTING?" · "+shootStep:state==State.INTAKE?" · "+source:"")
            +String.format(java.util.Locale.ROOT," · %.1f / 26 s parking deadline",elapsed())+(error.isEmpty()?"":" · "+error);
    }
    private void enter(State state) { this.state=state;stateAt=io.nowSeconds();navigator.reset();if(state==State.SHOOTING)shootStep=ShootStep.CHECK_BALLS; }
    private void moveThen(Pose target,String name,State after) {
        destination=target;destinationName=name;afterMove=after;enter(State.MOVING);
        Pose current=currentPose;
        java.util.List<Pose> route=navigator.plan(current,target,target.heading);
        double distance=route==null?current.distance(target)*1.6:0;Pose last=current;if(route!=null)for(Pose p:route){distance+=last.distance(p);last=p;}
        moveDeadline=Math.max(3,distance/Math.max(1,25*field.drivePower)+5);
    }
    private Pose hive() { return flipped?field.farHive[alliance]:field.hive[alliance]; }
    private void pickup() {
        if(!gardenDone){source=Source.GARDEN;moveThen(field.garden[alliance],"Garden",State.INTAKE);}
        else if(!flowerDone){source=Source.FLOWER;moveThen(field.flower[alliance],"Flower",State.INTAKE);}
        else{destination=field.loadingPark[alliance];enter(State.PARK);}
    }
    private void finishPickup() {
        if(source==Source.GARDEN)gardenDone=true;if(source==Source.FLOWER)flowerDone=true;
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
        Outputs out=new Outputs();double now=io.nowSeconds();
        if(!running){last=out;io.apply(out);return;}
        Sensors sensors=io.readSensors();now=io.nowSeconds();
        if(!Double.isFinite(now)||now<lastNow){fail("Clock moved backward",false);running=false;last=out;io.apply(out);return;}lastNow=now;
        if(elapsed()>=PARK_DEADLINE_SECONDS){running=false;state=State.PARK;error="26-second parking deadline; all actuators stopped";last=out;io.apply(out);return;}
        if(!sensors.odometryValid||!sensors.pose.finite()||now-sensors.sampledAt<0||now-sensors.sampledAt>.25){
            fail("Odometry unavailable or stale",false);running=false;last=out;io.apply(out);return;
        }
        if(sensors.ballCount<0||sensors.ballCount>4||!sensors.cameraConnected){
            fail(sensors.ballCount<0||sensors.ballCount>4?"Ball-count sensor unavailable":"Camera disconnected",false);running=false;last=out;io.apply(out);return;
        }
        hiveVision.update(sensors.vision,now);
        currentPose=sensors.pose;
        // Recheck route-dependent parking reserve twice per second. The extra
        // half second covers movement between checks; never wait until 26 to depart.
        if(state!=State.PARK&&elapsed()>=nextParkBudgetCheck){
            nextParkBudgetCheck=elapsed()+.5;
            double required=navigator.parkingSeconds(sensors.pose,field.loadingPark[alliance])+.5;
            if(elapsed()+required>=PARK_DEADLINE_SECONDS){destination=field.loadingPark[alliance];enter(State.PARK);}
        }
        double age=now-stateAt;
        State before=state;
        try {
            switch(state) {
                case INIT:moveThen(hive(),"Hive",State.SHOOTING);break;
                case MOVING:
                    Outputs drive=navigator.move(sensors.pose,destination);
                    if(drive==null)enter(afterMove);else out=drive;
                    if(state==State.MOVING&&age>moveDeadline)fail("Destination timeout",false);
                    break;
                case SHOOTING:
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
                    if(age>4){fail("Pickup did not fill four slots",false);break;}
                    out.pipeline=1;out.intakePower=1;
                    if(source==Source.FLOWER&&age<.2)out.forward=.2;
                    else if(source==Source.FLOWER&&age<1.5)out.forward=-.3;
                    else if(sensors.vision.fresh(now,1)&&sensors.vision.targets.length>0){
                        Target t=sensors.vision.targets[0];out.forward=Math.abs(t.tx)<20?.3:0;out.turnClockwise=FieldLayout.clip(t.tx*.02,.3);
                    }else out.turnClockwise=.2;
                    break;
                case ERROR:
                    if(recoverable&&retries++<3&&destination!=null)enter(State.MOVING);
                    else{destination=field.loadingPark[alliance];enter(State.PARK);}break;
                case PARK:
                    Outputs park=navigator.move(sensors.pose,destination);
                    if(park==null)running=false;else out=park;break;
                default:break;
            }
        }catch(IllegalStateException e){if(state==State.PARK)running=false;else fail(e.getMessage(),true);}
        // State transitions cancel every actuator from the previous state in this same tick.
        if(!running||state==State.ERROR||state!=before)out=new Outputs();
        // Start intake while approaching a pickup spot, before the chassis pushes the balls away.
        if(running&&state==State.MOVING&&afterMove==State.INTAKE&&sensors.ballCount<4&&sensors.pose.distance(destination)<24){out.intakePower=1;out.pipeline=1;}
        if(running&&state==State.MOVING&&afterMove==State.SHOOTING){out.shooterPower=power;if(Double.isNaN(shooterAt))shooterAt=now;}
        last=out;io.apply(out);
    }
}
