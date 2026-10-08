package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** QuantumStorm's autonomous sequence, independent of FTC SDK, browser, or wall-clock timers. */
public final class AutoStateMachine {
    public enum State { INIT, MOVING, SHOOTING, INTAKE, ERROR, PARK, STOPPED }
    private enum ShootStep { CHECK_BALLS, AIM, FIRE }
    private enum Source { GARDEN, FLOWER, GROUND }
    private final RobotIO io;
    private final FieldLayout field;
    private final Navigator navigator;
    private State state=State.STOPPED, afterMove;
    private ShootStep shootStep;
    private Source source;
    private Pose destination;
    private String destinationName="",error="";
    private boolean running,gardenDone,flowerDone,flipped,recoverable;
    private int alliance,retries;
    private double startAt,stateAt,lastNow,power;
    private Outputs last=new Outputs();
    public AutoStateMachine(RobotIO io,FieldLayout field) { this.io=io;this.field=field;this.navigator=new Navigator(field); }
    public void start(int alliance,double power) {
        if(alliance<0||alliance>1||!Double.isFinite(power)||power<0||power>1)throw new IllegalArgumentException("Invalid alliance or shooter power");
        this.alliance=alliance;this.power=power;startAt=lastNow=io.nowSeconds();running=true;retries=0;
        gardenDone=flowerDone=flipped=false;error="";destination=null;enter(State.INIT);io.apply(new Outputs());
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
            +String.format(java.util.Locale.ROOT," · %.1f / 30 s",elapsed())+(error.isEmpty()?"":" · "+error);
    }
    private void enter(State state) { this.state=state;stateAt=io.nowSeconds();navigator.reset();if(state==State.SHOOTING)shootStep=ShootStep.CHECK_BALLS; }
    private void moveThen(Pose target,String name,State after) { destination=target;destinationName=name;afterMove=after;enter(State.MOVING); }
    private Pose hive() { return flipped?field.farHive[alliance]:field.hive[alliance]; }
    private void pickup() {
        if(!gardenDone){source=Source.GARDEN;moveThen(field.garden[alliance],"Garden",State.INTAKE);}
        else if(!flowerDone){source=Source.FLOWER;moveThen(field.flower[alliance],"Flower",State.INTAKE);}
        else{source=Source.GROUND;enter(State.INTAKE);}
    }
    private void finishPickup() { if(source==Source.GARDEN)gardenDone=true;if(source==Source.FLOWER)flowerDone=true;moveThen(hive(),"Hive",State.SHOOTING); }
    private void fail(String reason,boolean recoverable) { error=reason;this.recoverable=recoverable;enter(State.ERROR); }
    private Target tag(Vision vision,double now) {
        if(!vision.fresh(now,0))return null;
        Target near=cluster(vision,field.nearTags[alliance]),far=cluster(vision,field.farTags[alliance]);
        // A cluster center avoids switching aim between its left/right individual tags.
        // An incidental far-face cluster does not imply a flip while the intended near cluster is visible.
        return flipped?far:near!=null?near:far;
    }
    private Target cluster(Vision vision,int[] ids) {
        int n=0,id=-1;double tx=0,ty=0;
        for(Target t:vision.targets)if(contains(ids,t.id)){n++;id=t.id;tx+=t.tx;ty+=t.ty;}
        return n==0?null:new Target(id,tx/n,ty/n);
    }
    private static boolean contains(int[] ids,int id) { for(int i:ids)if(i==id)return true;return false; }
    public void tick() {
        Outputs out=new Outputs();double now=io.nowSeconds();
        if(!running){last=out;io.apply(out);return;}
        Sensors sensors=io.readSensors();now=io.nowSeconds();
        if(!Double.isFinite(now)||now<lastNow){fail("Clock moved backward",false);running=false;last=out;io.apply(out);return;}lastNow=now;
        if(elapsed()>=30){running=false;state=State.PARK;last=out;io.apply(out);return;}
        if(!sensors.odometryValid||!sensors.pose.finite()||now-sensors.sampledAt<0||now-sensors.sampledAt>.25){
            fail("Odometry unavailable or stale",false);running=false;last=out;io.apply(out);return;
        }
        if(sensors.ballCount<0||sensors.ballCount>4||!sensors.cameraConnected){
            fail(sensors.ballCount<0||sensors.ballCount>4?"Ball-count sensor unavailable":"Camera disconnected",false);running=false;last=out;io.apply(out);return;
        }
        if(elapsed()>=26&&state!=State.PARK){destination=field.park(sensors.pose);enter(State.PARK);}
        double age=now-stateAt;
        State before=state;
        try {
            switch(state) {
                case INIT:moveThen(hive(),"Hive",State.SHOOTING);break;
                case MOVING:
                    Outputs drive=navigator.move(sensors.pose,destination);
                    if(drive==null)enter(afterMove);else out=drive;
                    if(state==State.MOVING&&age>5)fail("Destination timeout",true);
                    break;
                case SHOOTING:
                    if(shootStep==ShootStep.CHECK_BALLS){
                        if(sensors.ballCount<4){pickup();break;}shootStep=ShootStep.AIM;stateAt=now;age=0;
                    }
                    out.shooterPower=power;
                    if(shootStep==ShootStep.AIM){
                        Target t=tag(sensors.vision,now);
                        if(t!=null&&!flipped&&contains(field.farTags[alliance],t.id)){flipped=true;moveThen(hive(),"Hive far face",State.SHOOTING);break;}
                        if(t!=null&&Math.abs(t.tx)<2&&age>=1){shootStep=ShootStep.FIRE;stateAt=now;age=0;}
                        else if(age>3){fail(t==null?"AprilTag not found":"AprilTag alignment timeout",true);break;}
                        else if(t!=null)out.turnClockwise=FieldLayout.clip(t.tx*.02,.6);
                    }
                    if(shootStep==ShootStep.FIRE){
                        if(sensors.ballCount==0){pickup();break;}
                        if(age>3){fail("Balls stuck in indexer",true);break;}out.feederPower=1;
                    }
                    break;
                case INTAKE:
                    if(sensors.ballCount>=4||age>(source==Source.GROUND?6:3)){finishPickup();break;}
                    out.pipeline=1;out.intakePower=1;
                    if(source==Source.FLOWER&&age<.2)out.forward=.2;
                    else if(source==Source.FLOWER&&age<1.5)out.forward=-.3;
                    else if(sensors.vision.fresh(now,1)&&sensors.vision.targets.length>0){
                        Target t=sensors.vision.targets[0];out.forward=Math.abs(t.tx)<20?.3:0;out.turnClockwise=FieldLayout.clip(t.tx*.02,.3);
                    }else out.turnClockwise=.2;
                    break;
                case ERROR:
                    if(recoverable&&retries++<3&&destination!=null)enter(State.MOVING);
                    else{destination=field.park(sensors.pose);enter(State.PARK);}break;
                case PARK:
                    Outputs park=navigator.move(sensors.pose,destination);
                    if(park==null||age>5)running=false;else out=park;break;
                default:break;
            }
        }catch(IllegalStateException e){if(state==State.PARK)running=false;else fail(e.getMessage(),true);}
        // State transitions cancel every actuator from the previous state in this same tick.
        if(!running||state==State.ERROR||state!=before)out=new Outputs();
        // Start intake while approaching a pickup spot, before the chassis pushes the balls away.
        if(running&&state==State.MOVING&&afterMove==State.INTAKE&&sensors.ballCount<4){out.intakePower=1;out.pipeline=1;}
        last=out;io.apply(out);
    }
}
