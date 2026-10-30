package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.List;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Hardware Team Two: opposite-wall shots, independently verified own far Cell, Flower, park. */
public final class TeamTwoHiveAuto implements RobotController {
    public enum State { TO_HIVE, VERIFY_HIVE, SHOOTING, TO_FLOWER, PRESS_FLOWER, CLEAR_FLOWER, TO_OUTLET_LANE, TO_PICKUP, INTAKE, SETTLING, WAITING, PARK, STOPPED, ERROR }
    private final RobotIO io;
    private final FieldLayout field;
    private final Navigator nav;
    private HiveVision hive,shotGate;
    private HiveShotGate centeredShotGate;
    private State state=State.STOPPED;
    private Pose current,destination,progressPose,parkVia;
    private boolean running,lastFeeding,batchFed;
    private boolean collecting;
    private int alliance,batches;
    private double startAt,lastNow,stateAt,power,spinAt,feedTime,lastCommandAt,progressAt;
    private String message="",error="";
    public double spinUpSeconds=1,feedSeconds=3,parkingToleranceInches=2;
    private int measuredBalls=-1,measuredPollen=-1;
    public TeamTwoHiveAuto(RobotIO io,FieldLayout field){this.io=io;this.field=field;nav=new Navigator(field,io::stopRequested);}
    public void start(int alliance,double power){
        if(alliance<0||alliance>1||!Double.isFinite(power)||power<0||power>1
                ||!Double.isFinite(spinUpSeconds)||spinUpSeconds<0||!Double.isFinite(feedSeconds)||feedSeconds<=0
                ||!Double.isFinite(parkingToleranceInches)||parkingToleranceInches<=0||parkingToleranceInches>3)
            throw new IllegalArgumentException("Invalid Team Two configuration");
        this.alliance=alliance;this.power=power;startAt=lastNow=lastCommandAt=io.nowSeconds();running=true;
        batches=0;collecting=false;lastFeeding=batchFed=false;feedTime=0;spinAt=Double.NaN;current=progressPose=parkVia=null;error=message="";
        hive=new HiveVision(field,alliance);shotGate=new HiveVision(field,alliance);centeredShotGate=new HiveShotGate(field,alliance,true);nav.useScriptedRoute(true,alliance);
        move(State.TO_HIVE,field.teamTwoWallHive[alliance]);io.apply(new Outputs());
    }
    private void enter(State next){
        state=next;stateAt=io.nowSeconds();nav.reset();progressPose=current;progressAt=stateAt;
        if(next==State.TO_FLOWER)collecting=true;
        if(next==State.PARK||next==State.TO_HIVE||next==State.ERROR||next==State.STOPPED)collecting=false;
    }
    private void move(State next,Pose target){destination=target;enter(next);}
    public void stop(){running=false;state=State.STOPPED;lastFeeding=false;io.apply(new Outputs());}
    public boolean isRunning(){return running;}
    public State getState(){return state;}
    public String lastError(){return error;}
    public String status(){return state+String.format(java.util.Locale.ROOT," · %.1f / 30 s · ",io.nowSeconds()-startAt)+message
            +" · measured "+measuredPollen+" pollen / "+measuredBalls+" total · feed attempts "+batches+" / 2 · require FOUR pollen and own far Cell UP · park by 26 s; stop at 30 s";}
    public Pose destination(){return running&&(state==State.TO_HIVE||state==State.TO_FLOWER||state==State.CLEAR_FLOWER||state==State.TO_OUTLET_LANE||state==State.TO_PICKUP||state==State.PARK)?destination:null;}
    public List<Pose> route(){return nav.route();}
    private void park(){
        lastFeeding=false;spinAt=Double.NaN;
        // Stay on the wall side of the Hive supports before heading up the alliance lane.
        // The long diagonal can graze a frame corner between navigation samples.
        parkVia=(alliance==0?current.y:-current.y)<-32
                ?new Pose(alliance==0?-56:56,alliance==0?-40:40,current.heading):null;
        Pose slot=field.teamTwoGardenSidePark[alliance];destination=new Pose(slot.x,slot.y,current.heading);enter(State.PARK);message="Other-half parking; shooter, feeder and intake OFF";
    }
    private boolean fullPollenLoad(Sensors s){return s.ballCount==4&&s.pollenCount==4;}
    private Pose flowerApproach(){return field.teamTwoFlowerTouch[alliance];}
    private int pipeline(){return state==State.TO_FLOWER||state==State.PRESS_FLOWER||state==State.CLEAR_FLOWER||state==State.TO_OUTLET_LANE||state==State.TO_PICKUP||state==State.INTAKE?1:0;}
    private Target flowerPollen(Vision vision,double now){
        if(!vision.fresh(now,1))return null;
        Target best=null;
        for(Target t:vision.targets)if(t.id<0&&"yellow".equals(t.color)&&Double.isFinite(t.tx)&&Double.isFinite(t.areaPercent)&&t.areaPercent>0
                &&(best==null||t.areaPercent>best.areaPercent))best=t;
        return best;
    }
    private void fail(String why){error=message=why;running=false;lastFeeding=false;state=State.ERROR;}
    private boolean farCellReady(Sensors s,double now){
        // Require positive fresh observations of the RAISED far cluster, not just an absent near cluster.
        Pose shot=field.teamTwoWallHive[alliance];
        return current.distance(shot)<=1&&Math.abs(Pose.angle(current.heading-shot.heading))<=Math.toRadians(2)
                &&s.cameraConnected&&shotGate.state(now)==1&&shotGate.aim(now,true)!=null&&centeredShotGate.ready(s,now);
    }
    /** Only unique raised far-cluster observations can build shooting permission. */
    private Vision farTags(Vision vision){
        List<Target> selected=new ArrayList<Target>();
        for(int id:field.farTags[alliance])for(Target target:vision.targets)if(target.id==id){selected.add(target);break;}
        return new Vision(vision.valid&&!selected.isEmpty(),vision.pipeline,vision.capturedAt,selected.toArray(new Target[0]));
    }
    private void finishFeed(){
        batches++;lastFeeding=false;spinAt=Double.NaN;feedTime=0;
        if(batches==1){move(State.TO_FLOWER,flowerApproach());message="Verified empty after first load; approach Flower for camera-guided pollen pickup";}
        else{enter(State.SETTLING);message="Second feed attempt complete; allow flight/settling, then wait for parking";}
    }
    private Outputs guardMove(Outputs out){
        if(out.forward==0&&out.strafeRight==0&&out.turnClockwise==0)return out;
        double dt=.06,h=Pose.angle(current.heading-out.turnClockwise*1.65*dt);
        Pose next=new Pose(current.x+(out.forward*Math.cos(h)+out.strafeRight*Math.sin(h))*25*dt,
                current.y+(out.forward*Math.sin(h)-out.strafeRight*Math.cos(h))*25*dt,h);
        if(!field.canRotate(current,h)||!field.clear(current,next,h)){
            // Slow near a wall/waypoint rather than rejecting a full-speed lookahead
            // beyond the target. Every reduced command still passes collision checks.
            for(double scale:new double[]{.5,.25,.125}){
                double sh=Pose.angle(current.heading-out.turnClockwise*scale*1.65*dt);
                Pose q=new Pose(current.x+(out.forward*Math.cos(sh)+out.strafeRight*Math.sin(sh))*scale*25*dt,
                        current.y+(out.forward*Math.sin(sh)-out.strafeRight*Math.cos(sh))*scale*25*dt,sh);
                if(field.canRotate(current,sh)&&field.clear(current,q,sh)){
                    out.forward*=scale;out.strafeRight*=scale;out.turnClockwise*=scale;return out;
                }
            }
            if(state==State.PARK)fail("Parking corridor blocked; all outputs OFF");
            else if(state==State.INTAKE)message="Flower approach blocked; camera search held safely";
            else{error=message="Route clearance blocked; holding safely for parking";enter(State.WAITING);}
            return new Outputs();
        }
        return out;
    }
    public void tick(){
        Outputs out=new Outputs();
        if(!running){io.apply(out);return;}
        if(io.stopRequested()){stop();return;}
        try{
            double now=io.nowSeconds();
            if(!Double.isFinite(now)||now<lastNow){fail("Clock invalid");io.apply(out);return;}
            if(now-startAt>=30){running=false;message="30-second hard stop";io.apply(out);return;}
            Sensors s=io.readSensors();now=io.nowSeconds();
            if(io.stopRequested()){stop();return;}
            if(!Double.isFinite(now)||now<lastNow){fail("Clock invalid");io.apply(out);return;}lastNow=now;
            if(now-startAt>=30){running=false;message="30-second hard stop";io.apply(out);return;}
            if(!s.odometryValid||!s.pose.finite()||now-s.sampledAt<0||now-s.sampledAt>.25){fail("Odometry unavailable or stale");io.apply(out);return;}
            current=s.pose;measuredBalls=s.ballCount;measuredPollen=s.pollenCount;double elapsed=now-startAt;
            if(s.ballCount>=4)collecting=false;
            if(elapsed>=26&&state!=State.PARK)park();
            // Never spend the last return-travel reserve on a shot/pickup/verification wait.
            if(state!=State.PARK&&state!=State.WAITING&&state!=State.SETTLING&&elapsed+current.distance(field.teamTwoGardenSidePark[alliance])*1.35/Math.max(1,25*field.drivePower)+1.0>=29.3)park();
            hive.update(s.vision,now);
            shotGate.update(farTags(s.vision),now);
            State before=state;double age=now-stateAt;
            switch(state){
                case TO_HIVE:
                case TO_FLOWER:
                case CLEAR_FLOWER:
                case TO_OUTLET_LANE:
                case TO_PICKUP:
                    out=nav.move(current,destination);
                    if(out==null){
                        out=new Outputs();
                        if(state==State.TO_HIVE){
                            feedTime=0;spinAt=Double.NaN;lastFeeding=batchFed=false;
                            // Permission observed in transit must not authorize a later shot.
                            hive=new HiveVision(field,alliance);shotGate=new HiveVision(field,alliance);centeredShotGate=new HiveShotGate(field,alliance,true);enter(State.VERIFY_HIVE);
                            message="At opposite wall; require fresh raised own-Hive far Cell tags before shooting";
                        }
                        else if(state==State.TO_FLOWER){enter(State.PRESS_FLOWER);message="Bounded low-power Flower trigger contact";}
                        else if(state==State.CLEAR_FLOWER){move(State.TO_OUTLET_LANE,field.teamTwoFlowerLane[alliance]);message="Back into the outlet lane without crossing released pollen";}
                        else if(state==State.TO_OUTLET_LANE){move(State.TO_PICKUP,field.teamTwoFlowerPickup[alliance]);message="Face incoming Flower pollen from the open lane";}
                        else{enter(State.INTAKE);message="Find yellow Flower pollen with Limelight; require four measured pollen";}
                    }else{
                        if(progressPose==null||current.distance(progressPose)>.25||Math.abs(Pose.angle(current.heading-progressPose.heading))>Math.toRadians(3)){
                            progressPose=current;progressAt=now;
                        }
                        if(now-progressAt>3){error=message="Movement stalled; holding safely for parking";enter(State.WAITING);}
                    }
                    break;
                case VERIFY_HIVE:
                    // No spinning or feeding before the independent, debounced orientation check.
                    message=fullPollenLoad(s)?"Four pollen measured; require fresh own-alliance far Cell UP":"Shooting disabled: need FOUR type-verified pollen, not a timer/unknown count";
                    if(fullPollenLoad(s)&&farCellReady(s,now)){spinAt=Double.NaN;enter(State.SHOOTING);message="Four pollen AND raised far Cell confirmed; spin up at wall";}
                    break;
                case SHOOTING:
                    // Admit a complete batch once. Counts naturally drop after each real shot;
                    // do not demand four remaining balls after the first ball has fired.
                    if(!batchFed&&!fullPollenLoad(s)){spinAt=Double.NaN;enter(State.VERIFY_HIVE);message="Full load changed before feed; reverify four pollen";break;}
                    if(batchFed&&s.ballCount==0&&s.pollenCount==0){finishFeed();break;}
                    if(s.ballCount<0||s.pollenCount<0||s.ballCount!=s.pollenCount){spinAt=Double.NaN;lastFeeding=false;message="Unknown/mixed inventory: shooter and feeder OFF";break;}
                    // If the final batch tips the Hive, do not fire leftovers at the now-lowered face.
                    if(batches==1&&batchFed&&s.cameraConnected&&hive.state(now)==0){finishFeed();message="Opposite Hive orientation confirmed; stop final feed";break;}
                    if(!farCellReady(s,now)){
                        spinAt=Double.NaN;lastFeeding=false;message="Tag lost / stale / wrong Cell: shooter and feeder OFF";break;
                    }
                    if(lastFeeding)feedTime+=Math.min(.10,Math.max(0,now-lastCommandAt));
                    if(feedTime>=feedSeconds){error=message="Feed timeout with pollen still measured; no assumed empty load";enter(State.WAITING);break;}
                    if(Double.isNaN(spinAt))spinAt=now;
                    out.shooterPower=power;
                    if(now-spinAt>=spinUpSeconds)out.feederPower=1;
                    message="Fresh own far Cell UP; "+(out.feederPower>0?"timed feeding":"spinning up");
                    break;
                case PRESS_FLOWER:
                    // Deliberate contact is required to release pollen. Bounded to 0.4 s
                    // at 12% strafe; never bypass STOP, park deadline or odometry validity.
                    if(age>=.4){move(State.CLEAR_FLOWER,field.teamTwoFlowerClear[alliance]);message="Flower trigger attempt complete; clear the outlet";}
                    else out.strafeRight=-.12;
                    break;
                case INTAKE:
                    if(fullPollenLoad(s)){move(State.TO_HIVE,field.teamTwoWallHive[alliance]);message="FOUR Flower pollen measured; return to wall and recheck tags";break;}
                    if(s.ballCount>=4){error=message="Full inventory is not four pollen; cannot shoot mixed/unknown load";enter(State.WAITING);break;}
                    if(!s.cameraConnected){message="Flower camera disconnected; drive/intake OFF";break;}
                    Target target=flowerPollen(s.vision,now);
                    if(target==null){
                        enter(State.WAITING);message="No fresh yellow target; hold still until parking, no search spin";
                    }else{
                        message="Flower intake: forward/lateral alignment only; no camera hunting turns";out.intakePower=1;
                        out.forward=target.areaPercent<2?.4:.22;
                        out.strafeRight=Math.abs(target.tx)>6?FieldLayout.clip(target.tx*.012,.15):0;
                        out.turnClockwise=0;
                        if(Math.abs(target.tx)>20)out.forward=0;
                    }
                    if(current.distance(field.teamTwoFlowerPickup[alliance])>30){error=message="Flower search boundary reached; no blind wandering";enter(State.WAITING);}
                    break;
                case SETTLING:
                    if(age>=3.5){enter(State.WAITING);message="Scoring attempts complete; wait for parking";}
                    break;
                case WAITING:break;
                case PARK:
                    if(current.distance(destination)<=parkingToleranceInches){running=false;message="Parked: all outputs OFF";break;}
                    out=nav.move(current,parkVia==null?destination:parkVia);
                    if(out==null){
                        out=new Outputs();
                        if(parkVia!=null){parkVia=null;nav.reset();}
                        else{running=false;message="Parked: all outputs OFF";}
                    }
                    break;
                default:break;
            }
            if(state!=before||!running)out=new Outputs();
            else if(state==State.TO_HIVE||state==State.TO_FLOWER||state==State.CLEAR_FLOWER||state==State.TO_OUTLET_LANE||state==State.TO_PICKUP||state==State.INTAKE||state==State.PARK)out=guardMove(out);
            out.pipeline=pipeline();
            out.intakePower=running&&collecting&&s.ballCount<4&&state!=State.PARK?1:0;
            lastFeeding=out.feederPower>0;batchFed|=lastFeeding;lastCommandAt=now;
            if(io.stopRequested()){stop();return;}
            io.apply(out);
        }catch(java.util.concurrent.CancellationException e){stop();}
        catch(RuntimeException e){fail("Team Two stopped safely: "+e.getMessage());io.apply(new Outputs());}
    }
}
