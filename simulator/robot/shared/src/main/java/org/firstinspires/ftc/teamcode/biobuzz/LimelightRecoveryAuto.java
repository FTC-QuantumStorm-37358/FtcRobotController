package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Simulator experiment: four preloads, measured Hive tilt, visual pollen/nectar recovery.
 * Largest-color selection/strafe approach is adapted from PollenAndAprilTagAuto.java.
 * Synthetic inputs test control behavior, not the Limelight SDK or real image processing.
 */
public final class LimelightRecoveryAuto implements RobotController {
    public enum State { AIM, SHOOTING, OBSERVE_TILT, WAIT_TILT, CLEAR_FENCE, RECENTER, SEARCH_POLLEN, APPROACH_POLLEN, WAITING, CAMERA_HOLD, PARK, STOPPED, ERROR }
    private final RobotIO io;
    private final FieldLayout field;
    private final Navigator nav;
    private HiveVision hive;
    private State state=State.STOPPED,resume;
    private boolean running,tiltConfirmed,recovered;
    private int alliance,startCount,lastCount;
    private double startAt,stateAt,power,lastNow,feedAt,spinAt;
    private Pose destination,current;
    private String message="";
    private int blockedTicks;
    private double avoidUntil,avoidStrafe;
    private boolean survey;
    private double surveyAngle,surveyHeading,bestArea,bestHeading;
    private String lastTargetColor="",surveyColor="";
    private boolean bestPreferred;
    public LimelightRecoveryAuto(RobotIO io,FieldLayout field){this.io=io;this.field=field;nav=new Navigator(field,io::stopRequested);}
    public void start(int alliance,double power){
        if(alliance<0||alliance>1||!Double.isFinite(power)||power<0||power>1)throw new IllegalArgumentException("Invalid alliance or shooter power");
        this.alliance=alliance;this.power=power;startAt=lastNow=io.nowSeconds();running=true;tiltConfirmed=recovered=false;
        startCount=lastCount=-1;feedAt=spinAt=Double.NaN;current=destination=null;message="";blockedTicks=0;avoidUntil=0;survey=false;lastTargetColor="";
        hive=new HiveVision(field,alliance);nav.useScriptedRoute(true,alliance);enter(State.AIM);io.apply(new Outputs());
    }
    private void enter(State next){
        state=next;stateAt=io.nowSeconds();nav.reset();
        if(next==State.SEARCH_POLLEN&&lastCount>=2){
            survey=true;surveyAngle=0;surveyHeading=current.heading;bestArea=0;bestHeading=Double.NaN;bestPreferred=false;
            // Seek the other visible color after a batch, not an inferred inventory type.
            surveyColor="yellow".equals(lastTargetColor)?(alliance==0?"red":"blue"):"yellow";
        }
    }
    private double elapsed(){return io.nowSeconds()-startAt;}
    public void stop(){running=false;state=State.STOPPED;io.apply(new Outputs());}
    public boolean isRunning(){return running;}
    public State getState(){return state;}
    public String status(){return state+String.format(java.util.Locale.ROOT," · %.1f / 30 s",elapsed())
        +" · SIMULATOR ONLY: Limelight pollen + nectar test · "+message
        +(tiltConfirmed?" · Hive tilt confirmed":" · Awaiting measured Hive state")
        +(recovered?" · Recovered "+lastCount+" / 4 balls":"")+" · collect until full / parking reserve; park by 26 s; stop at 30 s";}
    public Pose destination(){return running&&(state==State.OBSERVE_TILT||state==State.CLEAR_FENCE||state==State.RECENTER||state==State.PARK)?destination:null;}
    public List<Pose> route(){return destination()==null?new ArrayList<Pose>():nav.route();}
    private void park(){destination=field.parkingTarget(current,alliance);message="Parking; vision collection ended";enter(State.PARK);}
    private void fail(String reason){message=reason;running=false;state=State.ERROR;}
    private boolean roomToScan(Pose pose){
        for(int i=0;i<24;i++)if(!field.allowed(new Pose(pose.x,pose.y,i*Math.PI/12)))return false;
        return true;
    }
    /** Leave the wall/frame before a full visual scan; never a ball pickup waypoint. */
    private void findNextBall(){
        Pose inset=new Pose(FieldLayout.clip(current.x,55),FieldLayout.clip(current.y,55),current.heading);
        if(lastCount>=2&&current.distance(inset)>.65&&roomToScan(inset)&&field.clear(current,inset,current.heading)){destination=inset;enter(State.RECENTER);return;}
        if(roomToScan(current)){enter(State.SEARCH_POLLEN);return;}
        if(roomToScan(inset)&&field.clear(current,inset,current.heading)){destination=inset;enter(State.RECENTER);return;}
        for(int radius=4;radius<=20;radius+=4)for(int i=0;i<8;i++){
            Pose candidate=new Pose(current.x+radius*Math.cos(i*Math.PI/4),current.y+radius*Math.sin(i*Math.PI/4),current.heading);
            if(roomToScan(candidate)&&field.clear(current,candidate,current.heading)){destination=candidate;enter(State.RECENTER);return;}
        }
        message="No safe scan clearance; waiting for parking";enter(State.WAITING);
    }
    /** Same largest-visible-color rule as the imported Limelight approach file. */
    private Target collectable(Vision vision,double now){
        return collectable(vision,now,"");
    }
    private Target collectable(Vision vision,double now,String wanted){
        if(!vision.fresh(now,1))return null;
        Target best=null;
        for(Target t:vision.targets)if(t.id<0&&("yellow".equals(t.color)||(alliance==0?"red":"blue").equals(t.color))&&(wanted.isEmpty()||wanted.equals(t.color))&&Double.isFinite(t.tx)&&Double.isFinite(t.areaPercent)&&t.areaPercent>0
                &&(best==null||t.areaPercent>best.areaPercent))best=t;
        return best;
    }
    private Outputs approach(Target target){
        Outputs out=new Outputs();out.pipeline=1;out.intakePower=1;
        // Faster cruise, with a slower final approach and unchanged collision guards.
        // Angular steering never uses hidden field coordinates.
        out.forward=target.areaPercent<.4?.70:target.areaPercent<1.2?.50:.25;
        out.strafeRight=Math.abs(target.tx)<=2?0:FieldLayout.clip(target.tx*.012,.20);
        out.turnClockwise=FieldLayout.clip(target.tx*.025,.50);
        if(Math.abs(target.tx)>20)out.forward=0;
        if(target.areaPercent>=12){out.forward=0;out.strafeRight=0;out.turnClockwise=0;}
        return out;
    }
    /** Collision-aware short-horizon guard; observations still choose the direction. */
    private Outputs safe(Outputs out){
        if(state==State.APPROACH_POLLEN&&io.nowSeconds()<avoidUntil){
            out.forward=out.turnClockwise=0;out.strafeRight=avoidStrafe;
        }
        double dt=.12,heading=Pose.angle(current.heading-out.turnClockwise*1.65*dt);
        double dx=(out.forward*Math.cos(heading)+out.strafeRight*Math.sin(heading))*25*dt;
        double dy=(out.forward*Math.sin(heading)-out.strafeRight*Math.cos(heading))*25*dt;
        Pose next=new Pose(current.x+dx,current.y+dy,heading);
        if(!field.canRotate(current,heading)||!field.clear(current,next,heading)){
            // A visible target may be behind a frame leg. Try a short, validated
            // sideways clearance while preserving the live camera bearing.
            // No Garden/Flower coordinates or remembered ball position is used.
            if(state==State.APPROACH_POLLEN&&out.forward>0){
                for(double side:new double[]{-.3,.3}){
                    Pose lateral=new Pose(current.x+Math.sin(current.heading)*side*25*.95,current.y-Math.cos(current.heading)*side*25*.95,current.heading);
                    Pose ahead=new Pose(lateral.x+Math.cos(current.heading)*3,lateral.y+Math.sin(current.heading)*3,current.heading);
                    if(field.clear(current,lateral,current.heading)&&field.clear(lateral,ahead,current.heading)){
                        avoidUntil=io.nowSeconds()+.95;avoidStrafe=side;blockedTicks=0;
                        out.forward=out.turnClockwise=0;out.strafeRight=side;message="Live ball target; collision-checked frame-leg clearance";return out;
                    }
                }
            }
            blockedTicks++;out.forward=out.strafeRight=out.turnClockwise=out.intakePower=0;
            message="Visible target approach blocked; no blind driving";
            if(blockedTicks>=25){message="Recovery corridor blocked; waiting safely for parking";enter(State.WAITING);}
        }else blockedTicks=0;
        return out;
    }
    public void tick(){
        Outputs out=new Outputs();if(!running){io.apply(out);return;}
        if(io.stopRequested()){stop();return;}
        try{
            Sensors s=io.readSensors();double now=io.nowSeconds();
            if(io.stopRequested()){stop();return;}
            if(!Double.isFinite(now)||now<lastNow){fail("Clock invalid");io.apply(out);return;}lastNow=now;
            if(elapsed()>=30){message="30-second hard stop";running=false;io.apply(out);return;}
            if(!s.odometryValid||!s.pose.finite()||now-s.sampledAt<0||now-s.sampledAt>.25){fail("Odometry unavailable or stale");io.apply(out);return;}
            current=s.pose;
            if(s.ballCount<0||s.ballCount>4){fail("Measured inventory required for recovery test");io.apply(out);return;}
            if(startCount<0){startCount=lastCount=s.ballCount;if(startCount!=4){fail("Load exactly four preloads for this test");io.apply(out);return;}}
            if(elapsed()>=26&&state!=State.PARK)park();
            // Vision targets can roll anywhere. Reserve return travel instead
            // of assuming a fixed pickup location will be near the park zone.
            if((state==State.SEARCH_POLLEN||state==State.APPROACH_POLLEN||state==State.RECENTER||state==State.WAITING)&&elapsed()+current.distance(field.parkingTarget(current,alliance))*1.6/25+1.2>=29)park();
            if(state!=State.PARK&&state!=State.WAITING&&!s.cameraConnected){
                if(state!=State.CAMERA_HOLD){resume=state;enter(State.CAMERA_HOLD);}message="Camera disconnected; all outputs OFF";io.apply(out);return;
            }
            if(state==State.CAMERA_HOLD){enter(resume);message="Camera reconnected";}
            if(out.pipeline==0&&state!=State.PARK)hive.update(s.vision,now);
            double age=now-stateAt;State before=state;
            switch(state){
                case AIM:
                    Target aim=hive.aim(now,false);
                    message="Pipeline 0: own raised Hive tags; four preloads";
                    if(aim!=null&&Math.abs(aim.tx)<2){out.shooterPower=power;if(Double.isNaN(spinAt))spinAt=now;if(now-spinAt>=1){feedAt=now;enter(State.SHOOTING);}}
                    else{spinAt=Double.NaN;if(aim!=null)out.turnClockwise=FieldLayout.clip(aim.tx*.02,.3);}
                    break;
                case SHOOTING:
                    if(s.ballCount==0){
                        Pose start=field.hive[alliance];
                        destination=new Pose(start.x,start.y+(alliance==0?-18:18),start.heading);
                        message="Four preloads fed; move to tag-observation clearance, intake OFF";enter(State.OBSERVE_TILT);break;
                    }
                    if(now-feedAt>3){fail("Preload feed did not empty measured inventory");break;}
                    Target fire=hive.aim(now,false);
                    // No fresh own-tag aim, no feeding. No any-tag-after-10s bypass.
                    if(fire!=null&&Math.abs(fire.tx)<2){out.shooterPower=power;out.feederPower=1;}
                    break;
                case OBSERVE_TILT:
                    Outputs observe=nav.move(current,destination);
                    if(observe==null){message="Observe stable Hive tilt from clearance";enter(State.WAIT_TILT);}else out=observe;
                    break;
                case WAIT_TILT:
                    message="Pipeline 0: require measured opposite Cell up; no timer-inferred tilt";
                    if(hive.state(now)==1){
                        tiltConfirmed=true;lastCount=s.ballCount;
                        // Do not force a 180-degree turn: fallen nectar can be
                        // directly in front of the intake, on the fence side.
                        // Back away slightly to bring nearby fallen nectar into
                        // the front camera's view and clear old chassis contact.
                        destination=new Pose(current.x,current.y+(alliance==0?-8:8),current.heading);
                        message="Tilt confirmed; back away for front-camera pickup clearance";enter(State.CLEAR_FENCE);
                    }
                    break;
                case CLEAR_FENCE:
                    Outputs clear=nav.move(current,destination);
                    if(clear==null){message="Pipeline 1: scan for pollen and alliance nectar";enter(State.SEARCH_POLLEN);}else out=clear;
                    break;
                case RECENTER:
                    Outputs recenter=nav.move(current,destination);
                    if(recenter==null){message="Scan clearance reached; looking for next ball";enter(State.SEARCH_POLLEN);}
                    else{out=recenter;out.pipeline=1;}
                    break;
                case SEARCH_POLLEN:
                case APPROACH_POLLEN:
                    out.pipeline=1;
                    if(s.ballCount>lastCount){
                        recovered=true;lastCount=s.ballCount;
                        if(s.ballCount>=4){park();message="Four measured balls loaded; parking without extra shots";}
                        else{message="Measured pickup; searching for the next pollen or nectar ball";findNextBall();}
                        break;
                    }
                    if(s.ballCount>=4){park();message="Inventory full; parking";break;}
                    if(state==State.SEARCH_POLLEN&&survey){
                        if(!roomToScan(current)){findNextBall();break;}
                        surveyAngle+=Math.abs(Pose.angle(current.heading-surveyHeading));surveyHeading=current.heading;
                        Target seen=collectable(s.vision,now,surveyColor);boolean preferred=seen!=null;
                        if(seen==null)seen=collectable(s.vision,now);
                        if(seen!=null&&((preferred&&!bestPreferred)||(preferred==bestPreferred&&seen.areaPercent>bestArea))){bestPreferred=preferred;bestArea=seen.areaPercent;bestHeading=Pose.angle(current.heading-Math.toRadians(seen.tx));}
                        if(surveyAngle<2*Math.PI-.08){
                            message="Surveying all directions for the closest-looking final ball";out.turnClockwise=.85;out=safe(out);break;
                        }
                        if(Double.isFinite(bestHeading)){
                            double error=Pose.angle(bestHeading-current.heading);
                            if(Math.abs(error)>Math.toRadians(4)){message="Reacquiring best camera bearing";out.turnClockwise=-FieldLayout.clip(Math.toDegrees(error)*.025,.85);out=safe(out);break;}
                        }
                        survey=false;stateAt=now;message="Survey complete; require a fresh visible target before approaching";break;
                    }
                    if(!s.vision.fresh(now,1)){
                        message="Pipeline 1: no fresh ball target; translation and intake OFF";
                        if(state==State.APPROACH_POLLEN){enter(State.SEARCH_POLLEN);break;}
                        if(age<.20)break; // Allow one pipeline-switch/camera-delay cycle before scanning.
                        // Search rotates only; it never drives to Garden/Flower coordinates.
                        if(!roomToScan(current)){findNextBall();break;}
                        out.turnClockwise=.85;out=safe(out);break;
                    }
                    Target target=collectable(s.vision,now);
                    if(target==null){message="No pollen/alliance-nectar target; no approach";enter(State.SEARCH_POLLEN);break;}
                    if(state==State.SEARCH_POLLEN)enter(State.APPROACH_POLLEN);
                    lastTargetColor=target.color;
                    message=String.format(java.util.Locale.ROOT,"Pipeline 1: %s target tx %.1f deg, area %.2f%%",target.color,target.tx,target.areaPercent);
                    out=safe(approach(target));break;
                case WAITING:message=recovered?"Recovered balls retained; mechanisms OFF until parking":"Waiting safely until parking";break;
                case PARK:
                    if(current.distance(destination)<=.65){running=false;message="Parked: all outputs OFF";break;}
                    Outputs driving=nav.move(current,destination);if(driving==null){running=false;message="Parked: all outputs OFF";}else out=driving;break;
                default:break;
            }
            if(state!=before){
                out=new Outputs();
                // A state change cancels actuators, not the selected camera pipeline.
                // Otherwise SEARCH/APPROACH would repeatedly flush every color frame.
                if(state==State.SEARCH_POLLEN||state==State.APPROACH_POLLEN||state==State.RECENTER)out.pipeline=1;
            }
            if(!running)out=new Outputs();
            if(io.stopRequested()){stop();return;}io.apply(out);
        }catch(java.util.concurrent.CancellationException e){stop();}
        catch(RuntimeException e){fail("Recovery stopped safely: "+e.getMessage());io.apply(new Outputs());}
    }
}
