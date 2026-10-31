package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.List;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Team One: paced preloads, camera-guided Garden pickup, parking. */
public final class TeamOnePollenAuto implements RobotController {
    public enum State { SHOOTING, TO_GARDEN, TO_GARDEN_BACK, TO_GARDEN_RIGHT, GARDEN_ALIGN_HEADING, INTAKE_GARDEN, GARDEN_BACK_UP, GARDEN_RIGHT_SHIFT, GARDEN_RESTORE_HEADING, GARDEN_FORWARD_RETRY, OBSERVE_TILT, WAIT_TILT, CLEAR_PICKUP, SEARCH, APPROACH, CLOSE_PICKUP, REPOSITION, WAITING, PARK, STOPPED, ERROR }
    private final RobotIO io;
    private final FieldLayout field;
    private final Navigator nav;
    private HiveVision hive;
    private HiveShotGate shotGate;
    private boolean collecting,lastFeeding;
    private double feedElapsed,lastCommandAt;
    private State state=State.STOPPED;
    private Pose current,destination;
    private Pose gardenEntry;
    /** Start pose and robot-relative distances for the no-turn Garden transit. */
    private Pose gardenLegStart;
    private double gardenBackDistance,gardenRightDistance;
    /** Inward bypass used to keep the direct Garden-to-park path clear of the Flower. */
    private Pose parkVia;
    private double gardenIntakeHeading;
    private int alliance;
    private boolean running,tiltConfirmed;
    private double power,startAt,stateAt,lastNow,spinAt;
    private String message="",error="";
    public double spinUpSeconds=1,feedSeconds=3,parkingToleranceInches=2,approachPower=.25;
    /** Measured forward stroke, not an assumed pickup count. Calibrate on the robot. */
    public double gardenAdvanceInches=18,gardenAdvanceSeconds=5,gardenAdvancePower=.18;
    public double gardenFirstPassSeconds=2,gardenIntakeHeadingRadians=Math.PI/2;
    /** Front-mounted camera: turn after wall clearance to view the Hive behind the shooter. */
    public double observationHeadingOffsetRadians=Math.PI;
    public boolean surveyBeforePickup=false;
    /** Practice only: allow a live tag frame without official Hive geometry. */
    public boolean allowAnyFreshAprilTag=false;
    /** Team 1 begins at a fixed, taped preload-shot pose; vision is not required to feed. */
    public boolean requireAprilTagForPreload=false;
    /** Optional old spilled-pollen test; normal mode uses the Garden. */
    public boolean recoverPollenAfterTilt=false;
    private boolean survey;
    private double surveyAngle,surveyHeading,bestHeading,bestArea;
    private double avoidUntil,avoidStrafe;

    public TeamOnePollenAuto(RobotIO io,FieldLayout field){this.io=io;this.field=field;nav=new Navigator(field,io::stopRequested);}
    public void start(int alliance,double power){
        if(alliance<0||alliance>1||!Double.isFinite(power)||power<0||power>1
                ||!Double.isFinite(spinUpSeconds)||spinUpSeconds<0||!Double.isFinite(feedSeconds)||feedSeconds<=0
                ||!Double.isFinite(observationHeadingOffsetRadians)||!Double.isFinite(parkingToleranceInches)||parkingToleranceInches<=0||parkingToleranceInches>3
                ||!Double.isFinite(approachPower)||approachPower<=0||approachPower>.5
                ||!Double.isFinite(gardenAdvanceInches)||gardenAdvanceInches<=0||gardenAdvanceInches>24
                ||!Double.isFinite(gardenAdvanceSeconds)||gardenAdvanceSeconds<=0||gardenAdvanceSeconds>8
                ||!Double.isFinite(gardenAdvancePower)||gardenAdvancePower<=0||gardenAdvancePower>.3
                ||!Double.isFinite(gardenFirstPassSeconds)||gardenFirstPassSeconds<=0||gardenFirstPassSeconds>5
                ||!Double.isFinite(gardenIntakeHeadingRadians))
            throw new IllegalArgumentException("Invalid Team One configuration");
        this.alliance=alliance;this.power=power;startAt=lastNow=io.nowSeconds();spinAt=Double.NaN;
        running=true;tiltConfirmed=survey=false;avoidUntil=0;current=destination=gardenEntry=parkVia=gardenLegStart=null;gardenBackDistance=gardenRightDistance=0;gardenIntakeHeading=Double.NaN;message=error="";
        collecting=lastFeeding=false;feedElapsed=0;lastCommandAt=startAt;
        shotGate=new HiveShotGate(field,alliance,false);
        hive=new HiveVision(field,alliance);nav.useScriptedRoute(true,alliance);enter(State.SHOOTING);io.apply(new Outputs());
    }
    private void enter(State next){
        state=next;stateAt=io.nowSeconds();nav.reset();
        if(next==State.TO_GARDEN)collecting=true;
        // The robot shoots with its rear toward the Hive.  When it reaches the
        // Garden, use the measured arrival heading as the intake heading rather
        // than commanding a field-fixed 90 degree turn.  This keeps the direct
        // diagonal trip and every retry pass aligned with the chassis direction
        // that actually brought it to the Garden.
        if(next==State.GARDEN_ALIGN_HEADING)
            gardenIntakeHeading=current!=null&&current.finite()
                    ? Pose.angle(current.heading) : Pose.angle(gardenIntakeHeadingRadians);
        if(next==State.TO_GARDEN_BACK||next==State.TO_GARDEN_RIGHT)gardenLegStart=current;
        if(next==State.INTAKE_GARDEN)gardenEntry=current;
        if(next==State.PARK||next==State.STOPPED||next==State.ERROR)collecting=false;
        if(next==State.SEARCH&&surveyBeforePickup&&current!=null){survey=true;surveyAngle=0;surveyHeading=current.heading;bestHeading=Double.NaN;bestArea=0;}
    }
    public void stop(){running=false;state=State.STOPPED;io.apply(new Outputs());}
    public boolean isRunning(){return running;}
    public State getState(){return state;}
    public String lastError(){return error;}
    public String status(){return state+String.format(java.util.Locale.ROOT," · %.1f / 30 s · ",io.nowSeconds()-startAt)+message
            +(recoverPollenAfterTilt?(tiltConfirmed?" · Hive tilt confirmed":" · Tilt not yet confirmed"):" · Garden collection mode")+" · park by 26 s; stop at 30 s";}
    public Pose destination(){return running&&(state==State.PARK||moving())?destination:null;}
    public List<Pose> route(){return nav.route();}
    private boolean moving(){return state==State.TO_GARDEN||state==State.TO_GARDEN_BACK||state==State.TO_GARDEN_RIGHT||state==State.OBSERVE_TILT||state==State.CLEAR_PICKUP||state==State.REPOSITION;}
    private boolean gardenMotion(){return state==State.TO_GARDEN||state==State.TO_GARDEN_BACK||state==State.TO_GARDEN_RIGHT||state==State.INTAKE_GARDEN||state==State.GARDEN_BACK_UP||state==State.GARDEN_RIGHT_SHIFT||state==State.GARDEN_RESTORE_HEADING||state==State.GARDEN_FORWARD_RETRY;}
    /**
     * Split the old diagonal translation into a backward leg and then a right
     * strafe.  The magnitudes are the same robot-relative components that the
     * diagonal command used, but neither leg ever asks the drivetrain to turn.
     */
    private void planGardenLegs(){
        double dx=destination.x-current.x,dy=destination.y-current.y;
        double c=Math.cos(current.heading),s=Math.sin(current.heading);
        gardenBackDistance=Math.abs(dx*c+dy*s);
        gardenRightDistance=Math.abs(dx*s-dy*c);
    }
    private boolean gardenLegDone(double targetInches){
        // Eight inches per second is deliberately conservative.  The timeout
        // guarantees that a momentary odometry dropout cannot strand the robot
        // before it starts the intake routine.
        return gardenLegStart!=null&&current!=null
                &&current.distance(gardenLegStart)>=Math.max(0,targetInches-1)
                || io.nowSeconds()-stateAt>=Math.max(.45,targetInches/8.0+.55);
    }
    private void park(){
        Pose location=field.teamOneStartSidePark[alliance];
        destination=location;
        // Move inward before travelling up the alliance lane. This keeps the
        // Garden-to-park diagonal outside the nearby Flower footprint.
        parkVia=new Pose(alliance==0?-38:38,alliance==0?-16:16,current.heading);
        enter(State.PARK);message="Start-side-half parking via Flower-clear bypass; collection OFF";
    }
    private void fail(String reason){error=message=reason;running=false;state=State.ERROR;}
    private int pipeline(){return state==State.TO_GARDEN||state==State.TO_GARDEN_BACK||state==State.TO_GARDEN_RIGHT||state==State.INTAKE_GARDEN||state==State.GARDEN_BACK_UP||state==State.GARDEN_RIGHT_SHIFT||state==State.GARDEN_RESTORE_HEADING||state==State.GARDEN_FORWARD_RETRY||state==State.SEARCH||state==State.APPROACH||state==State.CLOSE_PICKUP||state==State.REPOSITION?1:0;}
    private Target pollen(Vision vision,double now){
        if(!vision.fresh(now,1))return null;
        Target best=null;
        for(Target t:vision.targets)if(t.id<0&&"yellow".equals(t.color)&&Double.isFinite(t.tx)&&Double.isFinite(t.areaPercent)&&t.areaPercent>0
                &&(best==null||t.areaPercent>best.areaPercent))best=t;
        return best;
    }
    private boolean roomToScan(Pose p){
        for(int i=0;i<24;i++)if(!field.allowed(new Pose(p.x,p.y,i*Math.PI/12)))return false;
        return true;
    }
    private void scanOrReposition(){
        if(roomToScan(current)){enter(State.SEARCH);return;}
        for(int radius=4;radius<=16;radius+=4)for(int i=0;i<8;i++){
            Pose q=new Pose(current.x+radius*Math.cos(i*Math.PI/4),current.y+radius*Math.sin(i*Math.PI/4),current.heading);
            if(roomToScan(q)&&field.clear(current,q,current.heading)){destination=q;enter(State.REPOSITION);return;}
        }
        message="No safe scanning clearance; waiting for parking";enter(State.WAITING);
    }
    /** Imported image-angle/area approach, with bounded cruise power and camera steering. */
    private Outputs approach(Target t){
        Outputs out=new Outputs();out.pipeline=1;out.intakePower=1;
        out.forward=approachPower;out.strafeRight=Math.abs(t.tx)<=2?0:FieldLayout.clip(t.tx*.008,.12);
        out.turnClockwise=FieldLayout.clip(t.tx*.018,.35);
        if(Math.abs(t.tx)>25){out.forward=0;out.turnClockwise=FieldLayout.clip(t.tx*.02,.25);}
        return guard(out);
    }
    private Outputs guard(Outputs out){
        if(state==State.APPROACH&&io.nowSeconds()<avoidUntil){out.forward=out.turnClockwise=0;out.strafeRight=avoidStrafe;}
        double dt=.12,h=Pose.angle(current.heading-out.turnClockwise*1.65*dt);
        Pose next=new Pose(current.x+(out.forward*Math.cos(h)+out.strafeRight*Math.sin(h))*25*dt,
                current.y+(out.forward*Math.sin(h)-out.strafeRight*Math.cos(h))*25*dt,h);
        if(!field.canRotate(current,h)||!field.clear(current,next,h)){
            if(state==State.APPROACH&&out.forward>0){
                for(double side:new double[]{-.3,.3}){
                    Pose lateral=new Pose(current.x+Math.sin(current.heading)*side*25*.75,current.y-Math.cos(current.heading)*side*25*.75,current.heading);
                    Pose ahead=new Pose(lateral.x+Math.cos(current.heading)*3,lateral.y+Math.sin(current.heading)*3,current.heading);
                    if(field.clear(current,lateral,current.heading)&&field.clear(lateral,ahead,current.heading)){
                        avoidUntil=io.nowSeconds()+.75;avoidStrafe=side;out.forward=out.turnClockwise=0;out.strafeRight=side;
                        message="Visible pollen; short collision-checked frame clearance";return out;
                    }
                }
            }
            out.forward=out.strafeRight=out.turnClockwise=out.intakePower=0;
            message="Camera approach blocked; holding rather than driving through obstacle";
            if(io.nowSeconds()-stateAt>1){enter(State.WAITING);}
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
            // Once the front is in the Garden, keep the commanded straight intake
            // stroke alive even if Pinpoint briefly drops a sample. This phase has no
            // steering, turns, or odometry-based stop; four measured pieces or the
            // 26-second park deadline ends it.
            boolean keepDriving=gardenMotion()||state==State.PARK;
            // Team 1 starts at the taped shot pose. A transient Pinpoint update
            // loss must not abort a stationary, timed preload feed; departure to
            // the Garden still waits until at least one usable pose is available.
            boolean stationaryShot=state==State.SHOOTING;
            boolean usablePose=s.pose!=null&&s.pose.finite()&&now-s.sampledAt>=0&&now-s.sampledAt<=.25;
            if(!keepDriving&&!stationaryShot&&(!s.odometryValid||!usablePose)){fail("Odometry unavailable or stale");io.apply(out);return;}
            if(usablePose)current=s.pose;
            else if(current==null&&!stationaryShot){fail("Odometry unavailable before a usable pose was recorded");io.apply(out);return;}
            if(lastFeeding)feedElapsed+=Math.min(.10,Math.max(0,now-lastCommandAt));
            lastFeeding=false;lastCommandAt=now;
            if(s.ballCount>=4)collecting=false;
            double elapsed=now-startAt;
            if(elapsed>=26&&state!=State.PARK)park();
            // Start earlier only when a distant search would consume the return-time reserve.
            if((state==State.TO_GARDEN||state==State.SEARCH||state==State.APPROACH||state==State.CLOSE_PICKUP||state==State.REPOSITION)
                    &&elapsed+current.distance(field.teamOneStartSidePark[alliance])*1.6/Math.max(1,25*field.drivePower)+1.2>=29)park();
            if((state==State.CLEAR_PICKUP||state==State.REPOSITION||state==State.SEARCH||state==State.APPROACH||state==State.CLOSE_PICKUP)&&!s.cameraConnected){
                message="Camera disconnected; recovery outputs OFF";out.pipeline=pipeline();io.apply(out);return;
            }
            if(pipeline()==0)hive.update(s.vision,now);
            State before=state;
            double age=now-stateAt;
            switch(state){
                case SHOOTING:
                    if(feedElapsed>=feedSeconds){
                        if(current==null){message="Preloads complete; waiting for a valid Pinpoint pose before Garden departure";break;}
                        if(!recoverPollenAfterTilt){
                            Pose g=field.garden[alliance];
                            // Keep the shot heading on the trip. Team 1 aligns to its
                            // intake heading only after it reaches the Garden.
                            destination=new Pose(g.x-18*Math.cos(g.heading),g.y-18*Math.sin(g.heading),current.heading);
                            enter(State.TO_GARDEN);message="Preloads complete; go straight to alliance Garden without a departure turn";break;
                        }
                        Pose start=field.hive[alliance];destination=new Pose(start.x,start.y-18*Math.sin(start.heading),Pose.angle(start.heading+observationHeadingOffsetRadians));
                        enter(State.OBSERVE_TILT);message="Preload feed attempt complete; move to tag-observation clearance";
                    }else{
                        Pose shot=field.hive[alliance];
                        boolean atShot=current.distance(shot)<=1&&Math.abs(Pose.angle(current.heading-shot.heading))<=Math.toRadians(2);
                        boolean practiceTagReady=s.cameraConnected&&s.vision.fresh(now,0)&&s.vision.targets.length>0;
                        boolean tagReady=allowAnyFreshAprilTag?practiceTagReady:(atShot&&shotGate.ready(s,now));
                        if(requireAprilTagForPreload&&!tagReady){
                            spinAt=Double.NaN;
                            message=allowAnyFreshAprilTag?"Shooting blocked: need any fresh AprilTag frame on pipeline 1":"Shooting blocked: need fresh centered raised own-Hive tags at start pose";
                            break;
                        }
                        if(Double.isNaN(spinAt))spinAt=now;
                        out.shooterPower=power;if(now-spinAt>=spinUpSeconds)out.feederPower=1;
                        message=requireAprilTagForPreload?(allowAnyFreshAprilTag?"Practice tag detected; paced preload feed":"Own raised Cell confirmed; paced preload feed"):"Fixed Team 1 start pose; paced preload feed";
                    }
                    break;
                case TO_GARDEN:
                    planGardenLegs();
                    enter(State.TO_GARDEN_BACK);
                    message="Garden transit: drive backward first, without turning";
                    break;
                case TO_GARDEN_BACK:
                    // The robot begins with its rear at the Hive.  Backing away
                    // is the first safe, non-turning leg toward the Garden.
                    out.forward=-.35;
                    if(gardenLegDone(gardenBackDistance)){
                        enter(State.TO_GARDEN_RIGHT);
                        message="Garden transit: backward leg complete; strafe right without turning";
                    }else message="Garden transit: backing up; hold heading";
                    break;
                case TO_GARDEN_RIGHT:
                    out.strafeRight=.35;
                    if(gardenLegDone(gardenRightDistance)){
                        enter(State.GARDEN_ALIGN_HEADING);
                        message="Garden reached; preserve arrival heading before intake";
                    }else message="Garden transit: strafing right; hold heading";
                    break;
                case GARDEN_ALIGN_HEADING:
                    double intakeHeadingError=Pose.angle(gardenIntakeHeading-current.heading);
                    if(Math.abs(intakeHeadingError)<=Math.toRadians(2)){enter(State.INTAKE_GARDEN);message="Garden arrival heading held; begin repeated forward intake passes";}
                    else out.turnClockwise=-FieldLayout.clip(Math.toDegrees(intakeHeadingError)*.05,.25);
                    break;
                case INTAKE_GARDEN:
                    if(s.ballCount>=4){enter(State.WAITING);message="Measured inventory full; wait for parking";break;}
                    if(age>=gardenFirstPassSeconds){
                        enter(State.GARDEN_BACK_UP);
                        message="Garden intake pass complete; back up slightly before the next pass";
                        break;
                    }
                    Target gardenBall=pollen(s.vision,now);
                    out.forward=gardenAdvancePower;out.intakePower=1;
                    if(gardenBall!=null){
                        out.strafeRight=Math.abs(gardenBall.tx)>6?FieldLayout.clip(gardenBall.tx*.012,.12):0;
                    }
                    // On the real robot, keep driving the intake forward until it has
                    // actually measured four pieces or the 26-second park command
                    // takes over. Do not use camera, simulated-clearance, distance,
                    // or timer cutoffs for this straight, no-turn collection phase.
                    double headingError=Double.isFinite(gardenIntakeHeading)?Pose.angle(gardenIntakeHeading-current.heading):0;
                    out.turnClockwise=-FieldLayout.clip(Math.toDegrees(headingError)*.05,.22);
                    message="Garden intake: hold first-pass heading; drive until four pieces or 26-second parking";
                    break;
                case GARDEN_BACK_UP:
                    out.forward=-.25;out.intakePower=1;
                    if(age>=.45){enter(State.GARDEN_RIGHT_SHIFT);message="Garden retry: shift right slightly";}
                    break;
                case GARDEN_RIGHT_SHIFT:
                    out.strafeRight=.25;out.intakePower=1;
                    if(age>=.45){enter(State.GARDEN_RESTORE_HEADING);message="Garden retry: restore first-pass heading";}
                    break;
                case GARDEN_RESTORE_HEADING:
                    double restoreError=Double.isFinite(gardenIntakeHeading)?Pose.angle(gardenIntakeHeading-current.heading):0;
                    if(Math.abs(restoreError)<=Math.toRadians(2)){enter(State.GARDEN_FORWARD_RETRY);message="Garden retry: original heading restored; drive into next pollen pair";}
                    else if(age>=2){enter(State.GARDEN_FORWARD_RETRY);message="Garden retry: heading-restore timeout; continue carefully";}
                    else out.turnClockwise=-FieldLayout.clip(Math.toDegrees(restoreError)*.05,.25);
                    break;
                case GARDEN_FORWARD_RETRY:
                    out.forward=gardenAdvancePower;out.intakePower=1;
                    Target retryBall=pollen(s.vision,now);
                    if(retryBall!=null)out.strafeRight=Math.abs(retryBall.tx)>6?FieldLayout.clip(retryBall.tx*.012,.12):0;
                    double retryHeadingError=Double.isFinite(gardenIntakeHeading)?Pose.angle(gardenIntakeHeading-current.heading):0;
                    out.turnClockwise=-FieldLayout.clip(Math.toDegrees(retryHeadingError)*.05,.22);
                    if(age>=.60){enter(State.INTAKE_GARDEN);message="Garden retry complete; repeat forward intake cycle until four pieces or parking";}
                    break;
                case OBSERVE_TILT:
                case CLEAR_PICKUP:
                case REPOSITION:
                    out=nav.move(current,destination);
                    if(out==null){
                        out=new Outputs();
                        if(state==State.OBSERVE_TILT){enter(State.WAIT_TILT);message="Observe own Hive: require stable opposite Cell up";}
                        else scanOrReposition();
                    }
                    break;
                case WAIT_TILT:
                    message=s.cameraConnected?"Await fresh, calibrated own-Hive tilt evidence":"Limelight unavailable; wait safely for parking";
                    if(s.cameraConnected&&hive.state(now)==1){
                        tiltConfirmed=true;destination=new Pose(current.x,current.y-8*Math.signum(field.hive[alliance].y),current.heading);
                        enter(State.CLEAR_PICKUP);message="Hive tilt confirmed; back off for front-camera pollen view";
                    }
                    break;
                case SEARCH:
                case APPROACH:
                case CLOSE_PICKUP:
                    if(s.ballCount>=4){enter(State.WAITING);message="Measured inventory full; wait for parking";break;}
                    if(!s.cameraConnected){enter(State.SEARCH);message="Camera disconnected; drive/intake OFF";break;}
                    if(state==State.SEARCH&&survey){
                        if(!roomToScan(current)){scanOrReposition();break;}
                        surveyAngle+=Math.abs(Pose.angle(current.heading-surveyHeading));surveyHeading=current.heading;
                        Target seen=pollen(s.vision,now);
                        if(seen!=null&&seen.areaPercent>bestArea){bestArea=seen.areaPercent;bestHeading=Pose.angle(current.heading-Math.toRadians(seen.tx));}
                        if(surveyAngle<2*Math.PI-.08){message="Survey all camera bearings for the closest-looking pollen";out.turnClockwise=.65;out=guard(out);break;}
                        if(Double.isFinite(bestHeading)){
                            double error=Pose.angle(bestHeading-current.heading);
                            if(Math.abs(error)>Math.toRadians(4)){message="Reacquire best observed pollen bearing";out.turnClockwise=-FieldLayout.clip(Math.toDegrees(error)*.025,.65);out=guard(out);break;}
                        }
                        survey=false;stateAt=now;message="Survey complete; require a fresh visible pollen target";break;
                    }
                    Target target=pollen(s.vision,now);
                    if(target==null){
                        enter(State.WAITING);message="No fresh pollen target; hold still until parking";
                    }else if(state==State.CLOSE_PICKUP){
                        out.intakePower=1;
                        if(age>=.7){
                            Pose q=new Pose(current.x-4*Math.cos(current.heading),current.y-4*Math.sin(current.heading),current.heading);
                            if(field.clear(current,q,current.heading)){destination=q;enter(State.REPOSITION);}
                            else{enter(State.WAITING);message="Close pickup attempt complete; no safe retreat";}
                        }
                    }else if(target.areaPercent>=12){enter(State.CLOSE_PICKUP);message="Pollen close; bounded intake attempt (count remains unknown)";}
                    else{if(state==State.SEARCH)enter(State.APPROACH);message="Live largest-pollen camera approach";out=approach(target);}
                    break;
                case WAITING:break;
                case PARK:
                    Pose parkingGoal=parkVia==null?destination:parkVia;
                    if(parkVia!=null&&current.distance(parkVia)<=3){parkVia=null;out=new Outputs();message="Flower-clear bypass reached; continue to configured parking target";break;}
                    double parkHeadingError=Pose.angle(parkingGoal.heading-current.heading);
                    if(parkVia==null&&current.distance(destination)<=parkingToleranceInches&&Math.abs(parkHeadingError)<=Math.toRadians(3)){running=false;message="Parked at configured location and heading: all outputs OFF";break;}
                    // Garden collection may have carried the robot outside of the
                    // simulator's modeled corridor. At the 26-second deadline use a
                    // simple field-vector drive toward parking instead of invoking
                    // the collision-route planner, which can reject that pose and
                    // throw a runtime exception. Correct toward the configured
                    // parking heading before the final few inches.
                    double dx=parkingGoal.x-current.x,dy=parkingGoal.y-current.y;
                    double c=Math.cos(current.heading),sine=Math.sin(current.heading);
                    if(parkVia==null&&current.distance(destination)<=4&&Math.abs(parkHeadingError)>Math.toRadians(3)){
                        out.forward=out.strafeRight=0;
                    }else{
                        out.forward=FieldLayout.clip((dx*c+dy*sine)*.08,.45);
                        out.strafeRight=FieldLayout.clip((dx*sine-dy*c)*.08,.45);
                    }
                    out.turnClockwise=-FieldLayout.clip(Math.toDegrees(parkHeadingError)*.05,.25);
                    message=parkVia==null?"Parking direct to configured location and heading":"Parking via Flower-clear bypass";
                    break;
                default:break;
            }
            if(state!=before||!running)out=new Outputs();
            out.intakePower=running&&collecting&&s.ballCount<4&&state!=State.PARK?1:0;
            lastFeeding=out.feederPower>0;
            out.pipeline=pipeline();
            if(io.stopRequested()){stop();return;}
            io.apply(out);
        }catch(java.util.concurrent.CancellationException e){stop();}
        catch(RuntimeException e){
            if(gardenMotion()||state==State.PARK){message="Transient Garden/parking fault; outputs held this loop and retrying";io.apply(new Outputs());}
            else{fail("Team One stopped safely: "+e.getMessage());io.apply(new Outputs());}
        }
    }
}
