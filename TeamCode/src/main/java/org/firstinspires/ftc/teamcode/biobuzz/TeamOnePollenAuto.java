package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.List;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Team One: paced preloads, camera-guided Garden pickup, parking. */
public final class TeamOnePollenAuto implements RobotController {
    public enum State { SHOOTING, POST_SHOT_TAG_WAIT, TO_GARDEN, TO_GARDEN_BACK, TO_GARDEN_RIGHT, GARDEN_ALIGN_HEADING, INTAKE_GARDEN, GARDEN_BACK_UP, GARDEN_RIGHT_SHIFT, GARDEN_RESTORE_HEADING, GARDEN_FORWARD_RETRY, TO_TEAM_TWO_SHOT, RETURN_TO_START_SHOT, WAIT_FOR_TEAM_TWO_TAG, FOLLOW_UP_SHOOTING, OBSERVE_TILT, WAIT_TILT, CLEAR_PICKUP, SEARCH, APPROACH, CLOSE_PICKUP, REPOSITION, WAITING, PARK, STOPPED, ERROR }
    private final RobotIO io;
    private final FieldLayout field;
    private final Navigator nav;
    private final ShootingAim.Alignment shootingAlignment=new ShootingAim.Alignment();
    private HiveVision hive;
    private HiveShotGate shotGate;
    private boolean collecting,lastFeeding,preloadTagLatched;
    private double feedElapsed,lastCommandAt;
    private State state=State.STOPPED;
    private Pose current,destination;
    private Pose gardenEntry;
    /** Start pose and robot-relative distances for the no-turn Garden transit. */
    private Pose gardenLegStart;
    private double gardenBackDistance,gardenRightDistance;
    /** Inward bypass used to keep the direct Garden-to-park path clear of the Flower. */
    private Pose parkVia;
    /** Backup-only three-leg exit from the Team Two wall-shot pose. */
    private Pose parkRetreat,parkSide;
    /** Alliance-wall corridor waypoint for the backup Garden-to-Team-Two shot route. */
    private Pose teamTwoShotVia;
    /** Initial sideways Garden exit before travelling the outside lane. */
    private Pose teamTwoShotExit;
    /** Shooting-lane stop between the Flower-safe waypoint and the wall shot. */
    private Pose teamTwoShotLane;
    private double gardenIntakeHeading;
    private int alliance;
    private boolean running,tiltConfirmed;
    private double power,startAt,stateAt,lastNow,spinAt;
    private String message="",error="";
    public double spinUpSeconds=1,feedSeconds=3,parkingToleranceInches=2,approachPower=.25;
    /** Measured forward stroke, not an assumed pickup count. Calibrate on the robot. */
    public double gardenAdvanceInches=18,gardenAdvanceSeconds=5,gardenAdvancePower=.18;
    /** Zero keeps legacy timed strokes; a positive value ends each stroke at this measured distance. */
    public double gardenPassDistanceInches=0,gardenTransitPower=.35,gardenRepositionPower=.25;
    public double gardenFirstPassSeconds=2,gardenRepositionSeconds=.45,gardenStagingOffsetInches=18,gardenIntakeHeadingRadians=Math.PI/2;
    /** Front-mounted camera: turn after wall clearance to view the Hive behind the shooter. */
    public double observationHeadingOffsetRadians=Math.PI;
    public boolean surveyBeforePickup=false;
    /** Practice only: allow a live tag frame without official Hive geometry. */
    public boolean allowAnyFreshAprilTag=false;
    /** Team 1 begins at a fixed, taped preload-shot pose; vision is not required to feed. */
    public boolean requireAprilTagForPreload=false;
    public boolean aimForShooting=false;
    public boolean colorPickupEnabled=false,collectBlue=true,collectRed=false;
    /** Normal robot mode: fresh yellow absence during match seconds 10-18 ends Garden pickup. */
    public boolean gardenYellowWindowEnabled=false;
    public double translationPowerMultiplier=1;
    private boolean leaveAuthorized;
    /** Backup mode: after two Garden passes, use Team Two's taped wall shot and fire only on a live tag. */
    public boolean useTeamTwoFollowUpShot=false;
    public int gardenPassLimit=0;
    /** Normal Team 1 branch when the first shot did not tilt the Hive. */
    private boolean repeatStartShotAfterGarden,postShotDecisionMade,postShotMissingTagFrame;
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
                ||!Double.isFinite(gardenAdvancePower)||gardenAdvancePower<=0||gardenAdvancePower>.6
                ||!Double.isFinite(gardenPassDistanceInches)||gardenPassDistanceInches<0||gardenPassDistanceInches>24
                ||!Double.isFinite(gardenTransitPower)||gardenTransitPower<=0||gardenTransitPower>.7
                ||!Double.isFinite(gardenRepositionPower)||gardenRepositionPower<=0||gardenRepositionPower>.6
                ||!Double.isFinite(gardenFirstPassSeconds)||gardenFirstPassSeconds<=0||gardenFirstPassSeconds>5
                ||!Double.isFinite(gardenRepositionSeconds)||gardenRepositionSeconds<=0||gardenRepositionSeconds>2
                ||!Double.isFinite(gardenStagingOffsetInches)||gardenStagingOffsetInches<0||gardenStagingOffsetInches>24
                ||!Double.isFinite(gardenIntakeHeadingRadians)||gardenPassLimit<0||gardenPassLimit>8
                ||!Double.isFinite(translationPowerMultiplier)||translationPowerMultiplier<=0||translationPowerMultiplier>1.25)
            throw new IllegalArgumentException("Invalid Team One configuration");
        this.alliance=alliance;this.power=power;startAt=lastNow=io.nowSeconds();spinAt=Double.NaN;
        shootingAlignment.reset();
        running=true;tiltConfirmed=survey=false;avoidUntil=0;current=destination=gardenEntry=parkVia=teamTwoShotExit=teamTwoShotVia=teamTwoShotLane=gardenLegStart=null;gardenBackDistance=gardenRightDistance=0;gardenIntakeHeading=Double.NaN;message=error="";
        collecting=lastFeeding=false;feedElapsed=0;lastCommandAt=startAt;
        leaveAuthorized=false;
        shotGate=new HiveShotGate(field,alliance,false);
        hive=new HiveVision(field,alliance);nav.useScriptedRoute(true,alliance);enter(State.SHOOTING);io.apply(new Outputs());
    }
    private void enter(State next){
        state=next;stateAt=io.nowSeconds();nav.reset();
        // The backup's second wall shot is a new shot sequence. Require one
        // fresh authorization there, then keep that authorization for the feed.
        if(next==State.WAIT_FOR_TEAM_TWO_TAG)shootingAlignment.reset();
        if(next==State.TO_GARDEN)collecting=true;
        if(next==State.POST_SHOT_TAG_WAIT)postShotMissingTagFrame=false;
        // The robot shoots with its rear toward the Hive.  When it reaches the
        // Garden, use the measured arrival heading as the intake heading rather
        // than commanding a field-fixed 90 degree turn.  This keeps the direct
        // diagonal trip and every retry pass aligned with the chassis direction
        // that actually brought it to the Garden.
        if(next==State.GARDEN_ALIGN_HEADING)
            gardenIntakeHeading=current!=null&&current.finite()
                    ? Pose.angle(current.heading) : Pose.angle(gardenIntakeHeadingRadians);
        if(next==State.TO_GARDEN_BACK||next==State.TO_GARDEN_RIGHT)gardenLegStart=current;
        if(next==State.INTAKE_GARDEN||next==State.GARDEN_FORWARD_RETRY)gardenEntry=current;
        if(next==State.SHOOTING||next==State.FOLLOW_UP_SHOOTING){spinAt=Double.NaN;feedElapsed=0;lastFeeding=false;}
        if(next==State.PARK||next==State.STOPPED||next==State.ERROR)collecting=false;
        if(next==State.SEARCH&&surveyBeforePickup&&current!=null){survey=true;surveyAngle=0;surveyHeading=current.heading;bestHeading=Double.NaN;bestArea=0;}
    }
    public void stop(){running=false;state=State.STOPPED;io.apply(new Outputs());}
    public boolean isRunning(){return running;}
    public State getState(){return state;}
    public boolean colorPickupActive(){return running&&colorPickupEnabled
            &&(state==State.INTAKE_GARDEN||state==State.GARDEN_FORWARD_RETRY);}
    public String lastError(){return error;}
    public boolean leaveAuthorized(){return leaveAuthorized;}
    private boolean gardenYellowWindowActive(double elapsed){return gardenYellowWindowEnabled&&!useTeamTwoFollowUpShot
            &&elapsed>=10&&elapsed<=18&&(state==State.INTAKE_GARDEN||state==State.GARDEN_BACK_UP
            ||state==State.GARDEN_RIGHT_SHIFT||state==State.GARDEN_RESTORE_HEADING||state==State.GARDEN_FORWARD_RETRY);}
    public String status(){return state+String.format(java.util.Locale.ROOT," · %.1f / 30 s · ",io.nowSeconds()-startAt)+message
            +(recoverPollenAfterTilt?(tiltConfirmed?" · Hive tilt confirmed":" · Tilt not yet confirmed"):" · Garden collection mode")+" · normal Team 1 parks near 29-30 s; stop at 30 s";}
    public Pose destination(){return running&&(state==State.PARK||moving())?destination:null;}
    public List<Pose> route(){return nav.route();}
    private boolean moving(){return state==State.TO_GARDEN||state==State.TO_GARDEN_BACK||state==State.TO_GARDEN_RIGHT||state==State.TO_TEAM_TWO_SHOT||state==State.RETURN_TO_START_SHOT||state==State.OBSERVE_TILT||state==State.CLEAR_PICKUP||state==State.REPOSITION;}
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
    private boolean fixedGardenPasses(){return useTeamTwoFollowUpShot||repeatStartShotAfterGarden;}
    private boolean gardenPassDone(double age){
        // The Backup must make two equal, complete pushes into the Garden.
        // Pinpoint can report a short or sideways displacement after Flower/
        // Garden contact, so do not let that noisy distance end either pass
        // early.  Its fixed timed stroke is the intentional wall-contact
        // fallback for both passes.
        if(fixedGardenPasses())return age>=gardenFirstPassSeconds;
        if(gardenPassDistanceInches>0&&gardenEntry!=null&&current!=null
                &&current.distance(gardenEntry)>=gardenPassDistanceInches)return true;
        return age>=gardenFirstPassSeconds;
    }
    private void followUpShotOrPark(){
        if(!useTeamTwoFollowUpShot){park();return;}
        collecting=false;
        destination=field.teamTwoWallHive[alliance];
        // First step into the deliberately open gap between the Flower and
        // Hive, then travel that gap away from both field elements.  Keep
        // every leg axis-aligned: a diagonal from the Garden can sweep the
        // rear of the chassis through the Flower or the Hive post.
        double gapX=alliance==0?-42:42;
        teamTwoShotExit=new Pose(gapX,current.y,current.heading);
        teamTwoShotVia=new Pose(gapX,alliance==0?42:-42,current.heading);
        teamTwoShotLane=new Pose(field.teamTwoWallHive[alliance].x,alliance==0?42:-42,current.heading);
        enter(State.TO_TEAM_TWO_SHOT);
        message="Garden passes complete; enter Flower/Hive gap, line up with shot lane, then drive straight into Team Two pose";
    }
    private void beginGarden(){
        Pose g=field.garden[alliance];
        destination=new Pose(g.x-gardenStagingOffsetInches*Math.cos(g.heading),g.y-gardenStagingOffsetInches*Math.sin(g.heading),current.heading);
        enter(State.TO_GARDEN);
    }
    private void finishFixedGardenPasses(){
        if(useTeamTwoFollowUpShot){followUpShotOrPark();return;}
        if(repeatStartShotAfterGarden){
            destination=field.hive[alliance];
            enter(State.RETURN_TO_START_SHOT);
            message="Two Garden passes complete; return quickly to original taped shot pose";
            return;
        }
        park();
    }
    /**
     * Backup route: translate straight to Team Two's shot area without the
     * navigator's intermediate heading turns.  Only turn once after arriving,
     * so the robot cannot make small circles while crossing the field.
     */
    private Outputs directMoveToTeamTwoShot(){
        // Do not rotate immediately after the Garden: a captured pollen can
        // be dragged into the chassis and jam there.  Translate out with the
        // Garden heading held, then rotate only at the safe shooting-lane
        // staging point before the final straight drive into the wall pose.
        Pose target=teamTwoShotExit!=null?teamTwoShotExit:teamTwoShotVia!=null?teamTwoShotVia:teamTwoShotLane!=null?teamTwoShotLane:destination;
        double dx=target.x-current.x,dy=target.y-current.y;
        double distance=Math.hypot(dx,dy);
        // Do not accept an approximate shot area here. The Backup must match
        // Team Two's taped wall-shot pose before vision can authorize a feed;
        // otherwise the camera can see the opponent Hive and fire incorrectly.
        if(teamTwoShotExit==null&&teamTwoShotVia==null&&teamTwoShotLane==null&&distance<=1.0)return null;
        if(distance>2.0){
            double c=Math.cos(current.heading),s=Math.sin(current.heading);
            Outputs out=new Outputs();
            out.forward=FieldLayout.clip((dx*c+dy*s)*.075,.90);
            out.strafeRight=FieldLayout.clip((dx*s-dy*c)*.075,.90);
            double magnitude=Math.hypot(out.forward,out.strafeRight);
            if(magnitude>.90){out.forward*=.90/magnitude;out.strafeRight*=.90/magnitude;}
            message=teamTwoShotLane==null?"Final straight Team Two shot approach":"Backup transfer: hold Garden heading until safe shot-lane rotation";
            return out;
        }
        // Stop at the Garden side-exit and outside-lane points without turning.
        if(teamTwoShotExit!=null){teamTwoShotExit=null;return new Outputs();}
        if(teamTwoShotVia!=null){teamTwoShotVia=null;return new Outputs();}
        // At the shooting-lane stop, turn before the final approach. The
        // robot's rear shooter (and rear-facing Hive view) is then pointed at
        // the Hive for the entire final straight move into the shot pose.
        if(teamTwoShotLane!=null){
            double laneHeadingError=Pose.angle(destination.heading-current.heading);
            if(Math.abs(laneHeadingError)>Math.toRadians(2)){
                Outputs out=new Outputs();
                out.turnClockwise=-FieldLayout.clip(Math.toDegrees(laneHeadingError)*.045,.35);
                return out;
            }
            teamTwoShotLane=null;
            message="Rear shooter faces Hive; make final straight approach to Team Two shot pose";
            return new Outputs();
        }
        double headingError=Pose.angle(destination.heading-current.heading);
        if(Math.abs(headingError)>Math.toRadians(2)){
            Outputs out=new Outputs();
            out.turnClockwise=-FieldLayout.clip(Math.toDegrees(headingError)*.045,.35);
            return out;
        }
        return null;
    }
    /** Validate only this alliance's post-tilt Team Two Hive tags at its exact taped wall pose. */
    private boolean correctTeamTwoTag(Sensors s,double now){
        Pose shot=field.teamTwoWallHive[alliance];
        if(current==null||current.distance(shot)>1.0
                ||Math.abs(Pose.angle(current.heading-shot.heading))>Math.toRadians(2)
                ||!s.cameraConnected||!s.vision.fresh(now,0))return false;
        // Seeing five or more tags means both tipped Hive faces are in view.
        // The aim gate already allows that case, so do not reject the same
        // confirmed view here merely because the individual family map is
        // temporarily ambiguous.
        if(ShootingAim.bothHivesVisible(s.vision))return true;
        // The second Team One shot happens only after its own Hive's first
        // tilt. At this wall pose, the post-tilt Cell uses the far family for
        // the selected alliance. Never admit opponent near/far tag IDs.
        for(int id:field.farTags[alliance])
            for(Target target:s.vision.targets)if(target.id==id)return true;
        return false;
    }
    private void park(){
        if(useTeamTwoFollowUpShot){
            // A direct vector from the Team Two shot pose can cross the Hive
            // post or enter the parking area beside an already-parked alliance
            // robot. Use fixed no-turn legs: reverse clear of the Hive, strafe
            // through the Flower/Hive gap, line up outside the parking area,
            // then make one final straight entry into the selected half.
            double clearY=alliance==0?42:-42;
            double clearX=alliance==0?-42:42;
            Pose location=field.teamOneStartSidePark[alliance];
            destination=new Pose(location.x,location.y,current.heading);
            parkRetreat=new Pose(current.x,clearY,current.heading);
            parkSide=new Pose(clearX,clearY,current.heading);
            // Remain on the field-side X lane until the final parking entry.
            // This prevents the diagonal from clipping through the other half
            // of the Alliance Parking Area before reaching our destination.
            parkVia=new Pose(clearX,location.y,current.heading);
            enter(State.PARK);message="Backup park: clear Hive, use Flower/Hive gap, line up outside parking, then final entry";
            return;
        }
        // Normal Team 1 leaves the Garden above the side Flower. Cross inward
        // first, travel past the Flower in that clear lane, then turn outward
        // only at the final parking row.
        Pose location=field.teamOneStartSidePark[alliance];
        double clearX=alliance==0?-42:42;
        destination=new Pose(location.x,location.y,current.heading);
        parkRetreat=new Pose(clearX,current.y,current.heading);
        parkSide=new Pose(clearX,location.y,current.heading);
        parkVia=null;
        enter(State.PARK);message="Garden exit: cross inward, pass Flower in clear lane, then enter parking";
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
            // Both wall shots are stationary, timed camera-gated operations.
            // Once the backup has arrived at Team Two's wall shot area, a
            // transient Pinpoint stale sample must not abort its tag wait/feed
            // or keep it from reaching the normal parking fallback.
            boolean stationaryShot=state==State.SHOOTING||state==State.WAIT_FOR_TEAM_TWO_TAG||state==State.FOLLOW_UP_SHOOTING;
            boolean usablePose=s.pose!=null&&s.pose.finite()&&now-s.sampledAt>=0&&now-s.sampledAt<=.25;
            if(!keepDriving&&!stationaryShot&&(!s.odometryValid||!usablePose)){
                if(current==null){message="Odometry unavailable; holding until a valid pose is received";out.pipeline=pipeline();io.apply(out);return;}
                message="Odometry stale; holding safely and retrying this autonomous state";
                out.pipeline=pipeline();io.apply(out);return;
            }
            if(usablePose)current=s.pose;
            else if(current==null&&!stationaryShot){message="Odometry unavailable; holding until a valid pose is received";out.pipeline=pipeline();io.apply(out);return;}
            if(lastFeeding)feedElapsed+=Math.min(.10,Math.max(0,now-lastCommandAt));
            lastFeeding=false;lastCommandAt=now;
            if(s.ballCount>=4)collecting=false;
            double elapsed=now-startAt;
            if(elapsed>=(useTeamTwoFollowUpShot?26.0:26.5)&&state!=State.PARK)park();
            // Normal Team 1 uses a three-leg Flower-clear parking route. Keep
            // collecting through 23 seconds, then reserve about five seconds
            // for the faster route to finish at the final target around 28–29.
            if(!useTeamTwoFollowUpShot&&!repeatStartShotAfterGarden&&gardenMotion()&&elapsed>=26.5&&state!=State.PARK)park();
            // Start earlier only when a distant search would consume the return-time reserve.
            if((state==State.TO_GARDEN||state==State.SEARCH||state==State.APPROACH||state==State.CLOSE_PICKUP||state==State.REPOSITION)
                    &&elapsed+current.distance(field.teamOneStartSidePark[alliance])*1.6/Math.max(1,25*field.drivePower)+1.2>=29)park();
            if((state==State.CLEAR_PICKUP||state==State.REPOSITION||state==State.SEARCH||state==State.APPROACH||state==State.CLOSE_PICKUP)&&!s.cameraConnected){
                message="Camera disconnected; recovery outputs OFF";out.pipeline=pipeline();io.apply(out);return;
            }
            if(pipeline()==0)hive.update(s.vision,now);
            boolean yellowWindow=gardenYellowWindowActive(elapsed);
            if(yellowWindow&&s.ballCount<4){
                Vision v=s.vision;
                if(!s.cameraConnected||!v.frameAvailable||v.pipeline!=1||!Double.isFinite(v.capturedAt)||now-v.capturedAt<0||now-v.capturedAt>.25){
                    message="Garden yellow observation unavailable; hold until a fresh frame";
                    out.pipeline=1;io.apply(out);return;
                }
                if(pollen(v,now)==null){
                    leaveAuthorized=true;collecting=false;enter(State.WAITING);
                    message="Leave authorized: no yellow pollen; continue normal 26-second parking schedule";
                }else if(state!=State.INTAKE_GARDEN){enter(State.INTAKE_GARDEN);}
            }
            State before=state;
            double age=now-stateAt;
            switch(state){
                case SHOOTING:
                    if(feedElapsed>=feedSeconds){
                        if(current==null){message="Preloads complete; waiting for a valid Pinpoint pose before Garden departure";break;}
                        if(!recoverPollenAfterTilt){
                            if(repeatStartShotAfterGarden){park();message="Return six-second shot completed; park";break;}
                            if(!useTeamTwoFollowUpShot&&!postShotDecisionMade){
                                enter(State.POST_SHOT_TAG_WAIT);message="Opening six-second shot complete; hold still and check whether Hive tags remain visible";break;
                            }
                            beginGarden();message="Opening six-second shot complete; go straight to alliance Garden without a departure turn";break;
                        }
                        Pose start=field.hive[alliance];destination=new Pose(start.x,start.y-18*Math.sin(start.heading),Pose.angle(start.heading+observationHeadingOffsetRadians));
                        enter(State.OBSERVE_TILT);message="Preload feed attempt complete; move to tag-observation clearance";
                    }else{
                        // The opening preload shot is from a taped fixed pose.
                        // Do not turn in place while waiting for a Hive tilt;
                        // the backup still uses AprilTag aim at its later
                        // Team Two wall-shot position.
                        if(aimForShooting&&requireAprilTagForPreload&&!shootingAlignment.align(s,now,out)){
                            spinAt=Double.NaN;message="Shooting blocked: need a fresh centered Hive AprilTag";break;
                        }
                        Pose shot=field.hive[alliance];
                        boolean atShot=current.distance(shot)<=1&&Math.abs(Pose.angle(current.heading-shot.heading))<=Math.toRadians(2);
                        boolean practiceTagReady=s.cameraConnected&&s.vision.fresh(now,0)&&s.vision.targets.length>0;
                        boolean tagReady=allowAnyFreshAprilTag?practiceTagReady:(atShot&&shotGate.ready(s,now));
                        if(tagReady)preloadTagLatched=true;
                        if(requireAprilTagForPreload&&!preloadTagLatched){
                            spinAt=Double.NaN;
                            message=allowAnyFreshAprilTag?"Shooting blocked: need any fresh AprilTag frame on pipeline 1":"Shooting blocked: need fresh centered raised own-Hive tags at start pose";
                            break;
                        }
                        if(Double.isNaN(spinAt))spinAt=now;
                        out.shooterPower=power;if(now-spinAt>=spinUpSeconds)out.feederPower=1;
                        message=requireAprilTagForPreload?(allowAnyFreshAprilTag?"AprilTag authorization latched; paced preload feed":"Own raised Cell authorization latched; paced preload feed"):"Fixed Team 1 start pose; paced preload feed";
                    }
                    break;
                case POST_SHOT_TAG_WAIT:
                    // Hold at the original shot pose for the whole 4.5-second
                    // tilt-settle window. Only the final half-second decides:
                    // a single no-tag frame means the Hive has tilted; otherwise
                    // retain the no-tilt branch and make two Garden passes.
                    boolean freshTagFrame=s.cameraConnected&&s.vision.fresh(now,0)&&s.vision.targets.length>0;
                    if(age>=4.0&&!freshTagFrame)postShotMissingTagFrame=true;
                    if(age<4.5){
                        message=age<4.0?"Post-shot Hive check: waiting for tilt to settle at starting pose"
                                :postShotMissingTagFrame?"Final tag-check window: missing AprilTag frame recorded"
                                :"Final tag-check window: AprilTag still visible";
                        break;
                    }
                    postShotDecisionMade=true;
                    if(!postShotMissingTagFrame){
                        repeatStartShotAfterGarden=true;gardenPassLimit=2;beginGarden();
                        message="AprilTags stayed visible through final check; run two full Garden passes, then retry original shot pose";
                    }else{
                        beginGarden();message="Missing AprilTag frame in final check; tilt assumed, begin normal Garden intake until parking";
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
                    out.forward=-gardenTransitPower;
                    if(gardenLegDone(gardenBackDistance)){
                        enter(State.TO_GARDEN_RIGHT);
                        message="Garden transit: backward leg complete; strafe right without turning";
                    }else message="Garden transit: backing up; hold heading";
                    break;
                case TO_GARDEN_RIGHT:
                    out.strafeRight=gardenTransitPower;
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
                    // The contingency routine deliberately completes both
                    // physical strokes.  A noisy/full inventory reading must
                    // not cancel its second forward-and-intake pass.
                    if(s.ballCount>=4&&!useTeamTwoFollowUpShot){followUpShotOrPark();break;}
                    if(!yellowWindow&&gardenPassDone(age)){
                        if(useTeamTwoFollowUpShot&&gardenPassLimit<=1){followUpShotOrPark();break;}
                        enter(State.GARDEN_BACK_UP);
                        message="Garden intake pass complete; back up slightly before the next pass";
                        break;
                    }
                    out.forward=gardenAdvancePower;out.intakePower=1;
                    Target gardenBall=pollen(s.vision,now);
                    if(!fixedGardenPasses()&&gardenBall!=null)
                        out.strafeRight=Math.abs(gardenBall.tx)>6?FieldLayout.clip(gardenBall.tx*.012,.12):0;
                    }
                    // Preserve arrival heading while collecting; the yellow window
                    // above can end pickup without inventing an inventory count.
                    double headingError=Double.isFinite(gardenIntakeHeading)?Pose.angle(gardenIntakeHeading-current.heading):0;
                    out.turnClockwise=-FieldLayout.clip(Math.toDegrees(headingError)*.05,.22);
                    message=yellowWindow?"Leave authorized: false; yellow visible, continue forward Garden collection":"Garden intake: hold first-pass heading; drive until four pieces or 26-second parking";
                    break;
                case GARDEN_BACK_UP:
                    out.forward=-gardenRepositionPower;out.intakePower=1;
                    if(age>=gardenRepositionSeconds){enter(State.GARDEN_RIGHT_SHIFT);message="Garden retry: shift right slightly";}
                    break;
                case GARDEN_RIGHT_SHIFT:
                    out.strafeRight=gardenRepositionPower;out.intakePower=1;
                    if(age>=gardenRepositionSeconds){enter(State.GARDEN_RESTORE_HEADING);message="Garden retry: restore first-pass heading";}
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
                    if(!fixedGardenPasses()&&retryBall!=null)out.strafeRight=Math.abs(retryBall.tx)>6?FieldLayout.clip(retryBall.tx*.012,.12):0;
                    double retryHeadingError=Double.isFinite(gardenIntakeHeading)?Pose.angle(gardenIntakeHeading-current.heading):0;
                    if(!fixedGardenPasses())out.turnClockwise=-FieldLayout.clip(Math.toDegrees(retryHeadingError)*.05,.22);
                    if((fixedGardenPasses()?gardenPassDone(age):age>=.60)){
                        if(fixedGardenPasses()&&gardenPassLimit<=2)finishFixedGardenPasses();
                        else if(fixedGardenPasses()){enter(State.GARDEN_BACK_UP);message="Garden pass complete; reset for the next measured intake pass";}
                        else {enter(State.INTAKE_GARDEN);message="Garden retry complete; repeat forward intake cycle until four pieces or parking";}
                    }
                    break;
                case TO_TEAM_TWO_SHOT:
                    out=directMoveToTeamTwoShot();
                    if(out==null){enter(State.WAIT_FOR_TEAM_TWO_TAG);message="At Team Two wall shot; waiting for a fresh AprilTag before follow-up feed";}
                    break;
                case RETURN_TO_START_SHOT:
                    out=nav.move(current,destination);
                    if(out==null){enter(State.SHOOTING);message="Original taped shot pose reached; spin up and feed five pulses";}
                    else message="Return from Garden to original Team One shot pose";
                    break;
                case WAIT_FOR_TEAM_TWO_TAG:
                    if(aimForShooting&&!shootingAlignment.align(s,now,out)){
                        message="Aligning follow-up shot: need a fresh centered Hive AprilTag";break;
                    }
                    if(correctTeamTwoTag(s,now)){
                        enter(State.FOLLOW_UP_SHOOTING);
                        message="Correct Hive AprilTag detected; begin paced follow-up feed";
                    }else message="At Team Two wall shot; waiting for a current Hive AprilTag";
                    break;
                case FOLLOW_UP_SHOOTING:
                    if(feedElapsed>=feedSeconds){park();break;}
                    if(aimForShooting&&!shootingAlignment.align(s,now,out)){
                        spinAt=Double.NaN;message="Follow-up feed paused: waiting for AprilTag aim to return";break;
                    }
                    if(Double.isNaN(spinAt))spinAt=now;
                    out.shooterPower=power;
                    if(now-spinAt>=spinUpSeconds)out.feederPower=1;
                    message="AprilTag confirmed at shot start; paced Team Two-position follow-up feed";
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
                    Pose parkingGoal=parkRetreat!=null?parkRetreat:parkSide!=null?parkSide:parkVia==null?destination:parkVia;
                    if(parkRetreat!=null&&current.distance(parkRetreat)<=3){
                        parkRetreat=null;out=new Outputs();message="Parking clearance reached; continue through safe lane";break;
                    }
                    if(parkSide!=null&&current.distance(parkSide)<=3){
                        parkSide=null;out=new Outputs();message="Parking safe lane reached; enter selected parking half";break;
                    }
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
                        out.forward=FieldLayout.clip((dx*c+dy*sine)*.12,.75);
                        out.strafeRight=FieldLayout.clip((dx*sine-dy*c)*.12,.75);
                    }
                    out.turnClockwise=-FieldLayout.clip(Math.toDegrees(parkHeadingError)*.05,.25);
                    message=parkVia==null?"Parking direct to configured location and heading":"Parking via Flower-clear bypass";
                    break;
                default:break;
            }
            if(state!=before||!running)out=new Outputs();
            boolean backupGardenStroke=fixedGardenPasses()&&(state==State.INTAKE_GARDEN||state==State.GARDEN_BACK_UP||state==State.GARDEN_RIGHT_SHIFT||state==State.GARDEN_RESTORE_HEADING||state==State.GARDEN_FORWARD_RETRY);
            out.intakePower=running&&state!=State.PARK&&(backupGardenStroke||(collecting&&s.ballCount<4))?1:0;
            if(state==before&&colorPickupActive()&&!yellowWindow)ColorPickup.steer(s,now,out,collectBlue,collectRed);
            out.forward=FieldLayout.clip(out.forward*translationPowerMultiplier,1);
            out.strafeRight=FieldLayout.clip(out.strafeRight*translationPowerMultiplier,1);
            lastFeeding=out.feederPower>0;
            out.pipeline=pipeline();
            if(io.stopRequested()){stop();return;}
            io.apply(out);
        }catch(java.util.concurrent.CancellationException e){stop();}
        catch(RuntimeException e){
            // The backup route must keep its two-pass sequence alive even if a
            // transient odometry/navigation sample fails.  It retries on the
            // next loop and still enters the normal 26-second parking fallback.
            error="";message="Transient autonomous fault; retrying current Team 1 state";io.apply(new Outputs());
        }
    }
}
