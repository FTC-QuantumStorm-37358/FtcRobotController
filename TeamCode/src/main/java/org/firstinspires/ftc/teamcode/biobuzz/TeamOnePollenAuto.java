package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.List;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Team One: paced preloads, camera-guided Garden pickup, parking. */
public final class TeamOnePollenAuto implements RobotController {
    public enum State { SHOOTING, POST_SHOT_TAG_WAIT, TO_GARDEN, TO_GARDEN_BACK, TO_GARDEN_RIGHT, GARDEN_ALIGN_HEADING, INTAKE_GARDEN, GARDEN_BACK_UP, GARDEN_RIGHT_SHIFT, GARDEN_RESTORE_HEADING, GARDEN_FORWARD_RETRY, TO_TEAM_TWO_SHOT, RETURN_TO_START_SHOT, RETURN_BACK_FROM_GARDEN, RETURN_STRAFE_TO_SHOT_LANE, RETURN_FORWARD_TO_SHOT, WAIT_FOR_TEAM_TWO_TAG, FOLLOW_UP_SHOOTING, OBSERVE_TILT, WAIT_TILT, CLEAR_PICKUP, SEARCH, APPROACH, CLOSE_PICKUP, REPOSITION, WAITING, PARK, STOPPED, ERROR }
    private final RobotIO io;
    private final FieldLayout field;
    private final Navigator nav;
    private final AutonomousShooter shooter=new AutonomousShooter();
    private final FeedBatchEnd batchEnd=new FeedBatchEnd();
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
    /** Fixed no-turn route from the Garden back to the original Team One shot pose. */
    private Pose returnBack,returnSide;
    private double returnSideDistance,returnForwardDistance;
    /** Inward bypass used to keep the direct Garden-to-park path clear of the Flower. */
    private Pose parkVia;
    /** Backup-only three-leg exit from the Team Two wall-shot pose. */
    private Pose parkRetreat,parkSide,parkEntry;
    /** Signed robot-relative strafe directions for normal Team One parking. */
    private double parkFirstStrafe,parkSecondStrafe,parkFinalStrafe;
    private double parkFirstDir,parkSideDir,parkViaDir;
    /** A normal parking leg ends this far before its target coordinate, absorbing loop delay and coast. */
    static final double PARK_LEG_LEAD_INCHES=2.0;
    /** Start time of the active no-turn parking leg. Prevents an odometry miss from trapping the robot in a strafe. */
    private double parkLegAt;
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
    public double spinUpSeconds=0.5,feedSeconds=3,parkingToleranceInches=2,approachPower=.25;
    /** Measured forward stroke, not an assumed pickup count. Calibrate on the robot. */
    public double gardenAdvanceInches=18,gardenAdvanceSeconds=5,gardenAdvancePower=.20;
    /** Zero keeps legacy timed strokes; a positive value ends each stroke at this measured distance. */
    public double gardenPassDistanceInches=0,gardenTransitPower=.35,gardenRepositionPower=.25;
    /** Cruise power for the retry branch's trip to the Garden (Hive not tipped after the first shot). */
    public double gardenRetryTransitPower=.35;
    /** Normal parking: end the final reverse this many inches sooner (the robot parks toward its front). */
    public double parkStopEarlierInches=0;
    /** Opening preload feed: final seconds that feed without waiting for flywheel speed (0 = off). */
    public double forceFeedLastSeconds=0;
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
    /** Optional Limelight Garden-exit behavior; disabled for the proven match route. */
    public boolean gardenYellowWindowEnabled=false;
    /** Multiplies translation commands when a controlled field tune is needed. */
    public double translationPowerMultiplier=1;
    /** Backup mode: after two Garden passes, use Team Two's taped wall shot and fire only on a live tag. */
    public boolean useTeamTwoFollowUpShot=false;
    public int gardenPassLimit=0;
    /** Completed full Garden intake strokes in the fixed two-pass routines. */
    private int gardenPassesCompleted;
    /** Normal Team 1 branch when the first shot did not tilt the Hive. */
    private boolean repeatStartShotAfterGarden,postShotDecisionMade,postShotMissingTagFrame,postShotTagAuthorized;
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
                ||!Double.isFinite(gardenRetryTransitPower)||gardenRetryTransitPower<=0||gardenRetryTransitPower>1
                ||!Double.isFinite(parkStopEarlierInches)||parkStopEarlierInches<0||parkStopEarlierInches>12
                ||!Double.isFinite(forceFeedLastSeconds)||forceFeedLastSeconds<0||forceFeedLastSeconds>feedSeconds
                ||!Double.isFinite(gardenRepositionPower)||gardenRepositionPower<=0||gardenRepositionPower>.6
                ||!Double.isFinite(gardenFirstPassSeconds)||gardenFirstPassSeconds<=0||gardenFirstPassSeconds>5
                ||!Double.isFinite(gardenRepositionSeconds)||gardenRepositionSeconds<=0||gardenRepositionSeconds>2
                ||!Double.isFinite(gardenStagingOffsetInches)||gardenStagingOffsetInches<0||gardenStagingOffsetInches>24
                ||!Double.isFinite(gardenIntakeHeadingRadians)||gardenPassLimit<0||gardenPassLimit>8
                ||!Double.isFinite(translationPowerMultiplier)||translationPowerMultiplier<=0||translationPowerMultiplier>1.25)
            throw new IllegalArgumentException("Invalid Team One configuration");
        this.alliance=alliance;this.power=power;startAt=lastNow=io.nowSeconds();spinAt=Double.NaN;
        shootingAlignment.reset();
        running=true;tiltConfirmed=survey=false;avoidUntil=0;current=destination=gardenEntry=parkVia=parkRetreat=parkSide=parkEntry=teamTwoShotExit=teamTwoShotVia=teamTwoShotLane=gardenLegStart=returnBack=returnSide=null;gardenBackDistance=gardenRightDistance=returnSideDistance=returnForwardDistance=parkFirstStrafe=parkSecondStrafe=parkFinalStrafe=0;parkLegAt=startAt;gardenIntakeHeading=Double.NaN;message=error="";
        collecting=lastFeeding=preloadTagLatched=false;feedElapsed=0;lastCommandAt=startAt;gardenPassesCompleted=0;repeatStartShotAfterGarden=postShotDecisionMade=postShotMissingTagFrame=postShotTagAuthorized=false;
        shotGate=new HiveShotGate(field,alliance,false);
        hive=new HiveVision(field,alliance);nav.useScriptedRoute(true,alliance);enter(State.SHOOTING);io.apply(new Outputs());
    }
    private void enter(State next){
        state=next;stateAt=io.nowSeconds();if(next==State.PARK)parkLegAt=stateAt;nav.reset();
        // The backup's second wall shot is a new shot sequence. Require one
        // fresh authorization there, then keep that authorization for the feed.
        if(next==State.WAIT_FOR_TEAM_TWO_TAG)shootingAlignment.reset();
        if(next==State.TO_GARDEN)collecting=true;
        if(next==State.POST_SHOT_TAG_WAIT)postShotMissingTagFrame=postShotTagAuthorized=false;
        // The robot shoots with its rear toward the Hive.  When it reaches the
        // Garden, use the measured arrival heading as the intake heading rather
        // than commanding a field-fixed 90 degree turn.  This keeps the direct
        // diagonal trip and every retry pass aligned with the chassis direction
        // that actually brought it to the Garden.
        if(next==State.GARDEN_ALIGN_HEADING)
            gardenIntakeHeading=current!=null&&current.finite()
                    ? Pose.angle(current.heading) : Pose.angle(gardenIntakeHeadingRadians);
        if(next==State.TO_GARDEN_BACK||next==State.TO_GARDEN_RIGHT
                ||next==State.RETURN_BACK_FROM_GARDEN||next==State.RETURN_STRAFE_TO_SHOT_LANE
                ||next==State.RETURN_FORWARD_TO_SHOT)gardenLegStart=current;
        if(next==State.INTAKE_GARDEN||next==State.GARDEN_FORWARD_RETRY)gardenEntry=current;
        // Every shot runs with the intake OFF, like the opening preload shot. Without calibrated
        // inventory, collecting otherwise stayed on through the return-to-shot retry, and the intake
        // pushed the first ball into the flywheel before it was up to speed (a low first shot).
        // beginGarden() turns collecting back on after the retry.
        // The opening SHOOTING holds the four preloads; the retry and follow-up loads are unknown.
        if(next==State.SHOOTING||next==State.FOLLOW_UP_SHOOTING){spinAt=Double.NaN;feedElapsed=0;lastFeeding=false;batchEnd.begin(io,next==State.SHOOTING&&!repeatStartShotAfterGarden);collecting=false;}
        if(next==State.PARK||next==State.STOPPED||next==State.ERROR)collecting=false;
        if(next==State.SEARCH&&surveyBeforePickup&&current!=null){survey=true;surveyAngle=0;surveyHeading=current.heading;bestHeading=Double.NaN;bestArea=0;}
    }
    public void stop(){running=false;state=State.STOPPED;io.apply(new Outputs());}
    public boolean isRunning(){return running;}
    public State getState(){return state;}
    public boolean colorPickupActive(){return running&&colorPickupEnabled
            &&(state==State.INTAKE_GARDEN||state==State.GARDEN_FORWARD_RETRY);}
    public String lastError(){return error;}
    /** Compatibility telemetry for the optional Yellow-pollen exit behavior. */
    public boolean leaveAuthorized(){return false;}
    public double shooterTargetRpm(){return shooter.targetRpm();}
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
    /**
     * Garden transit power. The retry branch (Hive not tipped) cruises at gardenRetryTransitPower,
     * then finishes the last 6 inches of each leg at the usual gardenTransitPower, so the stop is
     * the same as before. Every other trip keeps gardenTransitPower throughout.
     */
    private double gardenOutboundPower(double legInches){
        if(!repeatStartShotAfterGarden||gardenRetryTransitPower<=gardenTransitPower)return gardenTransitPower;
        double travelled=gardenLegStart==null||current==null?0:current.distance(gardenLegStart);
        return legInches-travelled>6?gardenRetryTransitPower:gardenTransitPower;
    }
    private boolean gardenLegDone(double targetInches){
        // Eight inches per second is deliberately conservative.  The timeout
        // guarantees that a momentary odometry dropout cannot strand the robot
        // before it starts the intake routine.
        return gardenLegStart!=null&&current!=null
                &&current.distance(gardenLegStart)>=Math.max(0,targetInches-1)
                || io.nowSeconds()-stateAt>=Math.max(.45,targetInches/8.0+.55);
    }
    private void planReturnToStartShot(){
        destination=field.hive[alliance];
        double h=current.heading,c=Math.cos(h),s=Math.sin(h);
        // First retreat straight away from the Garden/Flower. Then use a pure
        // strafe to line up with the original shot lane before the final
        // forward approach. No diagonal or turn is permitted in this route.
        double clear=gardenStagingOffsetInches;
        returnBack=new Pose(current.x-clear*c,current.y-clear*s,h);
        double toShotX=destination.x-returnBack.x,toShotY=destination.y-returnBack.y;
        // The prior return ended slightly to the robot's right. Keep a small
        // inward margin on this sideways leg and finish on the taped lane.
        double lateral=(toShotX*s-toShotY*c)*.90;
        returnSide=new Pose(returnBack.x+lateral*s,returnBack.y-lateral*c,h);
        returnSideDistance=Math.abs(lateral);
        returnForwardDistance=Math.abs((destination.x-returnSide.x)*c+(destination.y-returnSide.y)*s);
    }
    private boolean fixedGardenPasses(){return useTeamTwoFollowUpShot||repeatStartShotAfterGarden;}
    private int requiredGardenPasses(){return Math.max(1,gardenPassLimit);}
    private boolean returningAfterFixedPasses(){
        return fixedGardenPasses()&&gardenPassesCompleted>=requiredGardenPasses()
                &&(state==State.TO_TEAM_TWO_SHOT||state==State.RETURN_BACK_FROM_GARDEN
                ||state==State.RETURN_STRAFE_TO_SHOT_LANE||state==State.RETURN_FORWARD_TO_SHOT);
    }
    /** A centered live tag, or five-plus tags, authorizes the normal Team 1 retry. */
    private boolean normalRetryTagReady(Sensors sensors,double now){
        if(!sensors.cameraConnected||sensors.vision==null||!sensors.vision.fresh(now,0))return false;
        Target[] targets=sensors.vision.targets;
        if(targets==null||targets.length==0)return false;
        if(targets.length>4)return true;
        for(Target target:targets)
            if(Double.isFinite(target.tx)&&Math.abs(target.tx)<=8)return true;
        return false;
    }
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
            planReturnToStartShot();
            enter(State.RETURN_BACK_FROM_GARDEN);
            message="Two Garden passes complete; back away from Garden before crossing to the shot lane";
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
        // Normal Team 1 uses the same four no-turn legs for both post-shot
        // outcomes: move into the Flower/Hive gap, reverse toward Parking,
        // strafe through the Flower/Parking lane without reaching the wall,
        // then reverse straight into the parking area. This preserves room to
        // push into an occupied legal parking half without touching the Flower.
        Pose location=field.teamOneStartSidePark[alliance];
        double heading=current.heading;
        double gapX=alliance==0?-42:42;
        // Use a wider field-side lane so the second strafe reaches the
        // parking corridor instead of leaving the robot too close to Flower.
        // Keep the final lane inside the Flower clearance corridor.  The
        // previous wider value placed the strafe too near the Flower frame.
        double parkingLaneX=alliance==0?-54:54;
        // Keep this reverse leg short.  The final entry begins from this
        // staging line, so an unnecessarily deep retreat can make the robot
        // continue backing past the intended parking position.
        double reverseLaneY=alliance==0?-22:22;
        // 1) Sideways to the safe open gap between the Flower and Hive.
        parkRetreat=new Pose(gapX,current.y,heading);
        // 2) Backward through that gap, clear of the Flower.
        parkSide=new Pose(gapX,reverseLaneY,heading);
        // 3) A short sideways move to the field-side parking lane, not wall.
        parkVia=new Pose(parkingLaneX,reverseLaneY,heading);
        parkEntry=null;
        double firstDx=parkRetreat.x-current.x,firstDy=parkRetreat.y-current.y;
        double secondDx=parkVia.x-parkSide.x,secondDy=parkVia.y-parkSide.y;
        double c=Math.cos(heading),s=Math.sin(heading);
        parkFirstStrafe=Math.signum(firstDx*s-firstDy*c);
        parkSecondStrafe=Math.signum(secondDx*s-secondDy*c);
        // Field direction of each axis-aligned leg: X for both strafes, Y for the reverse.
        parkFirstDir=Math.signum(parkRetreat.x-current.x);
        parkSideDir=Math.signum(parkSide.y-parkRetreat.y);
        parkViaDir=Math.signum(parkVia.x-parkSide.x);
        // 4) A slightly shorter pure reverse into the assigned parking half.
        double finalParkingY=location.y+(alliance==0?1:-1);
        // Stop the reverse sooner by parkStopEarlierInches: pull the target back toward the lane.
        finalParkingY-=Math.signum(finalParkingY-reverseLaneY)*parkStopEarlierInches;
        parkEntry=new Pose(parkingLaneX,finalParkingY,heading);
        // 5) Move only one inch inward from the field wall after entering so
        // the chassis is not pressed against it while still parked legally.
        double finalParkingX=parkingLaneX+(alliance==0?1:-1);
        destination=new Pose(finalParkingX,finalParkingY,heading);
        double finalDx=destination.x-parkEntry.x,finalDy=destination.y-parkEntry.y;
        parkFinalStrafe=Math.signum(finalDx*s-finalDy*c);
        enter(State.PARK);message="Parking: gap, reverse clear, parking-lane strafe, short reverse, then inward clearance strafe";
    }
    /**
     * The final parking reverse is usually ended by this time limit, not by reaching its target,
     * so stopping earlier also shortens it: about 0.02 s per inch at parking power.
     */
    private double parkFinalReverseSeconds(){return Math.max(.10,.35-.02*parkStopEarlierInches);}
    /** True once position has reached (within the lead) or passed target moving in direction dir. */
    static boolean legReached(double position,double target,double dir){
        return dir==0||(target-position)*dir<=PARK_LEG_LEAD_INCHES;
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
            boolean stationaryShot=state==State.SHOOTING||state==State.POST_SHOT_TAG_WAIT
                    ||state==State.WAIT_FOR_TEAM_TWO_TAG||state==State.FOLLOW_UP_SHOOTING;
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
            // Let a completed fixed two-pass routine actually leave the Garden
            // before the parking cutoff takes over.  Otherwise the second pass
            // can finish exactly at the cutoff and be replaced by PARK before
            // the return-to-shot state receives a drive command.
            if(elapsed>=26.0&&state!=State.PARK&&!returningAfterFixedPasses())park();
            // Normal Team 1 uses a three-leg Flower-clear parking route. Keep
            // collecting through 23 seconds, then reserve about five seconds
            // for the faster route to finish at the final target around 28–29.
            if(!useTeamTwoFollowUpShot&&!repeatStartShotAfterGarden&&gardenMotion()&&elapsed>=26.0&&state!=State.PARK)park();
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
                    // The timed window is the upper limit; flywheel-confirmed launches end it early.
                    if(feedElapsed>=feedSeconds||batchEnd.complete(io)){
                        if(current==null){message="Preloads complete; waiting for a valid Pinpoint pose before Garden departure";break;}
                        if(!recoverPollenAfterTilt){
                            if(repeatStartShotAfterGarden){
                                // The retry shot is complete. Return to the
                                // normal continuous Garden intake until park.
                                repeatStartShotAfterGarden=false;gardenPassLimit=0;beginGarden();
                                message="Retry shot complete; resume continuous Garden intake until parking";
                                break;
                            }
                            if(!useTeamTwoFollowUpShot&&!postShotDecisionMade){
                                enter(State.POST_SHOT_TAG_WAIT);message="Opening timed shot complete; wait 4.5 seconds for the Hive to settle and check AprilTags";break;
                            }
                            beginGarden();message="Opening timed shot complete; go straight to alliance Garden without a departure turn";break;
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
                        out.shooterPower=shooter.command(current,power);if(now-spinAt>=spinUpSeconds)out.feederPower=1;
                        // Final stretch of the preload window with a preload still inside: feed it anyway.
                        out.forceFeed=out.feederPower>0&&batchEnd.forceRemaining(io,feedElapsed,feedSeconds,forceFeedLastSeconds);
                        message=requireAprilTagForPreload?(allowAnyFreshAprilTag?"AprilTag authorization latched; paced preload feed":"Own raised Cell authorization latched; paced preload feed"):"Fixed Team 1 start pose; paced preload feed";
                    }
                    break;
                case POST_SHOT_TAG_WAIT:
                    // Give the Hive four seconds to settle. Only the last
                    // half-second is the decision window: one qualifying live
                    // frame latches the retry route (centered within ±8
                    // degrees, or five-plus visible tags).
                    if(age>=4.0&&normalRetryTagReady(s,now))postShotTagAuthorized=true;
                    if(age<4.5){
                        message=age<4.0?"Post-shot Hive check: waiting four seconds for Hive to settle"
                                :postShotTagAuthorized
                                ?"Final 0.5-second tag window: retry-shot authorization latched"
                                :"Final 0.5-second tag window: need centered tag within ±8° or five visible AprilTags";
                        break;
                    }
                    postShotDecisionMade=true;
                    if(postShotTagAuthorized){
                        repeatStartShotAfterGarden=true;gardenPassLimit=2;beginGarden();
                        message="AprilTag retry authorization latched; run two Garden intake passes, return, then shoot again";
                    }else{
                        beginGarden();message="No AprilTag retry authorization after 4.5 seconds; begin continuous Garden intake until parking";
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
                    out.forward=-gardenOutboundPower(gardenBackDistance);
                    if(gardenLegDone(gardenBackDistance)){
                        enter(State.TO_GARDEN_RIGHT);
                        message="Garden transit: backward leg complete; strafe right without turning";
                    }else message="Garden transit: backing up; hold heading";
                    break;
                case TO_GARDEN_RIGHT:
                    out.strafeRight=gardenOutboundPower(gardenRightDistance);
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
                    if(s.ballCount>=4&&!fixedGardenPasses()){followUpShotOrPark();break;}
                    if(gardenPassDone(age)){
                        if(fixedGardenPasses()){
                            gardenPassesCompleted++;
                            if(gardenPassesCompleted>=requiredGardenPasses()){finishFixedGardenPasses();break;}
                        }
                        enter(State.GARDEN_BACK_UP);
                        message="Garden intake pass complete; back up slightly before the next pass";
                        break;
                    }
                    out.forward=gardenAdvancePower;out.intakePower=1;
                    Target gardenBall=pollen(s.vision,now);
                    if(!fixedGardenPasses()&&gardenBall!=null)
                        out.strafeRight=Math.abs(gardenBall.tx)>6?FieldLayout.clip(gardenBall.tx*.012,.12):0;
                    // Keep driving the intake forward until it has measured four
                    // pieces or the parking deadline takes over.
                    double headingError=Double.isFinite(gardenIntakeHeading)?Pose.angle(gardenIntakeHeading-current.heading):0;
                    if(!fixedGardenPasses())
                        out.turnClockwise=-FieldLayout.clip(Math.toDegrees(headingError)*.05,.22);
                    message=fixedGardenPasses()?"Fixed Garden pass: full forward wall push; no turn or strafe":"Garden intake: hold first-pass heading; drive until four pieces or parking";
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
                        if(fixedGardenPasses()){
                            gardenPassesCompleted++;
                            if(gardenPassesCompleted>=requiredGardenPasses())finishFixedGardenPasses();
                            else {enter(State.GARDEN_BACK_UP);message="Garden pass complete; reset for the next measured intake pass";}
                        }
                        else {enter(State.INTAKE_GARDEN);message="Garden retry complete; repeat forward intake cycle until four pieces or parking";}
                    }
                    break;
                case TO_TEAM_TWO_SHOT:
                    out=directMoveToTeamTwoShot();
                    if(out==null){enter(State.WAIT_FOR_TEAM_TWO_TAG);message="At Team Two wall shot; waiting for a fresh AprilTag before follow-up feed";}
                    break;
                case RETURN_TO_START_SHOT:
                    // Retained only for compatibility with saved simulator
                    // states. Newly started routines use the fixed three legs.
                    planReturnToStartShot();enter(State.RETURN_BACK_FROM_GARDEN);
                    message="Return route: reverse clear of Garden, strafe to shot lane, then forward";
                    break;
                case RETURN_BACK_FROM_GARDEN:
                    out.forward=-gardenTransitPower;
                    if(gardenLegDone(gardenStagingOffsetInches)){
                        enter(State.RETURN_STRAFE_TO_SHOT_LANE);
                        message="Return route: Garden clearance complete; strafe to original shot lane";
                    }else message="Return route: backing away from Garden; hold heading";
                    break;
                case RETURN_STRAFE_TO_SHOT_LANE:
                    if(returnSideDistance<=1){enter(State.RETURN_FORWARD_TO_SHOT);message="Return route: already on shot lane; drive forward";break;}
                    double sideDx=returnSide.x-current.x,sideDy=returnSide.y-current.y;
                    double sideC=Math.cos(current.heading),sideS=Math.sin(current.heading);
                    out.strafeRight=FieldLayout.clip((sideDx*sideS-sideDy*sideC)*.12,gardenTransitPower);
                    if(gardenLegDone(returnSideDistance)){
                        enter(State.RETURN_FORWARD_TO_SHOT);
                        message="Return route: shot lane reached; drive forward to taped shot pose";
                    }else message="Return route: strafing to original shot lane; hold heading";
                    break;
                case RETURN_FORWARD_TO_SHOT:
                    // The strafe leg already put the chassis on the original
                    // shot lane. Finish with a proportional forward-only move
                    // so it arrives at the taped pose without overshooting,
                    // hitting the wall, or making a sideways correction.
                    double forwardDx=destination.x-current.x,forwardDy=destination.y-current.y;
                    double forwardC=Math.cos(current.heading),forwardS=Math.sin(current.heading);
                    double forwardRemaining=forwardDx*forwardC+forwardDy*forwardS;
                    out.forward=FieldLayout.clip(forwardRemaining*.16,gardenTransitPower);
                    out.strafeRight=out.turnClockwise=0;
                    if(current.distance(destination)<=1.0||forwardRemaining<=1.0){
                        enter(State.SHOOTING);message="Original taped shot pose reached; spin up and fire retry shot";
                    }else message="Return route: driving forward into original taped shot pose";
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
                    if(feedElapsed>=feedSeconds||batchEnd.complete(io)){park();break;}
                    if(aimForShooting&&!shootingAlignment.align(s,now,out)){
                        spinAt=Double.NaN;message="Follow-up feed paused: waiting for AprilTag aim to return";break;
                    }
                    if(Double.isNaN(spinAt))spinAt=now;
                    out.shooterPower=shooter.command(current,power);
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
                    // Advance through parking waypoints without outputting a
                    // zero-power loop between legs. This keeps the route
                    // fluid and avoids the small stop/reverse adjustments.
                    // A pure strafe cannot correct a forward-coordinate error.
                    // If Pinpoint misses a waypoint, advance after a short
                    // bounded window rather than strafing forever. Normal
                    // odometry still advances immediately from the distance test.
                    // Normal Team One legs are pure strafes/reverses along one field axis. End each one
                    // when that coordinate is reached or passed: a strafe that drifts a few inches
                    // forward used to miss the 3-inch circle and keep strafing for the full 0.9 s,
                    // overshooting the Flower/Hive gap and reversing into a Hive post.
                    boolean normalPark=!useTeamTwoFollowUpShot;
                    if(parkRetreat!=null&&(current.distance(parkRetreat)<=3||now-parkLegAt>=.90
                            ||normalPark&&legReached(current.x,parkRetreat.x,parkFirstDir))){
                        parkRetreat=null;parkLegAt=now;message="Parking gap reached; continue reverse clearance";
                    }else if(parkRetreat==null&&parkSide!=null&&(current.distance(parkSide)<=3||now-parkLegAt>=.90
                            ||normalPark&&legReached(current.y,parkSide.y,parkSideDir))){
                        parkSide=null;parkLegAt=now;message="Parking reverse clearance reached; strafe to parking lane";
                    }else if(parkRetreat==null&&parkSide==null&&parkVia!=null&&(current.distance(parkVia)<=3||now-parkLegAt>=.90
                            ||normalPark&&legReached(current.x,parkVia.x,parkViaDir))){
                        parkVia=null;parkLegAt=now;message="Parking lane reached; reverse into parking";
                    }
                    if(parkRetreat==null&&parkSide==null&&parkVia==null&&parkEntry!=null
                            &&((current.distance(parkEntry)<=1&&Math.abs(Pose.angle(parkEntry.heading-current.heading))<=Math.toRadians(3))
                            ||now-parkLegAt>=parkFinalReverseSeconds())){
                        parkEntry=null;parkLegAt=now;message="Parking depth reached; make final inward wall-clearance strafe";
                    }
                    Pose parkingGoal=parkRetreat!=null?parkRetreat:parkSide!=null?parkSide:parkVia!=null?parkVia:parkEntry!=null?parkEntry:destination;
                    double parkHeadingError=Pose.angle(parkingGoal.heading-current.heading);
                    if(parkVia==null&&parkEntry==null&&current.distance(destination)<=parkingToleranceInches
                            &&(!useTeamTwoFollowUpShot||Math.abs(parkHeadingError)<=Math.toRadians(3))){running=false;message="Parked at configured location and heading: all outputs OFF";break;}
                    // Garden collection may have carried the robot outside of the
                    // simulator's modeled corridor. At the 26-second deadline use a
                    // simple field-vector drive toward parking instead of invoking
                    // the collision-route planner, which can reject that pose and
                    // throw a runtime exception. Correct toward the configured
                    // parking heading before the final few inches.
                    double dx=parkingGoal.x-current.x,dy=parkingGoal.y-current.y;
                    double c=Math.cos(current.heading),sine=Math.sin(current.heading);
                    // Parking begins at 26 seconds. Increase speed as the
                    // reserve gets tighter, while still using the fixed safe
                    // corridor and no-turn normal Team 1 route.
                    double parkMax=elapsed>=28.0?1.0:.90;
                    double parkGain=elapsed>=28.0?.20:.16;
                    if(!useTeamTwoFollowUpShot){
                        // Normal Team One parking is intentionally four pure
                        // robot-relative commands: strafe, reverse, strafe,
                        // reverse. Never combine axes or command a turn.
                        if(parkRetreat!=null){
                            out.strafeRight=parkFirstStrafe*parkMax;
                            message="Parking: strafe into Flower/Hive gap";
                        }else if(parkSide!=null){
                            out.forward=-parkMax;
                            message="Parking: reverse through Flower/Hive gap";
                        }else if(parkVia!=null){
                            out.strafeRight=parkSecondStrafe*parkMax;
                            message="Parking: strafe to field-side parking lane";
                        }else if(parkEntry!=null){
                            out.forward=-parkMax;
                            message="Parking: short final reverse into assigned parking half";
                        }else{
                            // Never run the final clearance strafe indefinitely when
                            // odometry has stopped updating near parking.
                            if(now-parkLegAt>=.15){
                                running=false;message="Parking final inward strafe timed out safely: all outputs OFF";break;
                            }
                            if(current.distance(destination)<=parkingToleranceInches){
                                running=false;message="Parked clear of wall: all outputs OFF";break;
                            }
                            out.strafeRight=parkFinalStrafe*parkMax;
                            message="Parking: final inward wall-clearance strafe";
                        }
                        out.turnClockwise=0;
                        break;
                    }
                    boolean finalNormalReverse=!useTeamTwoFollowUpShot&&parkRetreat==null&&parkSide==null&&parkVia==null&&parkEntry==null;
                    if(finalNormalReverse){
                        // The previous leg stopped at the wall-side staging
                        // lane with the rear aimed at parking. Finish with a
                        // pure reverse command: no strafe and no turn can
                        // replace this final entry.
                        // Decelerate to the final parking coordinate. A fixed
                        // reverse command could overshoot between Pinpoint
                        // samples and then run until the 30-second hard stop.
                        double finalForwardRemaining=dx*c+dy*sine;
                        double finalLateralRemaining=dx*sine-dy*c;
                        // Do not reverse forever when the final lane is a few
                        // inches off laterally. Once the commanded reverse
                        // depth is reached, stop rather than correcting forward
                        // or driving farther into the wall.
                        if(finalForwardRemaining>=-parkingToleranceInches
                                &&Math.abs(finalLateralRemaining)<=4){
                            running=false;message="Parked at final reverse depth: all outputs OFF";break;
                        }
                        out.forward=Math.min(0,FieldLayout.clip(finalForwardRemaining*parkGain,parkMax));
                        out.strafeRight=0;
                    }else if(parkVia==null&&parkEntry==null&&current.distance(destination)<=4&&Math.abs(parkHeadingError)>Math.toRadians(3)){
                        out.forward=out.strafeRight=0;
                    }else{
                        out.forward=FieldLayout.clip((dx*c+dy*sine)*parkGain,parkMax);
                        out.strafeRight=FieldLayout.clip((dx*sine-dy*c)*parkGain,parkMax);
                    }
                    // The normal route is intentionally translation-only.
                    // It must never spin while entering a potentially occupied
                    // parking area.  Backup auto retains its separate heading
                    // controller.
                    out.turnClockwise=useTeamTwoFollowUpShot
                            ?-FieldLayout.clip(Math.toDegrees(parkHeadingError)*.05,.25):0;
                    message=parkVia!=null||parkEntry!=null?"Parking through Flower/Hive and outside-parking lanes":"Parking reverse entry to configured location and heading";
                    break;
                default:break;
            }
            if(state!=before||!running)out=new Outputs();
            boolean backupGardenStroke=fixedGardenPasses()&&(state==State.INTAKE_GARDEN||state==State.GARDEN_BACK_UP||state==State.GARDEN_RIGHT_SHIFT||state==State.GARDEN_RESTORE_HEADING||state==State.GARDEN_FORWARD_RETRY);
            out.intakePower=running&&state!=State.PARK&&(backupGardenStroke||(collecting&&s.ballCount<4))?1:0;
            if(state==before&&colorPickupActive())ColorPickup.steer(s,now,out,collectBlue,collectRed);
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
