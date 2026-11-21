package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Hive image-center calculation and fresh tx gate for autonomous shooting states. */
public final class ShootingAim {
    /** A slightly wider practical window prevents harmless camera jitter from blocking a shot. */
    public static final double ALIGNMENT_TOLERANCE_DEGREES=5.0;
    public static final double TURN_GAIN_PER_DEGREE=.02;
    public static final double MAX_TURN_POWER=.25;
    public static final int MIN_VISIBLE_TAGS=1;
    /** Five or more AprilTags means both Hive faces are visible and tipped. */
    public static final int BOTH_HIVES_VISIBLE_TAGS=5;
    public static final int REQUIRED_STABLE_FRAMES=1;
    public static final double REQUIRED_STABLE_SECONDS=0;
    private ShootingAim() { }

    /**
     * When both tipped Hives are in view, there can be 5–8 valid tags. The
     * field is then unambiguous enough to start a fixed, timed feed even if
     * the individual family matcher is temporarily undecided.
     */
    public static boolean bothHivesVisible(Vision vision) {
        if(vision==null||vision.targets==null)return false;
        int tags=0;
        for(Target target:vision.targets)
            if(target.id>=0&&Double.isFinite(target.tx)&&Math.abs(target.tx)<90)tags++;
        return tags>=BOTH_HIVES_VISIBLE_TAGS;
    }

    /** A negative alliance selects the single visible family for robot-relative routes. */
    public static Vision centerOnHive(Vision vision, int[][] tagGroups, int alliance) {
        if (!vision.valid || vision.pipeline != 0) {
            return new Vision(vision.valid, vision.pipeline, vision.capturedAt, vision.targets, Double.NaN, 0);
        }
        Target closest=null;
        // Multiple Hives may show AprilTags in the same image.  Select the
        // one nearest the crosshair, rather than averaging tags from separate
        // Hives; the nearest valid tag is the one that represents the shot aim.
        for (Target target : vision.targets) {
            if (!Double.isFinite(target.tx) || Math.abs(target.tx) >= 90) continue;
            if(closest==null||Math.abs(target.tx)<Math.abs(closest.tx))closest=target;
        }
        double center = closest == null ? Double.NaN : closest.tx;
        return new Vision(vision.valid, vision.pipeline, vision.capturedAt, vision.targets, center, closest==null?0:1);
    }

    public static String turnSuggestion(boolean tvAuthorized, double tx) {
        if (!tvAuthorized) return "No turn: no fresh valid target (tv)";
        if (!Double.isFinite(tx)) return "No turn: invalid tx";
        if (Math.abs(tx) <= ALIGNMENT_TOLERANCE_DEGREES + 1e-9) return "Aligned: hold heading (within 5 degrees)";
        return String.format(java.util.Locale.US, "Turn %s %.1f degrees toward target",
                tx > 0 ? "CLOCKWISE / RIGHT" : "COUNTERCLOCKWISE / LEFT", Math.abs(tx));
    }

    public static final class Alignment {
        private int stableFrames;
        private double stableSince=Double.NaN,lastFrame=Double.NaN;
        private boolean authorized;

        public void reset() {
            stableFrames=0;stableSince=lastFrame=Double.NaN;authorized=false;
        }

        public boolean align(Sensors sensors,double now,Outputs outputs) {
            return align(sensors,now,outputs,ALIGNMENT_TOLERANCE_DEGREES,true);
        }

        /**
         * Check a shot aim without necessarily commanding a correction turn.
         * Fixed starting poses use this to wait still for a valid tag frame.
         */
        public boolean align(Sensors sensors,double now,Outputs outputs,double toleranceDegrees,boolean allowTurn) {
            Vision vision=sensors.vision;
            outputs.shooterPower=outputs.feederPower=0;
            outputs.forward=outputs.strafeRight=0;
            outputs.turnClockwise=0;
            // A shot is a timed, paced sequence.  Once the robot receives one valid
            // centered AprilTag authorization, do not interrupt that sequence because
            // a later camera frame is delayed, invalid, or briefly loses the tag.
            if(authorized)return true;
            if(!sensors.cameraConnected||!vision.fresh(now,0)||now-vision.capturedAt>.25)
                return false;
            if(bothHivesVisible(vision)){
                authorized=true;
                return true;
            }
            if(vision.targets.length==0||vision.aimTagCount<MIN_VISIBLE_TAGS||!Double.isFinite(vision.tx)){
                return false;
            }
            if(Math.abs(vision.tx)>toleranceDegrees+1e-9){
                reset();
                if(allowTurn&&sensors.odometryValid&&sensors.pose!=null&&sensors.pose.finite()
                        &&now-sensors.sampledAt>=0&&now-sensors.sampledAt<=.25)
                    outputs.turnClockwise=FieldLayout.clip(vision.tx*TURN_GAIN_PER_DEGREE,MAX_TURN_POWER);
                return false;
            }
            if(vision.capturedAt==lastFrame){
                boolean authorized=stableFrames>=REQUIRED_STABLE_FRAMES
                        &&vision.capturedAt-stableSince+1e-9>=REQUIRED_STABLE_SECONDS;
                if(authorized)this.authorized=true;
                return this.authorized;
            }
            if(Double.isFinite(lastFrame)&&vision.capturedAt<lastFrame){reset();return false;}
            if(Double.isFinite(lastFrame)&&vision.capturedAt-lastFrame>.25)reset();
            if(stableFrames==0)stableSince=vision.capturedAt;
            stableFrames++;lastFrame=vision.capturedAt;
            boolean authorized=stableFrames>=REQUIRED_STABLE_FRAMES
                    &&vision.capturedAt-stableSince+1e-9>=REQUIRED_STABLE_SECONDS;
            if(authorized)this.authorized=true;
            return this.authorized;
        }
    }
}
