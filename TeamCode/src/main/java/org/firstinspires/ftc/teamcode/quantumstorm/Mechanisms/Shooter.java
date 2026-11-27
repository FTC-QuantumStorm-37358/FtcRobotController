package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.ftc.SafeShutdown;

/**
 * Single shooter flywheel motor + the CR servo that feeds balls into it.
 * Same hardware and directions as QuantumStormTeleOp.
 */
public class Shooter {

    private final DcMotorEx shooter;
    private double targetRpm;
    private final CRServo feeder;
    private boolean feederPulseActive;
    private final org.firstinspires.ftc.teamcode.biobuzz.FeedCadence cadence=new org.firstinspires.ftc.teamcode.biobuzz.FeedCadence(Constants.SPINDEXER_FEED_PULSE_SEC,Constants.SPINDEXER_SHOT_GAP_SEC);
    // A new feed pulse waits for measured RPM to recover; the timed feed window keeps running.
    private final ShooterReadyGate readyGate=new ShooterReadyGate();
    // Time a feed has been requested with no pulse running (OFF gap plus any at-speed wait).
    private double heldSince=Double.NaN;
    private int forcedPulses;
    private boolean lastPulseForced;
    // Launch detection: a pulse opens a window; an RPM drop below the pulse-start RPM inside it is a launch.
    // Counters update when the window closes, so a counted ball has already left the flywheel.
    private int detectedShots,emptyPulseStreak;
    private double windowStart=Double.NaN,windowStartRpm,windowMinRpm,windowDipRpm,lastDipRpm=Double.NaN;
    private final DoubleSupplier clock;
    private double now(){return clock.getAsDouble();}

    public Shooter(HardwareMap hardwareMap) {
        this(hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR),
                hardwareMap.get(CRServo.class, Constants.SHOOTER_SERVO));
    }

    Shooter(DcMotorEx shooter,CRServo feeder) {
        this(shooter,feeder,()->System.nanoTime()/1e9);
    }

    /** Desktop checks inject a clock to exercise the settle time without sleeping. */
    Shooter(DcMotorEx shooter,CRServo feeder,DoubleSupplier clock) {
        this.shooter=shooter;
        this.feeder=feeder;
        this.clock=clock;

        shooter.setDirection(DcMotor.Direction.FORWARD);
        ShooterMotorConfiguration.apply(shooter);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feeder.setDirection(CRServo.Direction.REVERSE);

        stop();
    }

    public void spinUp() {
        apply(Constants.SHOOTER_RPM_MULTIPLIER,0);
    }

    /** RobotIO.shooterPower is a fraction of SHOOTER_BASE_RPM, not motor power. */
    public void apply(double rpmMultiplier,double feedPower) {
        apply(rpmMultiplier,feedPower,true);
    }

    public void apply(double rpmMultiplier,double feedPower,boolean newPulsesAllowed) {
        apply(rpmMultiplier,feedPower,newPulsesAllowed,false);
    }

    /**
     * newPulsesAllowed=false (e.g. just before the parking cutoff) lets a running pulse finish but
     * starts no new one. forceFeed=true (end of a preload window) starts pulses without waiting for
     * flywheel speed or the minimum-RPM floor; the pulse/OFF-gap cadence still applies.
     */
    public void apply(double rpmMultiplier,double feedPower,boolean newPulsesAllowed,boolean forceFeed) {
        double scale=Double.isFinite(rpmMultiplier)?Math.max(-1,Math.min(1,rpmMultiplier)):0;
        targetRpm=Constants.SHOOTER_BASE_RPM*scale;
        // One RPM is six degrees per second; the SDK uses the configured encoder resolution.
        shooter.setVelocity(targetRpm*6.0,AngleUnit.DEGREES);
        double now=now();
        // Track recovery every running loop, including spin-up, so the settle time is already
        // complete when the routine first requests feeding. No encoder read while stopped.
        double measured=targetRpm==0?Double.NaN:getMeasuredRpm();
        boolean atSpeed=readyGate.update(targetRpm,measured,now);
        boolean feedRequested=scale>0&&feedPower>0;
        if(!feedRequested||feederPulseActive)heldSince=Double.NaN;
        else if(Double.isNaN(heldSince))heldSince=now;
        // The gate must never cost a ball inside the timed feed window: once the request has
        // waited the maximum time since the last pulse ended, the next pulse starts anyway.
        // A forced pulse still needs a minimum RPM: a much slower flywheel only throws the ball away.
        boolean waitedTooLong=!Double.isNaN(heldSince)&&now-heldSince>=Constants.SHOOTER_READY_MAX_WAIT_SEC
                &&measured>=Constants.SHOOTER_FEED_MIN_RPM_FRACTION*targetRpm;
        double feederCommand=cadence.apply(scale>0?feedPower:0,now,(forceFeed||atSpeed||waitedTooLong)&&newPulsesAllowed);
        boolean pulseStarted=feederCommand>0&&!feederPulseActive;
        if(pulseStarted){lastPulseForced=!atSpeed;if(lastPulseForced)forcedPulses++;}
        trackLaunch(measured,now,pulseStarted);
        feederPulseActive=feederCommand>0;
        feeder.setPower(feederCommand*Constants.SHOOTER_SERVO_POWER);
    }

    private void trackLaunch(double measured,double now,boolean pulseStarted){
        if(!Double.isNaN(windowStart)){
            if(Double.isFinite(measured))windowMinRpm=Math.min(windowMinRpm,measured);
            // The next pulse, the window time or a stopped flywheel closes the current window.
            if(pulseStarted||targetRpm<=0||!(now>=windowStart)||now-windowStart>=Constants.SHOT_DETECT_WINDOW_SEC){
                lastDipRpm=windowStartRpm-windowMinRpm;
                if(lastDipRpm>=windowDipRpm){detectedShots++;emptyPulseStreak=0;}
                else emptyPulseStreak++;
                windowStart=Double.NaN;
            }
        }
        if(pulseStarted&&Double.isFinite(measured)&&targetRpm>0){
            windowStart=now;windowStartRpm=windowMinRpm=measured;
            windowDipRpm=Constants.SHOT_DETECT_DIP_FRACTION*targetRpm;
        }
    }

    /** Pulses started before the flywheel settled (max-wait fallback or forced feed) since INIT. */
    public int getForcedPulses(){return forcedPulses;}

    /** True when the most recent pulse started on the max-wait fallback, before the flywheel settled. */
    public boolean wasLastPulseForced(){return lastPulseForced;}

    /** Launches confirmed by an RPM dip since INIT; each is counted after its ball has left. */
    public int getDetectedShots(){return detectedShots;}

    /** Consecutive feed pulses whose detection window closed without a launch. */
    public int getEmptyPulseStreak(){return emptyPulseStreak;}

    /** Telemetry: launch count, the last window's RPM drop and the empty-pulse streak. */
    public String describeLaunches(){
        return String.format(java.util.Locale.US,"%d launched | last dip %.0f RPM (need %.0f) | empty pulses %d",
                detectedShots,lastDipRpm,Constants.SHOT_DETECT_DIP_FRACTION*Math.abs(targetRpm),emptyPulseStreak);
    }

    public double getTargetRpm() { return targetRpm; }

    /** At-speed feed gate state for telemetry, plus how many pulses started on the max-wait fallback. */
    public String describeReadiness() {
        return readyGate.describe()+(forcedPulses>0?" | pulses fed on timeout: "+forcedPulses:"");
    }

    public double getMeasuredRpm() { return shooter.getVelocity(AngleUnit.DEGREES)/6.0; }

    /** True only while the current timed spindexer pulse is actually feeding a pollen. */
    public boolean isFeederPulseActive() { return feederPulseActive; }

    public void feed() {
        feeder.setPower(cadence.apply(1,now())*Constants.SHOOTER_SERVO_POWER);
    }

    public void stopFeed() {
        cadence.apply(0,now());
        feederPulseActive=false;
        feeder.setPower(0);
    }

    public void stop() {
        targetRpm=0;
        cadence.apply(0,now());
        feederPulseActive=false;
        SafeShutdown.run(()->shooter.setVelocity(0),()->feeder.setPower(0));
    }
}
