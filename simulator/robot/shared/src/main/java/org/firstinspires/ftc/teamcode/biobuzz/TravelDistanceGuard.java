package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.HashMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose;

/** Actual odometry travel, never motor power multiplied by elapsed time. */
public final class TravelDistanceGuard {
    private final Map<String,Integer> occurrences=new HashMap<String,Integer>();
    private Pose previous;
    private String leg="";
    private double traveled,reference=Double.NaN,total;
    public void reset(){occurrences.clear();previous=null;leg="";traveled=total=0;reference=Double.NaN;}
    public void begin(String key,AutoStateMachine.Routine routine,Pose pose){
        int n=occurrences.containsKey(key)?occurrences.get(key)+1:1;occurrences.put(key,n);
        leg=key+" #"+n;traveled=0;reference=RecordedDistances.inches(routine,key,n);previous=pose;
    }
    /** Re-anchor after an invalid sensor interval; do not invent missing travel. */
    public void resumeAt(Pose pose){previous=pose;}
    public void update(Pose pose){
        if(pose==null||!pose.finite())throw new IllegalArgumentException("Valid measured pose required");
        if(previous!=null){double delta=previous.distance(pose);traveled+=delta;total+=delta;}
        previous=pose;
    }
    public String leg(){return leg;}
    public double traveled(){return traveled;}
    public double total(){return total;}
    public double reference(){return reference;}
    public double limit(){return Double.isFinite(reference)?reference+Math.max(3,reference*.15):Double.NaN;}
    public boolean exceeded(){return !Double.isFinite(reference)||traveled>limit();}
}
