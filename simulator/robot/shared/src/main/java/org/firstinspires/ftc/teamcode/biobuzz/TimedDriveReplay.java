package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Outputs;
import org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine.State;

/** Bounded open-loop playback. Intentionally never calls RobotIO.readSensors(). */
public final class TimedDriveReplay {
    private static final class Segment {
        final double end;final Outputs outputs;final State state;
        Segment(String row,double start){
            String[] v=row.split(",");int ticks=Integer.parseInt(v[0]);
            if(ticks<1||ticks>1500)throw new IllegalArgumentException("Invalid recording duration");
            end=start+ticks*.02;outputs=new Outputs();outputs.forward=Double.parseDouble(v[1]);
            outputs.strafeRight=Double.parseDouble(v[2]);outputs.turnClockwise=Double.parseDouble(v[3]);state=State.valueOf(v[4]);
            for(double p:new double[]{outputs.forward,outputs.strafeRight,outputs.turnClockwise})
                if(!Double.isFinite(p)||Math.abs(p)>1)throw new IllegalArgumentException("Invalid recorded drive power");
            if(state==State.ERROR||state==State.SAFETY_HOLD||state==State.STOPPED)throw new IllegalArgumentException("Fault in recording");
        }
    }
    private final RobotIO io;private final Segment[] segments;private final double duration;
    private boolean running;private int index,alliance;private double startAt,lastTime;
    private State state=State.STOPPED;private String error="";private Outputs last=new Outputs();
    public TimedDriveReplay(RobotIO io,AutoStateMachine.Routine routine){
        this.io=io;if(routine==null)throw new IllegalArgumentException("Routine required");
        List<Segment> list=new ArrayList<Segment>();double end=0;
        for(String chunk:routine==AutoStateMachine.Routine.SINGLE_TILT?TimedDriveRecording.SINGLE:TimedDriveRecording.FULL)
            for(String row:chunk.split(";"))if(!row.isEmpty()){Segment s=new Segment(row,end);list.add(s);end=s.end;}
        if(list.isEmpty()||end>=30||list.get(list.size()-1).state!=State.PARK)throw new IllegalArgumentException("Recording must end in park before 30 seconds");
        segments=list.toArray(new Segment[list.size()]);duration=end;
    }
    public void start(int alliance){
        if(alliance<0||alliance>1)throw new IllegalArgumentException("Alliance");
        this.alliance=alliance;startAt=lastTime=io.nowSeconds();index=0;state=segments[0].state;error="";running=true;
        if(!Double.isFinite(startAt)){fault("Invalid clock at start");return;}apply(new Outputs());
    }
    public void stop(){running=false;state=State.STOPPED;apply(new Outputs());}
    public boolean isRunning(){return running;}public State getState(){return state;}
    public String lastError(){return error;}public double duration(){return duration;}
    public Outputs getOutputs(){return last.copy();}
    public String status(){return String.format(java.util.Locale.ROOT,"%s · TIMED REPLAY (NO ODOMETRY) · %.1f / %.2f s · %s · mechanisms OFF%s",
        state,Math.max(0,io.nowSeconds()-startAt),duration,alliance==0?"RED":"BLUE",error.isEmpty()?"":" · "+error);}
    private void apply(Outputs out){
        if(io.stopRequested()){running=false;state=State.STOPPED;out=new Outputs();}
        last=out.copy();io.apply(out);
    }
    private void fault(String message){error=message;stop();}
    public void tick(){
        if(io.stopRequested()){stop();return;}
        if(!running){apply(new Outputs());return;}
        double now=io.nowSeconds();
        if(!Double.isFinite(now)||now<lastTime){fault("Clock invalid/backward; drive stopped");return;}
        double elapsed=now-startAt;
        if(elapsed>=30){fault("30-second hard stop");return;}
        if(now-lastTime>.25){fault("Control loop delayed; drive stopped");return;}lastTime=now;
        if(elapsed+1e-9>=duration){state=State.PARK;running=false;apply(new Outputs());return;}
        while(index<segments.length-1&&elapsed+1e-9>=segments[index].end)index++;
        Segment segment=segments[index];state=segment.state;apply(segment.outputs.copy());
    }
}
