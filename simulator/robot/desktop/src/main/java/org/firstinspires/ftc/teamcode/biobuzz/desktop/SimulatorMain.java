package org.firstinspires.ftc.teamcode.biobuzz.desktop;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import org.firstinspires.ftc.teamcode.biobuzz.*;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** JSON-lines transport only. All autonomous decisions live in the shared Java source tree. */
public final class SimulatorMain implements RobotIO {
    private double time;
    private Sensors sensors;
    private Outputs outputs=new Outputs();
    private final FieldLayout field=new FieldLayout();
    private final AutoStateMachine auto=new AutoStateMachine(this,field);
    private String runId="";
    private int seq=-1;
    public double nowSeconds(){return time;}
    public Sensors readSensors(){return sensors;}
    public void apply(Outputs o){outputs=o.copy();}
    public static void main(String[] args)throws IOException{
        SimulatorMain main=new SimulatorMain();
        System.out.println(Json.write(Json.map("type","ready","protocol",1,"engine","java","tickSeconds",.02,"nearTags",tagLists(main.field.nearTags),"farTags",tagLists(main.field.farTags))));
        BufferedReader reader=new BufferedReader(new InputStreamReader(System.in,StandardCharsets.UTF_8));String line;
        while((line=reader.readLine())!=null){try{if(line.length()>131072)throw new IllegalArgumentException("Message too large");System.out.println(Json.write(main.message(Json.object(Json.parse(line)))));}
            catch(RuntimeException e){main.auto.stop();System.out.println(Json.write(Json.map("type","error","runId",main.runId,"seq",main.seq,"message",e.getMessage()==null?"Invalid message":e.getMessage())));}}
        main.auto.stop();
    }
    private static List<List<Integer>> tagLists(int[][] groups){List<List<Integer>> result=new ArrayList<List<Integer>>();for(int[] group:groups){List<Integer> ids=new ArrayList<Integer>();for(int id:group)ids.add(id);result.add(ids);}return result;}
    private static Object pose(Pose p) { return p==null?null:Json.map("x",p.x,"y",p.y,"heading",p.heading); }
    private Object navigation() { List<Object> route=new ArrayList<Object>();for(Pose p:auto.route())route.add(pose(p));return Json.map("destination",pose(auto.destination()),"route",route); }
    private Object message(Map<String,Object> m){
        if(Json.integer(m,"protocol")!=1)throw new IllegalArgumentException("Unsupported protocol");
        String type=Json.string(m,"type"),id=Json.string(m,"runId");int next=Json.integer(m,"seq");
        if(type.equals("start")){
            if(id.length()<1||id.length()>80||next!=0||Json.number(m,"time")!=0)throw new IllegalArgumentException("Invalid start envelope");
            auto.stop();runId=id;seq=0;time=0;sensors=decodeSensors(Json.object(m.get("sensors")));
            auto.start(Json.integer(m,"alliance"),Json.number(m,"shooterPower"));auto.tick();
        }else{
            if(!id.equals(runId)||next!=seq+1)throw new IllegalArgumentException("Stale run or out-of-order sequence");
            if(type.equals("stop")){seq=next;auto.stop();}
            else if(type.equals("step")){
                double nextTime=Json.number(m,"time");if(Math.abs(nextTime-time-.02)>1e-8)throw new IllegalArgumentException("Expected fixed 20 ms step");
                seq=next;time=nextTime;sensors=decodeSensors(Json.object(m.get("sensors")));auto.tick();
            }else throw new IllegalArgumentException("Unknown message type");
        }
        double[] w=outputs.wheelPowers();
        return Json.map("type","command","protocol",1,"runId",runId,"seq",seq,"time",time,"running",auto.isRunning(),"state",auto.getState().name(),"status",auto.status(),"navigation",navigation(),
            "outputs",Json.map("wheelPowers",Arrays.asList(w[0],w[1],w[2],w[3]),"forward",outputs.forward,"strafeRight",outputs.strafeRight,"turnClockwise",outputs.turnClockwise,"shooterPower",outputs.shooterPower,"feederPower",outputs.feederPower,"intakePower",outputs.intakePower,"pipeline",outputs.pipeline));
    }
    private Sensors decodeSensors(Map<String,Object> m){
        Map<String,Object> p=Json.object(m.get("pose")),v=Json.object(m.get("vision"));List<Target> targets=new ArrayList<Target>();
        Object raw=v.get("targets");if(!(raw instanceof List)||((List<?>)raw).size()>64)throw new IllegalArgumentException("Invalid targets");
        for(Object item:(List<?>)raw){Map<String,Object> t=Json.object(item);double tx=Json.number(t,"tx"),ty=Json.number(t,"ty");if(Math.abs(tx)>180||Math.abs(ty)>180)throw new IllegalArgumentException("Invalid vision angles");targets.add(new Target(Json.integer(t,"id"),tx,ty));}
        Pose pose=new Pose(Json.number(p,"x"),Json.number(p,"y"),Json.number(p,"heading"));
        if(Math.abs(pose.x)>1000||Math.abs(pose.y)>1000)throw new IllegalArgumentException("Invalid pose");
        return new Sensors(pose,Json.number(m,"sampledAt"),Json.bool(m,"odometryValid"),Json.bool(m,"cameraConnected"),Json.integer(m,"ballCount"),
            new Vision(Json.bool(v,"valid"),Json.integer(v,"pipeline"),Json.number(v,"capturedAt"),targets.toArray(new Target[0])));
    }
}
