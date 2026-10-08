package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.*;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Outputs;

/** Same collision-aware navigator on the JVM and the robot. */
public final class Navigator {
    private final FieldLayout field;
    private int stage;
    private List<Pose> route;
    private Pose turnPose;
    public Navigator(FieldLayout field) { this.field=field; }
    public void reset() { stage=0;route=null;turnPose=null; }
    /** Read-only telemetry; callers cannot change the navigation route. */
    public List<Pose> route() { return route==null?new ArrayList<Pose>():new ArrayList<Pose>(route); }
    /** Find a reachable clearance point before turning beside any obstacle,
     * not only beside a perimeter wall. Keep the current heading on departure. */
    private Pose turnPoint(Pose start,double heading) {
        if(field.canRotate(start,heading))return start;
        List<Pose> candidates=new ArrayList<Pose>();
        for(int x=-56;x<=56;x+=4)for(int y=-56;y<=56;y+=4){
            Pose p=new Pose(x,y,start.heading);
            if(field.allowed(p)&&field.canRotate(p,heading))candidates.add(p);
        }
        candidates.sort(Comparator.comparingDouble(p->p.distance(start)));
        for(Pose p:candidates)if(plan(start,p,start.heading)!=null)return p;
        return null;
    }
    /** null means arrival. Blocked paths throw and are handled by the state machine. */
    public Outputs move(Pose p,Pose target) {
        if(stage==0){turnPose=turnPoint(p,target.heading);if(turnPose==null)throw new IllegalStateException("No reachable turn clearance");stage=p.distance(turnPose)<.01?2:1;}
        if(stage==1){
            if(p.distance(turnPose)<.6){stage=2;route=null;}else return follow(p,turnPose,turnPose.heading);}
        if(stage==2){
            if(!field.canRotate(p,target.heading))throw new IllegalStateException("Turn blocked");
            double a=Pose.angle(target.heading-p.heading);
            if(Math.abs(a)>Math.toRadians(1.5)){Outputs o=new Outputs();o.turnClockwise=-FieldLayout.clip(Math.toDegrees(a)*.03,field.turnPower);return o;}
            stage=3;route=null;
        }
        if(p.distance(target)<.65&&Math.abs(Pose.angle(target.heading-p.heading))<Math.toRadians(2))return null;
        return follow(p,target,target.heading);
    }
    private Outputs follow(Pose p,Pose target,double heading) {
        if(route==null){route=plan(p,target,heading);if(route==null)throw new IllegalStateException("No collision-free route");}
        while(route.size()>1&&(p.distance(route.get(0))<.6||field.clear(p,route.get(1),heading)))route.remove(0);
        Pose q=route.get(0);double dx=q.x-p.x,dy=q.y-p.y,c=Math.cos(p.heading),s=Math.sin(p.heading);
        Outputs o=new Outputs();o.forward=(dx*c+dy*s)*.12;o.strafeRight=(dx*s-dy*c)*.12;
        double mag=Math.hypot(o.forward,o.strafeRight);if(mag>field.drivePower){o.forward*=field.drivePower/mag;o.strafeRight*=field.drivePower/mag;}
        o.turnClockwise=-FieldLayout.clip(Math.toDegrees(Pose.angle(heading-p.heading))*.03,field.turnPower);return o;
    }
    private static final class Node {
        final Pose p;final int x,y;double cost=Double.POSITIVE_INFINITY;Node parent;
        Node(int x,int y,double h){this.x=x;this.y=y;p=new Pose(x,y,h);}
        String key(){return x+":"+y;}
    }
    public List<Pose> plan(Pose start,Pose goal,double heading) {
        if(!field.allowed(new Pose(goal.x,goal.y,heading)))return null;
        if(field.clear(start,goal,heading))return new ArrayList<Pose>(Arrays.asList(goal));
        List<Node> nodes=new ArrayList<Node>();Map<String,Node> map=new HashMap<String,Node>();
        for(int x=-64;x<=64;x+=4)for(int y=-64;y<=64;y+=4){Node n=new Node(x,y,heading);if(field.allowed(n.p)){nodes.add(n);map.put(n.key(),n);}}
        Node first=near(nodes,start,heading),last=near(nodes,goal,heading);if(first==null||last==null)return null;
        PriorityQueue<Node> open=new PriorityQueue<Node>(Comparator.comparingDouble(n->n.cost+n.p.distance(last.p)));
        Set<String> closed=new HashSet<String>();first.cost=0;open.add(first);
        int[][] offsets={{4,0},{-4,0},{0,4},{0,-4},{4,4},{4,-4},{-4,4},{-4,-4}};
        while(!open.isEmpty()){
            Node n=open.poll();if(closed.contains(n.key()))continue;
            if(n==last){List<Pose> result=new ArrayList<Pose>();result.add(goal);for(Node q=n;q!=null;q=q.parent)result.add(0,q.p);return result;}
            closed.add(n.key());
            for(int[] d:offsets){Node q=map.get((n.x+d[0])+":"+(n.y+d[1]));if(q==null||closed.contains(q.key())||!field.clear(n.p,q.p,heading))continue;
                double cost=n.cost+Math.hypot(d[0],d[1]);if(cost<q.cost){open.remove(q);q.cost=cost;q.parent=n;open.add(q);}}
        }return null;
    }
    private Node near(List<Node> nodes,Pose p,double h) {
        List<Node> sorted=new ArrayList<Node>(nodes);sorted.sort(Comparator.comparingDouble(n->n.p.distance(p)));
        for(int i=0;i<Math.min(24,sorted.size());i++)if(field.clear(p,sorted.get(i).p,h))return sorted.get(i);return null;
    }
}
