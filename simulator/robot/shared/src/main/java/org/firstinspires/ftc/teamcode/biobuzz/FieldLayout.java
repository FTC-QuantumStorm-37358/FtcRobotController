package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose;

/** Shared geometry and waypoints. Defaults match this simulator, not measured robot calibration. */
public final class FieldLayout {
    public final Pose[] hive={fromBrowser(-12.75,-63.945,Math.PI),fromBrowser(12.75,63.945,2*Math.PI)};
    public final Pose[] farHive={fromBrowser(-12.75,63.945,2*Math.PI),fromBrowser(12.75,-63.945,Math.PI)};
    public final Pose[] garden={fromBrowser(-58,60.8,Math.PI),fromBrowser(58,-60.8,0)};
    // Side Flowers are closer to the Garden/shot route than the opposite rear Flower.
    public final Pose[] flower={fromBrowser(-57.45,24,Math.PI/2),fromBrowser(57.45,-24,-Math.PI/2)};
    public final Pose[] loadingPark={new Pose(-62,36,0),new Pose(62,-36,Math.PI)};
    public double drivePower=1.0,turnPower=1.0;
    // BIOBUZZ TU03 p76: Red rear 30..33 / audience 34..37;
    // Blue audience 38..41 / rear 42..45. Near follows the simulator's initial Cell.
    public final int[][] nearTags={{30,31,32,33},{38,39,40,41}}, farTags={{34,35,36,37},{42,43,44,45}};
    // Nominal tag-plane heights at the two stable Hive positions. Measure on hardware.
    public double upperTagHeight=52.02,lowerTagHeight=36.57,tagHeightTolerance=1.5;
    public double tagHeightMotionTolerance=.20; // inches of drift allowed during confirmation
    public final List<double[]> obstacles=new ArrayList<double[]>();
    public FieldLayout() {
        for(double x:new double[]{-24.73,24.73})for(double y:new double[]{-15.5,15.5})obstacles.add(new double[]{x,y,1,4.9});
        obstacles.addAll(Arrays.asList(new double[]{-69,-24,2.25,2.25},new double[]{69,24,2.25,2.25},new double[]{-24,69,2.25,2.25},new double[]{24,-69,2.25,2.25}));
    }
    public static Pose fromBrowser(double x,double z,double theta) { return new Pose(x,-z,Pose.angle(theta+Math.PI/2)); }
    public boolean allowed(Pose p) {
        double t=p.heading-Math.PI/2,c=Math.cos(t),s=Math.sin(t);
        for(double x:new double[]{-8.75,8.75})for(double z:new double[]{-9.05,8.055})
            if(Math.abs(p.x+c*x+s*z)>72.001||Math.abs(-p.y-s*x+c*z)>72.001)return false;
        for(double[] b:obstacles){
            double dx=b[0]-p.x,dz=-b[1]+p.y,a=8.8,d=9.15;
            if(Math.abs(dx)<a*Math.abs(c)+d*Math.abs(s)+b[2]&&Math.abs(dz)<a*Math.abs(s)+d*Math.abs(c)+b[3]
                &&Math.abs(dx*c-dz*s)<a+b[2]*Math.abs(c)+b[3]*Math.abs(s)
                &&Math.abs(dx*s+dz*c)<d+b[2]*Math.abs(s)+b[3]*Math.abs(c))return false;
        }
        return true;
    }
    public boolean clear(Pose a,Pose b,double heading) {
        int n=Math.max(1,(int)Math.ceil(a.distance(b)));
        for(int i=0;i<=n;i++){double t=(double)i/n;if(!allowed(new Pose(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,heading)))return false;}
        return true;
    }
    public boolean canRotate(Pose p,double heading) {
        double a=Pose.angle(heading-p.heading);
        for(int i=0;i<=24;i++)if(!allowed(new Pose(p.x,p.y,p.heading+a*i/24)))return false;
        return true;
    }
    public Pose park(Pose p) {
        double t=p.heading-Math.PI/2,c=Math.abs(Math.cos(t)),s=Math.abs(Math.sin(t));
        double x=72-8.75*c-9.05*s,y=72-8.75*s-9.05*c;
        Pose best=p;double min=Double.POSITIVE_INFINITY;
        Pose[] choices={new Pose(-x,clip(p.y,y),p.heading),new Pose(x,clip(p.y,y),p.heading),new Pose(clip(p.x,x),-y,p.heading),new Pose(clip(p.x,x),y,p.heading)};
        for(Pose q:choices)if(allowed(q)&&p.distance(q)<min){best=q;min=p.distance(q);}return best;
    }
    public static double clip(double value,double max) { return Math.max(-max,Math.min(max,value)); }
}
