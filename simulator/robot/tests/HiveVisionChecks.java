package org.firstinspires.ftc.teamcode.biobuzz;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

public final class HiveVisionChecks {
    private static void check(boolean ok,String reason){if(!ok)throw new AssertionError(reason);}
    private static Vision frame(double time,int first,double height){return new Vision(true,0,time,
        new Target[]{new Target(first,-2,0,height),new Target(first+1,2,0,height)});}
    private static void confirm(HiveVision v,int first,double height,double time){
        for(int i=0;i<4;i++){double t=time+i*.04;v.update(frame(t,first,height),t);}
    }
    public static void main(String[]args){
        FieldLayout f=new FieldLayout();
        for(int alliance=0;alliance<2;alliance++){
            HiveVision v=new HiveVision(f,alliance);int near=f.nearTags[alliance][0],far=f.farTags[alliance][0];
            confirm(v,f.nearTags[1-alliance][0],f.upperTagHeight,0);
            check(v.state(.12)==-1&&v.aim(.12,false)==null,"opponent must not authorize shot/state");
            for(int i=0;i<20;i++)v.update(frame(.2,near,f.upperTagHeight),.2+i*.001);
            check(v.state(.22)==-1,"same frame must not confirm state");
            confirm(v,near,f.upperTagHeight,.3);
            check(v.state(.42)==0,"normal state");check(Math.abs(v.aim(.42,false).tx)<1e-9,"aim uses own cluster mean");
            check(v.aim(.42,true)==null,"wrong shooting side");
            v.update(new Vision(true,0,.46,new Target[]{new Target(near,0,0,f.upperTagHeight-.4),new Target(near+1,0,0,f.upperTagHeight-.4)}),.46);
            check(v.state(.46)==-1,"motion within height band must cancel confirmation");
            confirm(v,far,f.upperTagHeight,.5);
            check(v.state(.62)==1&&v.aim(.62,true)!=null,"flipped state");
            confirm(v,near,f.upperTagHeight,.7);
            check(v.state(.82)==0,"second tip must switch back");
            v.update(new Vision(true,0,.86,new Target[]{new Target(near,0,0,f.upperTagHeight),new Target(near+1,0,0,f.upperTagHeight),new Target(far,0,0,f.upperTagHeight),new Target(far+1,0,0,f.upperTagHeight)}),.86);
            check(v.state(.86)==-1,"contradictory cells");
            confirm(v,near,f.lowerTagHeight,.9);
            check(v.state(1.02)==1&&v.aim(1.02,true)==null,"down Cell determines state but cannot aim unseen up Cell");
            check(v.state(1.4)==-1,"stale state must expire");
            confirm(v,near,Double.NaN,1.5);check(v.state(1.62)==-1,"missing calibrated pose");
            v.update(new Vision(true,1,1.7,new Target[]{new Target(near,0,0,f.upperTagHeight)}),1.7);check(v.state(1.7)==-1,"wrong pipeline");
            confirm(v,near,f.upperTagHeight,1.8);
            v.update(frame(1.85,far,f.upperTagHeight),1.94);check(v.state(1.94)==-1,"older frame cannot restore state");
        }
        System.out.println("Both alliances: own-cluster aim, fresh-frame debounce, calibrated heights, motion rejection, flip/back, stale/conflicting/missing-pose rejection passed.");
    }
}
