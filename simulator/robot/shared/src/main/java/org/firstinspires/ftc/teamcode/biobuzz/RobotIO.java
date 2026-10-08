package org.firstinspires.ftc.teamcode.biobuzz;

/** The only boundary between autonomous decisions and hardware/simulation. */
public interface RobotIO {
    double nowSeconds();
    Sensors readSensors();
    void apply(Outputs outputs);

    /** Inches, field X/Y, radians counterclockwise from +X. */
    final class Pose {
        public final double x, y, heading;
        public Pose(double x, double y, double heading) { this.x=x; this.y=y; this.heading=heading; }
        public double distance(Pose other) { return Math.hypot(x-other.x,y-other.y); }
        public static double angle(double a) { return Math.atan2(Math.sin(a),Math.cos(a)); }
        public boolean finite() { return Double.isFinite(x)&&Double.isFinite(y)&&Double.isFinite(heading); }
    }
    final class Target {
        public final int id;
        /** Limelight convention: tx is degrees, right/clockwise positive. */
        public final double tx, ty;
        public Target(int id,double tx,double ty) { this.id=id; this.tx=tx; this.ty=ty; }
    }
    final class Vision {
        public final boolean valid;
        public final int pipeline;
        public final double capturedAt;
        public final Target[] targets;
        public Vision(boolean valid,int pipeline,double capturedAt,Target[] targets) {
            this.valid=valid; this.pipeline=pipeline; this.capturedAt=capturedAt; this.targets=targets.clone();
        }
        public boolean fresh(double now,int expectedPipeline) {
            return valid&&pipeline==expectedPipeline&&now-capturedAt>=0&&now-capturedAt<=.25;
        }
        public static Vision empty(int pipeline,double now) { return new Vision(false,pipeline,now,new Target[0]); }
    }
    final class Sensors {
        public final Pose pose;
        public final double sampledAt;
        public final boolean odometryValid, cameraConnected;
        /** -1 means unavailable; never manufacture a full/empty count. */
        public final int ballCount;
        public final Vision vision;
        public Sensors(Pose pose,double sampledAt,boolean odometryValid,boolean cameraConnected,int ballCount,Vision vision) {
            this.pose=pose; this.sampledAt=sampledAt; this.odometryValid=odometryValid;
            this.cameraConnected=cameraConnected; this.ballCount=ballCount; this.vision=vision;
        }
    }
    final class Outputs {
        /** Robot relative forward, right strafe, clockwise turn; normalized powers. */
        public double forward, strafeRight, turnClockwise;
        public double shooterPower, feederPower, intakePower;
        /** 0 = hive AprilTags / camera up, 1 = balls / camera down. */
        public int pipeline;
        /** Front-left, front-right, back-left, back-right, after mecanum normalization. */
        public double[] wheelPowers() {
            double fl=forward+strafeRight+turnClockwise,fr=forward-strafeRight-turnClockwise,
                bl=forward-strafeRight+turnClockwise,br=forward+strafeRight-turnClockwise;
            double scale=Math.max(1,Math.max(Math.max(Math.abs(fl),Math.abs(fr)),Math.max(Math.abs(bl),Math.abs(br))));
            return new double[]{fl/scale,fr/scale,bl/scale,br/scale};
        }
        public Outputs copy() {
            Outputs o=new Outputs(); o.forward=forward; o.strafeRight=strafeRight; o.turnClockwise=turnClockwise;
            o.shooterPower=shooterPower; o.feederPower=feederPower; o.intakePower=intakePower; o.pipeline=pipeline; return o;
        }
    }
}
