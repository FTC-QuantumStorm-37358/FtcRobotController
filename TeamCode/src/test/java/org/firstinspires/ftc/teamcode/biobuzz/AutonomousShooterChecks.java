package org.firstinspires.ftc.teamcode.biobuzz;

import java.lang.reflect.Method;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;
import org.firstinspires.ftc.teamcode.biobuzz.ftc.FtcHardwareConfig;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.AutoShooterTarget;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.TableShooterCalibration;

/** Production autonomous target adapter and real routine ticks, using synthetic field/camera data. */
public final class AutonomousShooterChecks {
    private static void check(boolean ok, String why) { if (!ok) throw new AssertionError(why); }
    private static void near(double actual, double expected, String why) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-9,
                why + ": expected " + expected + ", got " + actual);
    }
    private static final class IO implements RobotIO {
        double time;
        boolean stop, showTags;
        int alliance;
        Pose pose;
        Outputs output = new Outputs();
        final FieldLayout field = new FieldLayout();
        public double nowSeconds() { return time; }
        public boolean stopRequested() { return stop; }
        public Sensors readSensors() {
            int id = field.farTags[alliance][0];
            Target[] tags = showTags ? new Target[]{new Target(id, 0, 0, field.upperTagHeight),
                    new Target(id + 1, 0, 0, field.upperTagHeight)} : new Target[0];
            return new Sensors(pose, time, true, true, 4, 4,
                    new Vision(true, output.pipeline, time, tags));
        }
        public void apply(Outputs output) { this.output = output.copy(); }
        void tick(RobotController auto) { time += .02; auto.tick(); }
    }
    private static void targetChecks() {
        FieldLayout field = new FieldLayout();
        AutonomousShooter target = new AutonomousShooter(field.hive[0]);
        TableShooterCalibration table = new TableShooterCalibration();
        near(new FtcHardwareConfig().shooterRpmMultiplier, .495, "Generic hardware default");
        near(Constants.SHOOTER_AUTO_START_MULTIPLIER, .495, "Normal hardware default");
        near(table.atPose(field.hive[0].x, field.hive[0].y,
                Math.toDegrees(field.hive[0].heading)).distanceInches, 51.55, "Actual autonomous reference distance");
        for (int xs : new int[]{-1, 1}) for (int ys : new int[]{-1, 1}) {
            Pose start = new Pose(xs * 12.75, ys * 62.9, ys * Math.PI / 2);
            near(target.command(start, .495), .495, "2970 RPM reference in all quadrants");
            near(target.targetRpm(), 2970, "Target telemetry in RPM");
            near(target.command(start, .55), .55, "Explicit reference override preserved");
            double x = xs * 36, y = ys * 57;
            double heading = Math.atan2(ys * 6 - y, xs * 12.75 - x) + Math.PI;
            Pose moved = new Pose(x, y, heading);
            double distance = Math.hypot(x - xs * 12.75, y - ys * 6) - 5.35;
            double expected = .495 + (table.rpmForDistance(distance) - table.rpmForDistance(51.55)) / 6000;
            near(target.command(moved, .495), expected, "Distance and radian heading drive automatic RPM");
            near(target.command(moved, 0), 0, "Zero configuration always disables shooting");
            near(target.command(start, .495), .495, "Return to start restores reference RPM");
        }
        for (Pose invalid : new Pose[]{null, new Pose(Double.NaN, 0, 0), new Pose(73, 0, 0)}) {
            near(target.command(invalid, .495), .495, "Existing stationary fallback now uses .495");
        }
        near(target.command(field.hive[0], Double.NaN), 0, "Invalid reference multiplier disables");
        AutoShooterTarget teleop = new AutoShooterTarget();
        teleop.updatePose(true, -12.75, -63.25, -90);
        near(teleop.getMultiplier(), .495, "TeleOp keeps its distinct reference unchanged");
        // Autonomous reference does not alter the shared lookup values.
        near(table.rpmForDistance(50), 2910, "Underlying table unchanged");
    }
    private static void routineChecks(int alliance, int role) throws Exception {
        IO io = new IO(); io.alliance = alliance;
        boolean two = role == 1;
        io.pose = io.field.roleStart(alliance, two);
        RobotController auto;
        if (two) {
            auto = new TeamTwoHiveAuto(io, io.field);
        } else if (role == 3) {
            auto = new AutoStateMachine(io, io.field, false, AutoStateMachine.Routine.TEAM_ONE);
        } else {
            TeamOnePollenAuto one = new TeamOnePollenAuto(io, io.field);
            one.useTeamTwoFollowUpShot = role == 2;
            auto = one;
        }
        auto.start(alliance, Constants.SHOOTER_AUTO_START_MULTIPLIER);
        near(io.output.shooterPower, 0, "START itself leaves motors off");
        if (two) {
            for (int n = 0; n < 10; n++) io.tick(auto);
            near(io.output.shooterPower, 0, "Team Two still waits for tag permission");
            near(io.output.feederPower, 0, "No tag cannot feed");
        }
        io.showTags = true;
        for (int n = 0; n < 100 && (io.output.shooterPower == 0 || !auto.getState().name().equals("SHOOTING")); n++) io.tick(auto);
        near(io.output.shooterPower, .495, "Role " + role + " uses calibrated default");
        near(io.output.feederPower, 0, "Spin-up still precedes feeding");
        Pose start = io.pose;
        io.pose = new Pose(start.x, start.y - Math.copySign(.3, start.y), start.heading);
        io.tick(auto);
        double expected = new AutonomousShooter(io.field.hive[0]).command(io.pose, .495);
        near(io.output.shooterPower, expected, "Active role " + role + " state " + auto.getState() + " recomputes RPM as distance changes");
        check(Math.abs(expected - .495) > 1e-6, "Moved pose actually changes the target");
        io.pose = start;
        for (int n = 0; n < 100 && io.output.feederPower == 0; n++) io.tick(auto);
        near(io.output.shooterPower, .495, "Distance target stays active through feed");
        check(io.output.feederPower > 0, "Authorized role still feeds after spin-up");
        if (role == 2) {
            // Enter the production follow-up phase directly to isolate shooter behavior from route timing.
            Method enter = TeamOnePollenAuto.class.getDeclaredMethod("enter", TeamOnePollenAuto.State.class);
            enter.setAccessible(true);
            enter.invoke(auto, TeamOnePollenAuto.State.FOLLOW_UP_SHOOTING);
            io.pose = io.field.teamTwoWallHive[alliance];
            io.tick(auto);
            near(io.output.shooterPower, .495, "Backup follow-up on opposite side uses same default");
            near(io.output.feederPower, 0, "Follow-up resets spin-up delay");
        }
        io.stop = true;
        io.tick(auto);
        near(io.output.shooterPower, 0, "STOP clears role target");
        near(io.output.feederPower, 0, "STOP clears feed");
    }
    public static void main(String[] args) throws Exception {
        targetChecks();
        for (int side = 0; side < 2; side++) for (int role = 0; role < 4; role++) routineChecks(side, role);
        System.out.println("Autonomous 0.495 references, distance/RPM updates, overrides/fallback, both sides and all shooter paths passed.");
    }
}
