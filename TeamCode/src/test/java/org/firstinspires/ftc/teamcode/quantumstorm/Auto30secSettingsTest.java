package org.firstinspires.ftc.teamcode.quantumstorm;

import org.firstinspires.ftc.teamcode.quantumstorm.Auto.Auto30secSettings;

public final class Auto30secSettingsTest {
    public static void main(String[] args) {
        for (double range : new double[] {0, 5, 10}) {
            check(Auto30secSettings.obstacleAhead(range, 0.25), "Close obstacle while moving forward");
            check(!Auto30secSettings.obstacleAhead(range, 0), "Stationary detection does not retreat");
            check(!Auto30secSettings.obstacleAhead(range, -0.25), "Front sensor does not block reversing");
        }
        check(!Auto30secSettings.obstacleAhead(10.001, 0.25), "Clear front range");
        for (double invalid : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY}) {
            check(!Auto30secSettings.validRange(invalid), "Invalid configured range must stop");
            check(!Auto30secSettings.canBackUp(0, 0, 0, true, invalid), "Invalid rear range blocks retreat");
        }
        check(Auto30secSettings.canBackUp(0, 0, 0, false, Double.NaN), "Optional rear sensor");
        check(Auto30secSettings.canBackUp(0, 0, 0, true, 10.001), "Clear rear allows retreat");
        check(!Auto30secSettings.canBackUp(0, 0, 0, true, 10), "Close rear blocks retreat");
        check(Auto30secSettings.canBackUp(32, 0, 0, false, Double.NaN), "Reverse away from right wall");
        check(!Auto30secSettings.canBackUp(-32, 0, 0, false, Double.NaN), "Reverse toward left wall blocked");
        check(!Auto30secSettings.canBackUp(32, 0, 180, false, Double.NaN), "Heading rotates reverse vector");
        check(Auto30secSettings.canBackUp(0, 32, 90, false, Double.NaN), "Reverse away from upper wall");
        check(!Auto30secSettings.canBackUp(0, 32, -90, false, Double.NaN), "Reverse toward upper wall blocked");
        check(!Auto30secSettings.canBackUp(34, 0, 0, false, Double.NaN), "Robot clearance enforced");
        check(!Auto30secSettings.canBackUp(Double.NaN, 0, 0, false, Double.NaN), "Invalid pose blocks retreat");
        check(!Auto30secSettings.canBackUp(0, 0, Double.NaN, false, Double.NaN), "Invalid heading blocks retreat");
        check(Auto30secSettings.preferBall(4, 20, 1, 0), "Nearer off-center ball beats farther centered ball");
        check(!Auto30secSettings.preferBall(1, 0, 4, 20), "Farther centered ball does not replace nearer ball");
        check(Auto30secSettings.preferBall(4, -2, 4, 10), "Equal area prefers centered ball");
        check(!Auto30secSettings.preferBall(4, 10, 4, -2), "Equal area keeps more centered ball");
        check(!Auto30secSettings.preferBall(4, 2, 4, -2), "Identical scores keep first target");
        check(Auto30secSettings.preferBall(1, 0, Double.NaN, Double.NaN), "First valid target selected");
        for (double invalid : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            check(!Auto30secSettings.preferBall(invalid, 0, Double.NaN, Double.NaN), "Reject invalid area");
        }
        check(!Auto30secSettings.preferBall(4, Double.NaN, 1, 0), "Reject invalid angle");
        check(Auto30secSettings.searchTurn(true, Double.NaN) == 0.15, "Slow no-target search");
        check(Auto30secSettings.searchTurn(false, Double.NaN) == 0, "No search on stale/disconnected/wrong pipeline");
        check(Auto30secSettings.searchTurn(true, -30) == -0.25, "Left turn capped");
        check(Auto30secSettings.searchTurn(true, 30) == 0.25, "Right turn capped");
        check(Auto30secSettings.searchTurn(true, 0) == 0, "Centered target needs no turn");
        check(Auto30secSettings.searchTurn(true, Double.POSITIVE_INFINITY) == 0, "Invalid angle stops turn");
        for (double tx : new double[] {-10.001, -10, 0, 10, 10.001, Double.NaN, Double.POSITIVE_INFINITY}) {
            for (boolean ready : new boolean[] {false, true}) {
                check(Auto30secSettings.pickupForward(ready, tx)
                        == (ready && Double.isFinite(tx) && Math.abs(tx) <= 10 ? 0.25 : 0),
                        "Approach only fresh centered-enough target");
            }
        }
        check(Auto30secSettings.TILES_PER_SIDE == 4, "Four tiles per side");
        check(Auto30secSettings.TILE_COUNT == 16, "Sixteen tiles total");
        check(Auto30secSettings.TILE_SIZE_INCHES == 24, "Two-foot tile");
        check(Auto30secSettings.FIELD_SIZE_INCHES == 96, "Eight-foot field");
        for (double x : new double[] {-48.001, -48, 0, 48, 48.001, Double.NaN, Double.POSITIVE_INFINITY}) {
            for (double y : new double[] {-48.001, -48, 0, 48, 48.001, Double.NaN, Double.NEGATIVE_INFINITY}) {
                check(Auto30secSettings.isOnField(x, y)
                        == (Double.isFinite(x) && Double.isFinite(y) && Math.abs(x) <= 48 && Math.abs(y) <= 48),
                        "Centered field bounds");
            }
        }
        for (Auto30secSettings.TeamOption team : Auto30secSettings.TeamOption.values()) {
            check(Auto30secSettings.select(team.label) == team, "Label mapping");
            check(team.fireOnStart == team.name().endsWith("1"), "Only team 1 overrides vision");
            check(team.blue == team.name().startsWith("BLUE"), "Alliance");
        }
        check(Auto30secSettings.select(" RED 1 ").fireOnStart, "Short red 1 string");
        check(Auto30secSettings.select("blue 1").fireOnStart, "Short blue 1 string");
        check(!Auto30secSettings.select("Red Team Two").fireOnStart, "Legacy team 2");
        for (String invalid : new String[] {null, "", "green 1"}) {
            try {
                Auto30secSettings.select(invalid);
                throw new AssertionError("Invalid team accepted");
            } catch (IllegalArgumentException expected) { }
        }
        for (boolean blue : new boolean[] {false, true}) {
            for (int id = 29; id <= 46; id++) {
                for (double tx : new double[] {-10.001, -10, 0, 10, 10.001, Double.NaN, Double.POSITIVE_INFINITY}) {
                    for (double pitch : new double[] {-10.001, -10, 0, 10, 10.001, Double.NaN}) {
                        boolean expected = (blue ? id >= 42 && id <= 45 : id >= 34 && id <= 37)
                                && Double.isFinite(tx) && Math.abs(tx) <= 10
                                && Double.isFinite(pitch) && Math.abs(pitch) <= 10;
                        check(fire(id, blue, tx, pitch, true, true, 0, 250) == expected, "Inclusive angle/alliance gate");
                    }
                }
            }
            int id = blue ? 42 : 34;
            check(!fire(id, blue, 0, 0, false, true, 0, 0), "Invalid result");
            check(!fire(id, blue, 0, 0, true, false, 0, 0), "Disconnected camera");
            check(!fire(id, blue, 0, 0, true, true, 1, 0), "Wrong pipeline");
            check(!fire(id, blue, 0, 0, true, true, 0, 251), "Stale result");
            check(!fire(id, blue, 0, 0, true, true, 0, -1), "Invalid age");
            check(fire(id, blue, 0, 0, true, true, 0, 0), "Target recovered");
        }
        System.out.println("Auto30secSettingsTest: all checks passed");
    }

    private static boolean fire(int id, boolean blue, double tx, double pitch,
                                boolean valid, boolean connected, int pipeline, long age) {
        return Auto30secSettings.mayFire(id, blue, tx, pitch, valid, connected, pipeline, 0, age);
    }

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
}
