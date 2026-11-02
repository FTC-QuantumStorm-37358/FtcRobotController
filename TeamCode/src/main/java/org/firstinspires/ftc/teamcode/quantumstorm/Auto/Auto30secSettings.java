package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import java.util.Locale;

/** Test-field geometry, team selection, and alliance-specific AprilTag firing rules. */
public final class Auto30secSettings {
    public static final int TILES_PER_SIDE = 4;
    public static final int TILE_COUNT = TILES_PER_SIDE * TILES_PER_SIDE;
    public static final double TILE_SIZE_INCHES = 2.0 * 12.0;
    public static final double FIELD_SIZE_INCHES = TILES_PER_SIDE * TILE_SIZE_INCHES;
    public static final double FIELD_HALF_SIZE_INCHES = FIELD_SIZE_INCHES / 2.0;

    public static final double OBSTACLE_STOP_INCHES = 10.0;
    public static final double RETREAT_INCHES = 3.0;
    public static final double RETREAT_POWER = 0.20;
    public static final double RETREAT_PAUSE_SECONDS = 0.15;
    public static final double RETREAT_TIMEOUT_SECONDS = 0.90;
    // 18-inch square robot: approximately 13-inch corner radius plus 2 inches clearance.
    public static final double ROBOT_CLEARANCE_INCHES = 15.0;

    public static boolean validRange(double inches) {
        return Double.isFinite(inches) && inches >= 0;
    }

    public static boolean obstacleAhead(double frontInches, double forwardPower) {
        return validRange(frontInches) && forwardPower > 0.02 && frontInches <= OBSTACLE_STOP_INCHES;
    }

    public static boolean canBackUp(double x, double y, double headingDegrees,
                                    boolean rearSensorPresent, double rearInches) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(headingDegrees)) return false;
        if (rearSensorPresent && (!validRange(rearInches) || rearInches <= OBSTACLE_STOP_INCHES)) return false;
        double limit = FIELD_HALF_SIZE_INCHES - ROBOT_CLEARANCE_INCHES;
        double heading = Math.toRadians(headingDegrees);
        double endX = x - RETREAT_INCHES * Math.cos(heading);
        double endY = y - RETREAT_INCHES * Math.sin(heading);
        return Math.abs(x) <= limit && Math.abs(y) <= limit
                && Math.abs(endX) <= limit && Math.abs(endY) <= limit;
    }

    // Coordinates are inches from field center, matching the existing odometry pose convention.
    public static boolean isOnField(double xInches, double yInches) {
        return Double.isFinite(xInches) && Double.isFinite(yInches)
                && Math.abs(xInches) <= FIELD_HALF_SIZE_INCHES
                && Math.abs(yInches) <= FIELD_HALF_SIZE_INCHES;
    }

    // Equal-sized balls appear larger when nearer; equal areas prefer less strafing.
    public static boolean preferBall(double area, double txDegrees, double bestArea, double bestTxDegrees) {
        if (!Double.isFinite(area) || area <= 0 || !Double.isFinite(txDegrees)) return false;
        if (!Double.isFinite(bestArea) || bestArea <= 0 || !Double.isFinite(bestTxDegrees)) return true;
        return area > bestArea || (area == bestArea && Math.abs(txDegrees) < Math.abs(bestTxDegrees));
    }

    public static double searchTurn(boolean visionReady, double txDegrees) {
        if (!visionReady) return 0;
        if (Double.isNaN(txDegrees)) return 0.15;
        if (!Double.isFinite(txDegrees)) return 0;
        return Math.max(-0.25, Math.min(0.25, txDegrees * 0.02));
    }

    public static double pickupForward(boolean visionReady, double txDegrees) {
        return visionReady && Double.isFinite(txDegrees) && Math.abs(txDegrees) <= 10.0 ? 0.25 : 0;
    }

    public enum TeamOption {
        RED_1("red 1 fire red", false, true),
        BLUE_1("blue 1 fire blue", true, true),
        RED_2("red 2 no fire", false, false),
        BLUE_2("blue 2 no fire", true, false);

        public final String label;
        public final boolean blue;
        public final boolean fireOnStart;

        TeamOption(String label, boolean blue, boolean fireOnStart) {
            this.label = label;
            this.blue = blue;
            this.fireOnStart = fireOnStart;
        }
    }

    private Auto30secSettings() { }

    public static TeamOption select(String team) {
        if (team == null) {
            throw new IllegalArgumentException("Team must be set");
        }
        switch (team.trim().toLowerCase(Locale.ROOT)) {
            case "red 1": case "red 1 fire red": case "red team one":
                return TeamOption.RED_1;
            case "blue 1": case "blue 1 fire blue": case "blue team one":
                return TeamOption.BLUE_1;
            case "red 2": case "red 2 no fire": case "red team two":
                return TeamOption.RED_2;
            case "blue 2": case "blue 2 no fire": case "blue team two":
                return TeamOption.BLUE_2;
            default:
                throw new IllegalArgumentException("Unknown Team: " + team);
        }
    }

    public static boolean matchesAlliance(int id, boolean blue) {
        return blue ? id >= 42 && id <= 45 : id >= 34 && id <= 37;
    }

    public static boolean mayFire(int id, boolean blue, double tx, double pitch,
                                  boolean valid, boolean connected, int actualPipeline,
                                  int expectedPipeline, long ageMs) {
        return valid && connected && actualPipeline == expectedPipeline
                && ageMs >= 0 && ageMs <= 250 && matchesAlliance(id, blue)
                && Double.isFinite(tx) && Math.abs(tx) <= 10.0
                && Double.isFinite(pitch) && Math.abs(pitch) <= 10.0;
    }
}
