package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * The 4 autonomous setups: alliance color + starting position.
 * Each one bundles the field locations that routine drives to.
 * Values come from Constants.
 */
public enum AutoStartPosition {

    RED_1("Red Team One", Constants.RED_1_START_POSE, Constants.RED_1_HIVE1_POSE,
            Constants.RED_1_HIVE1_FLIPPED_POSE,
            Constants.RED_GARDEN_POSE, Constants.RED_FLOWER_POSE, Constants.RED_PARK_POSE),

    RED_2("Red Team Two", Constants.RED_2_START_POSE, Constants.RED_2_HIVE1_POSE,
            Constants.RED_2_HIVE1_FLIPPED_POSE,
            Constants.RED_GARDEN_POSE, Constants.RED_FLOWER_POSE, Constants.RED_PARK_POSE),

    BLUE_1("Blue Team One", Constants.BLUE_1_START_POSE, Constants.BLUE_1_HIVE1_POSE,
            Constants.BLUE_1_HIVE1_FLIPPED_POSE,
            Constants.BLUE_GARDEN_POSE, Constants.BLUE_FLOWER_POSE, Constants.BLUE_PARK_POSE),

    BLUE_2("Blue Team Two", Constants.BLUE_2_START_POSE, Constants.BLUE_2_HIVE1_POSE,
            Constants.BLUE_2_HIVE1_FLIPPED_POSE,
            Constants.BLUE_GARDEN_POSE, Constants.BLUE_FLOWER_POSE, Constants.BLUE_PARK_POSE);

    public final String label;
    public final Pose2D start;
    public final Pose2D hive1;          // same-color hive closest to this start
    public final Pose2D hive1Flipped;   // shooting spot for Hive 1's far face after it tips
    public final Pose2D garden;
    public final Pose2D flower;
    public final Pose2D park;

    AutoStartPosition(String label, Pose2D start, Pose2D hive1, Pose2D hive1Flipped,
                      Pose2D garden, Pose2D flower, Pose2D park) {
        this.label = label;
        this.start = start;
        this.hive1 = hive1;
        this.hive1Flipped = hive1Flipped;
        this.garden = garden;
        this.flower = flower;
        this.park = park;
    }
}
