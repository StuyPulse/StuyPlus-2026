/************************ PROJECT KITBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.util.vision.AprilTag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

import java.util.ArrayList;
import java.util.List;

/** This interface stores information about the field elements. */
public interface Field {

    public static final Field2d FIELD2D = new Field2d();

    double WIDTH = Units.inchesToMeters(317.000); 
    double LENGTH = Units.inchesToMeters(690.876);

    /*** APRILTAGS ***/

    enum NamedTags {
        RED_RIGHT_TRENCH_NZ,        // #1
        RED_HUB_RIGHT_SIDE_MID,
        RED_HUB_BACK_SIDE_LEFT,
        RED_HUB_BACK_SIDE_MID,
        RED_HUB_LEFT_SIDE_MID,      // #5
        RED_LEFT_TRENCH_NZ,
        RED_LEFT_TRENCH_AZ,
        RED_HUB_LEFT_SIDE_RIGHT,
        RED_HUB_FRONT_SIDE_LEFT,
        RED_HUB_FRONT_SIDE_MID,     // #10
        RED_HUB_RIGHT_SIDE_LEFT,
        RED_RIGHT_TRENCH_AZ,
        RED_HP_MID,
        RED_HP_RIGHT,
        RED_TOWER_MID,              // #15
        RED_TOWER_RIGHT,
        BLUE_RIGHT_TRENCH_NZ,
        BLUE_HUB_RIGHT_SIDE_MID,
        BLUE_HUB_BACK_SIDE_LEFT,
        BLUE_HUB_BACK_SIDE_MID,     // #20
        BLUE_HUB_LEFT_SIDE_MID,
        BLUE_LEFT_TRENCH_NZ,
        BLUE_LEFT_TRENCH_AZ,
        BLUE_HUB_LEFT_SIDE_RIGHT,
        BLUE_HUB_FRONT_SIDE_LEFT,   // #25
        BLUE_HUB_FRONT_SIDE_MID,
        BLUE_HUB_RIGHT_SIDE_LEFT,
        BLUE_RIGHT_TRENCH_AZ,
        BLUE_HP_MID,
        BLUE_HP_RIGHT,              // #30
        BLUE_TOWER_MID,
        BLUE_TOWER_RIGHT;

        public final AprilTag tag;

        public int getID() {
            return tag.getID();
        }

        public Pose3d getLocation() {
            return Robot.isBlue()
                ? tag.getLocation()
                : transformToOppositeAlliance(tag.getLocation());
        }

        private NamedTags() {
            tag = APRILTAGS[ordinal()];
        }
    }

    AprilTag APRILTAGS[] = {
        // 2026 Field AprilTag Layout
        new AprilTag(1,  new Pose3d(new Translation3d(Units.inchesToMeters(467.64), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))), 
        new AprilTag(2,  new Pose3d(new Translation3d(Units.inchesToMeters(469.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
        new AprilTag(3,  new Pose3d(new Translation3d(Units.inchesToMeters(445.35), Units.inchesToMeters(172.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(4,  new Pose3d(new Translation3d(Units.inchesToMeters(445.35), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(5,  new Pose3d(new Translation3d(Units.inchesToMeters(469.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(6,  new Pose3d(new Translation3d(Units.inchesToMeters(467.64), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(7,  new Pose3d(new Translation3d(Units.inchesToMeters(470.59), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(8,  new Pose3d(new Translation3d(Units.inchesToMeters(483.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(9,  new Pose3d(new Translation3d(Units.inchesToMeters(492.88), Units.inchesToMeters(144.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(10,  new Pose3d(new Translation3d(Units.inchesToMeters(492.88), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(11,  new Pose3d(new Translation3d(Units.inchesToMeters(483.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
        new AprilTag(12,  new Pose3d(new Translation3d(Units.inchesToMeters(470.59), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(13,  new Pose3d(new Translation3d(Units.inchesToMeters(650.92), Units.inchesToMeters(291.47), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(14,  new Pose3d(new Translation3d(Units.inchesToMeters(650.92), Units.inchesToMeters(274.47), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(15,  new Pose3d(new Translation3d(Units.inchesToMeters(650.90), Units.inchesToMeters(170.22), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(16,  new Pose3d(new Translation3d(Units.inchesToMeters(650.90), Units.inchesToMeters(153.22), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(17,  new Pose3d(new Translation3d(Units.inchesToMeters(183.59), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(18,  new Pose3d(new Translation3d(Units.inchesToMeters(182.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(19,  new Pose3d(new Translation3d(Units.inchesToMeters(205.87), Units.inchesToMeters(144.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(20,  new Pose3d(new Translation3d(Units.inchesToMeters(205.87), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(21,  new Pose3d(new Translation3d(Units.inchesToMeters(182.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
        new AprilTag(22,  new Pose3d(new Translation3d(Units.inchesToMeters(183.59), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(23,  new Pose3d(new Translation3d(Units.inchesToMeters(180.64), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(24,  new Pose3d(new Translation3d(Units.inchesToMeters(168.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
        new AprilTag(25,  new Pose3d(new Translation3d(Units.inchesToMeters(158.34), Units.inchesToMeters(172.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(26,  new Pose3d(new Translation3d(Units.inchesToMeters(158.34), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(27,  new Pose3d(new Translation3d(Units.inchesToMeters(168.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(28,  new Pose3d(new Translation3d(Units.inchesToMeters(180.64), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(29,  new Pose3d(new Translation3d(Units.inchesToMeters(0.30), Units.inchesToMeters(26.22), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(30,  new Pose3d(new Translation3d(Units.inchesToMeters(0.30), Units.inchesToMeters(43.22), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(31,  new Pose3d(new Translation3d(Units.inchesToMeters(0.32), Units.inchesToMeters(147.47), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(32,  new Pose3d(new Translation3d(Units.inchesToMeters(0.32), Units.inchesToMeters(164.47), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))))
    };

    public static boolean isValidTag(int id) {
        for (AprilTag tag : APRILTAGS) {
            if (tag.getID() == id) {
                return true;
            }
        }
        return false;
    }

    public static AprilTag getTag(int id) {
        for (AprilTag tag : APRILTAGS) {
            if (tag.getID() == id) {
                return tag;
            }
        }
        return null;
    }

    public final int[] RED_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
    public final int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};
    public final int[] RED_TRENCH_TAG_IDS = {1, 6, 7, 12};
    public final int[] BLUE_TRENCH_TAG_IDS = {17, 22, 23, 28};
    public final int[] RED_TOWER_TAG_IDS = {15, 16};
    public final int[] BLUE_TOWER_TAG_IDS = {31, 32};
    public final int[] RED_HP_TAG_IDS = {13, 14};
    public final int[] BLUE_HP_TAG_IDS = {29, 30};

    public final Pose2d blueHubCenter = new Pose2d(Units.inchesToMeters(158.60), Units.inchesToMeters(WIDTH / 2.0), new Rotation2d());

    public static Pose2d getAllianceHubPose() {
        return (Robot.isBlue() ? blueHubCenter : transformToOppositeAlliance(blueHubCenter));
    }

    /* TRANSFORM FUNCTIONS */
    
    public static Pose3d transformToOppositeAlliance(Pose3d pose) {
        Pose3d rotated = pose.rotateBy(new Rotation3d(0, 0, Math.PI));

        return new Pose3d(
            rotated.getTranslation().plus(new Translation3d(LENGTH, WIDTH, 0)),
            rotated.getRotation());
    }

    public static Pose2d transformToOppositeAlliance(Pose2d pose) {
        Pose2d rotated = pose.rotateBy(Rotation2d.fromDegrees(180));
        return new Pose2d(
            rotated.getTranslation().plus(new Translation2d(LENGTH, WIDTH)),
            rotated.getRotation());
    }
    
    public static Translation2d transformToOppositeAlliance(Translation2d translation) {
        return new Translation2d(LENGTH - translation.getX(), WIDTH - translation.getY());
    }

    public static List<Pose2d> transformToOppositeAlliance(List<Pose2d> poses) {
        List<Pose2d> newPoses = new ArrayList<>();
        for (Pose2d pose : poses) {
            newPoses.add(transformToOppositeAlliance(pose));
        }
        return newPoses;
    }

    /**** EMPTY FIELD POSES ****/

    Pose2d EMPTY_FIELD_POSE2D = new Pose2d(new Translation2d(-1, -1), new Rotation2d());
    Pose3d EMPTY_FIELD_POSE3D = new Pose3d(-1, -1, 0, new Rotation3d());

    public static void clearFieldObject(FieldObject2d fieldObject)  {
        fieldObject.setPose(EMPTY_FIELD_POSE2D);
    }
}