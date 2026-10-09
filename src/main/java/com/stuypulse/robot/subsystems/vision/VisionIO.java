/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.stuypulse.robot.constants.Field;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    record TargetObservation(Rotation2d tx, Rotation2d ty) {
    }

    /** Represents a robot pose sample used for pose estimation. */
    record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type) {
    }

    enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    enum MegaTagMode {
        MEGATAG_1,
        MEGATAG_2
    }

    class VisionIOOutputs {
        @AutoLogOutput(key = "Vision/Summary/MegaTagMode")
        public MegaTagMode megaTagMode = MegaTagMode.MEGATAG_1;

        @AutoLogOutput(key = "Vision/Summary/Pipeline")
        public int pipeline = 0;

        @AutoLogOutput(key = "Vision/Summary/TagWhitelist")
        public int[] aprilTagIDWhitelist = Field.ALL_TAGS;
    }

    public default void updateInputs(VisionIOInputs inputs) {
    }

    public default void applyOutputs(VisionIOOutputs outputs) {
    }
}
