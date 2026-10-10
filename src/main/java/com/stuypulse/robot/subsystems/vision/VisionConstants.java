/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.*;

public interface VisionConstants {
    public interface VisionSettings {
        int RESET_IMU_INDEX = 1;

        // Basic filtering thresholds
        double MAX_AMBIGUITY = 0.3;
        double MAX_Z_ERROR = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
        double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

        // Multipliers to apply for MegaTag 2 observations
        double LINEAR_STD_DEV_MEGATAG_2_FACTOR = 0.5; // More stable than full 3D solve
        double ANGULAR_STD_DEV_MEGATAG_2_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available

        double BUZZ_DEBOUNCE = 0.25;
        Time HDR_TIMEOUT = Seconds.of(0.5);
    }

    record CameraData(String name, Transform3d robotToCamera, double stdDevFactor) {
    }

    public enum Cameras {
        FRONT(
                "limelight-front",
                new Transform3d(
                        Inches.zero(),
                        Inches.zero(),
                        Inches.of(26.1),
                        new Rotation3d(
                                Degrees.zero(),
                                Degrees.of(9.764),
                                Degrees.zero())),
                1.0),
        BACK(
                "limelight-back",
                new Transform3d(
                        Inches.of(-12.109),
                        Inches.of(-7.129),
                        Inches.of(8.375),
                        new Rotation3d(
                                Degrees.of(180),
                                Degrees.of(28),
                                Degrees.of(180))),
                1.0);

        private final CameraData data;

        private Cameras(String name, Transform3d robotToCamera, double stdDevFactor) {
            this.data = new CameraData(name, robotToCamera, stdDevFactor);
        }

        public String getName() {
            return data.name();
        }

        public Transform3d getRobotToCamera() {
            return data.robotToCamera();
        }

        public double getStdDevFactor() {
            return data.stdDevFactor();
        }

        public CameraData getData() {
            return data;
        }
    }

    public enum Pipelines {
        LOW_SUN(0),
        HIGH_SUN(1);

        private final int index;

        private Pipelines(int index) {
            this.index = index;
        }

        public int getIndex() {
            return index;
        }
    }
}
