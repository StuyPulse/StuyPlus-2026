/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Cameras {
    public static final Camera[] LimelightCameras = new Camera[] {
            new Camera("shooter_front_facing",
                    new Pose3d(
                            Units.inchesToMeters(0),
                            Units.inchesToMeters(0),
                            Units.inchesToMeters(26.1),
                            new Rotation3d(
                                    Units.degreesToRadians(0),
                                    Units.degreesToRadians(9.764),
                                    Units.degreesToRadians(0))))
    };

    public static record Camera(String name, Pose3d location) {};
}