/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import com.stuypulse.robot.util.vision.Camera;

public class Cameras {
        public static final Camera[] LimelightCameras = new Camera[] {
                new Camera("limelight",
                                new Pose3d(
                                        Units.inchesToMeters(-12.109),
                                        Units.inchesToMeters(-7.129),
                                        Units.inchesToMeters(8.375), // TODO: get actual camera info
                                new Rotation3d(
                                        Units.degreesToRadians(180),
                                        Units.degreesToRadians(28),
                                        Units.degreesToRadians(180))))
        };
}
