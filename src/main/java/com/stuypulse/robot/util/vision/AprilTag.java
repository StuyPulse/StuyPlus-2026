/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.vision;

import edu.wpi.first.math.geometry.Pose3d;

public record AprilTag(int id, Pose3d location) {
}