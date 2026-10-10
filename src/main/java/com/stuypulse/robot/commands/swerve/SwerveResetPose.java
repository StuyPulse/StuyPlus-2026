/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveResetPose extends InstantCommand {
    public SwerveResetPose(Pose2d newPose) {
        super(() -> Swerve.getInstance().resetPose(newPose));
    }
}
