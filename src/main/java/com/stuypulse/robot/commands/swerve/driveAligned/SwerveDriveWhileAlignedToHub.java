/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve.driveAligned;


import com.stuypulse.robot.constants.Field;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveWhileAlignedToHub extends SwerveDriveDriveWhileAligned {
    public SwerveDriveWhileAlignedToHub(CommandXboxController driver) {
        super(driver, () -> Field.getHubPose());
    }
}