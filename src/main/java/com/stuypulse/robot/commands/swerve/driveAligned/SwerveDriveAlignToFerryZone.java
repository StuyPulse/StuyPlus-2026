/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.constants.Field;

public class SwerveDriveAlignToFerryZone extends SwerveDriveSetAlignment{
    public SwerveDriveAlignToFerryZone() {
        super(() -> Field.getFerryZonePose(swerve.getPose().getTranslation()));
    }
}
