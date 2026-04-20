package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.constants.Field;

public class SwerveDriveAlignToHub extends SwerveDriveSetAlignment{
    public SwerveDriveAlignToHub() {
        super(Field.getHubPose());
    }
}
