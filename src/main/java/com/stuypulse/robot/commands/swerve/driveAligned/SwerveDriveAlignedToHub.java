package com.stuypulse.robot.commands.swerve.driveAligned;


import com.stuypulse.robot.constants.Field;

public class SwerveDriveAlignedToHub extends SwerveDriveSetAlignment {
    public SwerveDriveAlignedToHub() {
        super(Field.getHubPose());
    }
}