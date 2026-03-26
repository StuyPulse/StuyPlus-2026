package com.stuypulse.robot.commands.swerve.driveAligned;


import com.stuypulse.robot.constants.Field;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveDriveAlignedToHub extends SwerveDriveSetAlignment {
    public SwerveDriveAlignedToHub() {
        super(Field.getHubPose());
    }
}