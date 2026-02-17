package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.commands.swerve.SwerveDriveGetAlignment;
import com.stuypulse.robot.constants.Field;


public class SwerveDriveAlignedToHub extends SwerveDriveGetAlignment {
    public SwerveDriveAlignedToHub() {
        super(Field.getHubPose());
    }

    @Override
    public void execute() {
        super.execute();
    }
}