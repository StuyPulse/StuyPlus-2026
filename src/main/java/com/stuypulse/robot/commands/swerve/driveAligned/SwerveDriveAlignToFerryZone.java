package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.constants.Field;

public class SwerveDriveAlignToFerryZone extends SwerveDriveSetAlignment{
    public SwerveDriveAlignToFerryZone() {
        super(Field.getFerryZonePose(swerve.getPose().getTranslation()));
    }
}
