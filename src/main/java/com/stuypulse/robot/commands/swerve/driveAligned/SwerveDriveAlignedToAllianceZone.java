package com.stuypulse.robot.commands.swerve.driveAligned;


import com.stuypulse.robot.constants.Field;

public class SwerveDriveAlignedToAllianceZone extends SwerveDriveSetAlignment {
    public SwerveDriveAlignedToAllianceZone() {
        super(Field.getFerryZonePose(swerve.getPose().getTranslation()));
    }

    @Override
    public void execute() {
        super.execute();
    }
}