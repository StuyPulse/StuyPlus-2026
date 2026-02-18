package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveGetAlignment;
import com.stuypulse.robot.constants.Field;

public class SwerveDriveAlignedToAllianceZone extends SwerveDriveGetAlignment {
    public SwerveDriveAlignedToAllianceZone() {
        super(Field.getFerryZonePose(instance.getPose().getTranslation()));
    }

    @Override
    public void execute() {
        super.execute();
    }
}