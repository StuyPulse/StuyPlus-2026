package com.stuypulse.robot.commands.swerve.driveAligned;


import com.stuypulse.robot.constants.Field;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveAlignedToAllianceZone extends SwerveDriveDriveWhileAligned {
    public SwerveDriveAlignedToAllianceZone(CommandXboxController driver) {
        super(driver, () -> Field.getFerryZonePose(swerve.getPose().getTranslation()));
    }
}