package com.stuypulse.robot.commands.swerve.driveAligned;


import com.stuypulse.robot.constants.Field;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveWhileAlignedToHub extends SwerveDriveDriveWhileAligned {
    public SwerveDriveWhileAlignedToHub(CommandXboxController driver) {
        super(driver, () -> Field.getHubPose());
    }
}