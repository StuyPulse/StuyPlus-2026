package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.util.shooter.ShotCalculator;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveFOTM extends SwerveDriveDriveWhileAligned{
    public SwerveFOTM(CommandXboxController driver) {
        super(driver, () -> ShotCalculator.solveFerryOnTheMove(swerve.getPose(), swerve.getChassisSpeeds(), 5, 0.02).virtualPose());
    }
}
