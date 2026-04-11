package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.util.shooter.ShotCalculator;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveSOTM extends SwerveDriveDriveWhileAligned {
    public SwerveSOTM(CommandXboxController driver) {
        super(driver, () -> ShotCalculator.solveShootOnTheMove(swerve.getPose(), swerve.getChassisSpeeds(), 5, 0.02).virtualPose());
    }

}