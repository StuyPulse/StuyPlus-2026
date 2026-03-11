package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.util.shooter.ShotCalculator;
import com.stuypulse.stuylib.input.Gamepad;

public class SwerveSOTM extends SwerveDriveDriveWhileAligned {
    public SwerveSOTM(Gamepad driver) {
        super(driver, ShotCalculator.solveShootOnTheMove(swerve.getPose(), Field.getHubPose(), swerve.getChassisSpeeds(), 5, 0.02).virtualPose());
    }
}