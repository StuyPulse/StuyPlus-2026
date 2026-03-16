package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.util.shooter.ShotCalculator;
import com.stuypulse.stuylib.input.Gamepad;

public class SwerveFOTM extends SwerveDriveDriveWhileAligned{
    public SwerveFOTM(Gamepad driver) {
        super(driver, () -> ShotCalculator.solveFerryOnTheMove(swerve.getPose(), swerve.getChassisSpeeds(), 5, 0.02).virtualPose());
    }

}
