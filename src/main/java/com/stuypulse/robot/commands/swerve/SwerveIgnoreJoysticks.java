package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveIgnoreJoysticks extends InstantCommand{
    public SwerveIgnoreJoysticks() {
        super(() -> Swerve.getInstance().shouldIgnoreJoysticks = true);
    }
}
