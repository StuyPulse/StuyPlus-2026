package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveUseJoysticks extends InstantCommand{
    public SwerveUseJoysticks() {
        super(() -> Swerve.getInstance().shouldIgnoreJoysticks = false);
    }
}
