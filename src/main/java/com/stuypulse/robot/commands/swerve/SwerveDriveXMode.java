/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class SwerveDriveXMode extends Command {

    private CommandSwerveDrivetrain swerve;

    public SwerveDriveXMode() {
        swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SwerveRequest request = new SwerveRequest.SwerveDriveBrake();
        swerve.setControl(request);
    }
}
