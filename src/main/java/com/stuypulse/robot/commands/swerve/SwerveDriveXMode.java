package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveXMode extends Command{
    private CommandSwerveDrivetrain swerve;

    public SwerveDriveXMode() {
        swerve = CommandSwerveDrivetrain.getInstance();
    }

    @Override
    public void initialize() {
        SwerveRequest request = new SwerveRequest.SwerveDriveBrake();
        swerve.setControl(request);
    }
}