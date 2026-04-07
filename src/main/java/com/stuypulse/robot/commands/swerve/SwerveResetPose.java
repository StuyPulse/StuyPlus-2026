package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveResetPose extends InstantCommand{
    private final CommandSwerveDrivetrain swerve;
    private final Pose2d newPose;

    public SwerveResetPose(Pose2d newPose) {
        swerve = CommandSwerveDrivetrain.getInstance();
        this.newPose = newPose; 

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.resetPose(newPose);
    }
}