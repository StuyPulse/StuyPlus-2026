package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LBDisrupt extends SequentialCommandGroup {

    public LBDisrupt(PathPlannerPath... paths) {
        addCommands(new SwerveResetPose(paths[0].getStartingHolonomicPose().get()), CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]), CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]), CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]), CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]));
    }
}
