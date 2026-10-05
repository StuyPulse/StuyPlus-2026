package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoMeterPath extends SequentialCommandGroup {

    public TwoMeterPath(PathPlannerPath... paths) {
        addCommands(new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
                Swerve.getInstance().followPathCommand(paths[0]));
    }
}
