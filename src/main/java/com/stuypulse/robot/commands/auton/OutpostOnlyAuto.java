package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.intake.IntakeSetOuttake;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OutpostOnlyAuto extends SequentialCommandGroup{

    public OutpostOnlyAuto(PathPlannerPath...paths){
        addCommands(
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new WaitCommand(20).deadlineFor(new IntakeSetOuttake())
            );
    }   
}