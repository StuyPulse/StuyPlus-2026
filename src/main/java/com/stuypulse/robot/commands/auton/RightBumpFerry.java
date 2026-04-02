package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;

import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.intake.IntakeSetOuttake;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RightBumpFerry extends SequentialCommandGroup{
    public RightBumpFerry(PathPlannerPath...paths){
        addCommands(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                .alongWith(new IntakeSetIntake()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new WaitCommand(2).deadlineFor(new IntakeSetOuttake()),
            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2])
                .alongWith(new IntakeSetIntake()),
            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
                new WaitCommand(2).deadlineFor(new IntakeSetOuttake()),

            new IntakeSetIntake()
        );
    }
}
