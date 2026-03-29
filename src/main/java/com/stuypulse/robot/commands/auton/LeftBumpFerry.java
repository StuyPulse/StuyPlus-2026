package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;

import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.intake.IntakeSetOutpost;
import com.stuypulse.robot.commands.intake.IntakeSetFerry;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToHub;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LeftBumpFerry extends SequentialCommandGroup{

    public LeftBumpFerry(PathPlannerPath...paths){

        addCommands(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths [0])
                .alongWith(new IntakeSetIntake()),
            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new WaitCommand(2).deadlineFor(new IntakeSetFerry()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2])
                .alongWith(new IntakeSetIntake()),
            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
                new WaitCommand(2).deadlineFor(new IntakeSetOutpost()),

            new IntakeSetIntake()
        );
    }
    
}
