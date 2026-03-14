package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.feeder.FeederForward;
import com.stuypulse.robot.commands.feeder.FeederIdle;
import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToHub;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Outpost extends SequentialCommandGroup{
    
    public Outpost(PathPlannerPath... paths) {
        addCommands(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                .alongWith(new IntakeSetIntake()),
            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]) 
                .alongWith(new IntakeSetIdle()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new SwerveDriveAlignedToHub(),
                new WaitCommand(Settings.Shooter.SHOOT_TIME_AUTO).deadlineFor(new FeederForward(), new IntakeAgitateOnce().repeatedly()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3])
                .alongWith(new FeederIdle(), new IntakeSetIdle()),
                new WaitCommand(2), // Get balls at outpost

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
                new SwerveDriveAlignedToHub(),
                new WaitCommand(Settings.Shooter.SHOOT_TIME_AUTO).deadlineFor(new FeederForward(), new IntakeAgitateOnce().repeatedly()),

            new FeederIdle().alongWith(new IntakeSetIdle())
        );
    }
}
