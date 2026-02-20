package com.stuypulse.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.feeder.FeederForward;
import com.stuypulse.robot.commands.feeder.FeederIdle;
import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToHub;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;


public class LeftBumpOnePointFiveCycle extends SequentialCommandGroup{
    public LeftBumpOnePointFiveCycle(PathPlannerPath... paths) {
        addCommands(
            new SwerveDriveAlignedToHub(),
            new FeederForward(),
            new WaitCommand(Settings.Shooter.SHOOT_TIME_AUTO).deadlineFor(new IntakeAgitateOnce().repeatedly()),
            new FeederIdle(),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                .alongWith(new IntakeSetIntake()),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1])
                .alongWith(new IntakeSetIdle()),

            new SwerveDriveAlignedToHub(),
            new FeederForward(),
            new WaitCommand(Settings.Shooter.SHOOT_TIME_AUTO).deadlineFor(new IntakeAgitateOnce().repeatedly()),
            new FeederIdle(),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            new IntakeSetIntake(), 
            new WaitCommand(3), // intake Time
            
            new SwerveDriveAlignedToHub(),
            new FeederForward(),
            new WaitCommand(Settings.Shooter.SHOOT_TIME_AUTO).deadlineFor(new IntakeAgitateOnce().repeatedly()),
            new FeederIdle() // done without the usage of a rotation path?
            // TODO: consider removing usage of rotation paths (paths that only rotate the robot about a point)
        );
    }
}
