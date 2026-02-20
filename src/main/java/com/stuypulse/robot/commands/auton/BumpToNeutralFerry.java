package com.stuypulse.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.feeder.FeederForward;
import com.stuypulse.robot.commands.feeder.FeederIdle;
import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.shooter.ShooterFerry;
import com.stuypulse.robot.commands.shooter.ShooterShoot;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToAllianceZone;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToHub;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class BumpToNeutralFerry extends SequentialCommandGroup {

    public BumpToNeutralFerry(PathPlannerPath... paths) {
        addCommands(
            new SwerveDriveAlignedToHub(),
            new FeederForward(),
            new WaitCommand(Settings.Shooter.SHOOT_TIME_AUTO).deadlineFor(new IntakeAgitateOnce().repeatedly()),
            new FeederIdle(),
            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]) 
                .alongWith(new IntakeSetIntake()),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1])
                .alongWith(new IntakeSetIntake()),
            new SwerveDriveAlignedToAllianceZone(),
            new ShooterFerry(),
            new FeederForward(),
            new WaitCommand(Settings.Shooter.SHOOT_TIME_AUTO).deadlineFor(new IntakeAgitateOnce().repeatedly()),
            new FeederIdle(),
            new ShooterShoot()
        );
    }
}