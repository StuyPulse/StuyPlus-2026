package com.stuypulse.robot.commands.auton.depot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.feeder.FeederSetForward;
import com.stuypulse.robot.commands.handoff.HandoffSetForward;
import com.stuypulse.robot.commands.intake.IntakeAgitateFastOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.shooter.ShooterSetShoot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForSpinUp;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignToHub;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CenterDepot extends SequentialCommandGroup {
    public CenterDepot(PathPlannerPath... paths) {
        addCommands(
           new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
           new IntakeSetIntake(),
           CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
           CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
           new SwerveDriveXMode(),
           new ShooterWaitForSpinUp(),
           new ShooterSetShoot(),
           new HandoffSetForward(),
           new ParallelDeadlineGroup(
            new WaitCommand(5),
               new FeederSetForward(),
               new IntakeAgitateFastOnce().repeatedly()
            )
        );
    }
}