package com.stuypulse.robot.commands.auton.shooting;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.feeder.FeederSetForward;
import com.stuypulse.robot.commands.handoff.HandoffSetForward;
import com.stuypulse.robot.commands.intake.IntakeAgitateFastOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.shooter.ShooterFirstShotIncrease;
import com.stuypulse.robot.commands.shooter.ShooterSetShoot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForSpinUp;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RBDumpy extends SequentialCommandGroup {
    public RBDumpy(PathPlannerPath... paths) {
        addCommands( // TODO" check
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
            new IntakeSetIntake(),
            Swerve.getInstance().followPathCommand(paths[0]),
            Swerve.getInstance().followPathCommand(paths[1]),
            Swerve.getInstance().followPathCommand(paths[2]),
            Swerve.getInstance().followPathCommand(paths[3]),
            new SwerveDriveXMode(),
            new ShooterWaitForSpinUp(),
            new ShooterSetShoot(),
            new HandoffSetForward(),
            new ParallelDeadlineGroup(
                new WaitCommand(7),
                new FeederSetForward(), 
                new IntakeAgitateFastOnce().repeatedly(),
                new ShooterFirstShotIncrease()
            )
        );
    }
}