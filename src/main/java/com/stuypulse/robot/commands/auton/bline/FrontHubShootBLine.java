package com.stuypulse.robot.commands.auton.bline;

import com.stuypulse.robot.commands.feeder.FeederSetForward;
import com.stuypulse.robot.commands.handoff.HandoffSetForward;
import com.stuypulse.robot.commands.intake.IntakeAgitateFastOnce;
import com.stuypulse.robot.commands.shooter.ShooterFirstShotIncrease;
import com.stuypulse.robot.commands.shooter.ShooterSetShoot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForSpinUp;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignToHub;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.BLine.Path;

public class FrontHubShootBLine extends BLineAuton {
    public FrontHubShootBLine(Path... paths) {
        super(paths);

        addCommands(
            resetPoseAtStart(paths[0].getStartPose()),

            swerve.followBlinePath(paths[0]),

            new SwerveDriveAlignToHub(),
            new SwerveDriveXMode(),
            new ShooterWaitForSpinUp(),
            new ShooterSetShoot(),
            new HandoffSetForward(),
            new ParallelDeadlineGroup(
                new WaitCommand(6.5),
                new FeederSetForward(), 
                new IntakeAgitateFastOnce().repeatedly(),
                new ShooterFirstShotIncrease()
            )
        );
    }
}
