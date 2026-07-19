package com.stuypulse.robot.commands.auton.bline;

import com.stuypulse.robot.commands.feeder.FeederSetForward;
import com.stuypulse.robot.commands.handoff.HandoffSetForward;
import com.stuypulse.robot.commands.intake.IntakeAgitateFastOnce;
import com.stuypulse.robot.commands.shooter.ShooterFirstShotIncrease;
import com.stuypulse.robot.commands.shooter.ShooterSetShoot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForSpinUp;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignToHub;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.lib.BLine.Path;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FrontHubShootPreloadsBLine extends SequentialCommandGroup {
    public FrontHubShootPreloadsBLine(String... pathNames) {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(
            swerve.getPathBuilder()
                .withPoseReset(swerve::resetPose)
                .build(new Path(pathNames[0])),
            new SwerveDriveAlignToHub(), // worth noting that alignment commands don't work so this may not aim correctly
            new SwerveDriveXMode(),
            new ShooterWaitForSpinUp(),
            new ShooterSetShoot(),
            new HandoffSetForward(),
            new ParallelDeadlineGroup(
                new WaitCommand(6.5), // shoot for 6-7 seconds
                new FeederSetForward(),
                new IntakeAgitateFastOnce().repeatedly(),
                new ShooterFirstShotIncrease()
            )
        );
    }
}