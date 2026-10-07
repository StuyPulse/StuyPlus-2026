package com.stuypulse.robot.commands.auton.bline;

import static edu.wpi.first.units.Units.Meters;

import com.stuypulse.robot.commands.feeder.FeederSetForward;
import com.stuypulse.robot.commands.handoff.HandoffSetForward;
import com.stuypulse.robot.commands.intake.IntakeAgitateFastOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.shooter.ShooterFirstShotIncrease;
import com.stuypulse.robot.commands.shooter.ShooterSetShoot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForSpinUp;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RBDumpyBLine extends BLineAuton {

    private final Distance HANDOFF_THRESHOLD_METERS = Meters.of(0.15);

    public RBDumpyBLine(String... pathNames) {
        addCommands(
            new IntakeSetIntake(),

            followUntil(pathNames[0], HANDOFF_THRESHOLD_METERS),
            followUntil(pathNames[1], HANDOFF_THRESHOLD_METERS),
            followUntil(pathNames[2], HANDOFF_THRESHOLD_METERS),
            followUntil(pathNames[3], HANDOFF_THRESHOLD_METERS),

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
