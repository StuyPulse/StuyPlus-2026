package com.stuypulse.robot.commands.auton.bline;

import static edu.wpi.first.units.Units.Meters;

import com.stuypulse.robot.subsystems.swerve.Swerve;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public abstract class BLineAuton extends SequentialCommandGroup {
    protected final Swerve swerve = Swerve.getInstance();

    protected Command followUntil(Path path, Distance threshold) {
        FollowPath pathCommand = swerve.followBlinePath(path);

        return pathCommand.raceWith(
            new WaitUntilCommand(() -> pathCommand.getRemainingPathDistanceMeters() < threshold.in(Meters))
        );
    }
}
