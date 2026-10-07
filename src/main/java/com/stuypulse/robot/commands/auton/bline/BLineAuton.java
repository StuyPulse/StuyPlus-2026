package com.stuypulse.robot.commands.auton.bline;

import static edu.wpi.first.units.Units.Meters;

import com.stuypulse.robot.subsystems.swerve.Swerve;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.BLine.FollowPath;

public abstract class BLineAuton extends SequentialCommandGroup {
    protected final Swerve swerve = Swerve.getInstance();

    protected Command followUntil(String pathName, Distance threshold) {
        FollowPath path = swerve.followBlinePath(pathName);

        return path.raceWith(
            new WaitUntilCommand(() -> path.getRemainingPathDistanceMeters() < threshold.in(Meters))
        );
    }
}
