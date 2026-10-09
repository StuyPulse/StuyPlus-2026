package com.stuypulse.robot.commands.auton.bline;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.Swerve;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.BLine.BLineField;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public abstract class BLineAuton extends SequentialCommandGroup {
    private final List<Path> displayedPaths = new ArrayList<>();

    protected final Swerve swerve = Swerve.getInstance();

    public BLineAuton(Path... paths) {
        for (Path path : paths) {
            displayedPaths.add(path.copy());
        }
    }

    public void displayPaths() {
        for (Path path : displayedPaths) {
            if (Robot.isBlue()) {
                path.undoFlip();
                BLineField.drawPath(Field.FIELD2D, path);
            } else {
                path.flip();
                BLineField.drawPath(Field.FIELD2D, path);
            }
        }
    }

    public void clearPaths() {
        for (Path path : displayedPaths) {
            Field.FIELD2D.getObject(BLineField.drawPath(Field.FIELD2D, path)).setPoses();
        }
    }

    protected Command followUntil(Path path, Distance threshold) {
        FollowPath pathCommand = swerve.followBlinePath(path);

        return pathCommand.raceWith(
            new WaitUntilCommand(() -> pathCommand.getRemainingPathDistanceMeters() < threshold.in(Meters))
        );
    }
}
