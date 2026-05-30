package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BCDot extends SequentialCommandGroup {

    public BCDot(PathPlannerPath... paths) {
        addCommands(
            new WaitCommand(Settings.BATTLECRY_DOT_DELAY.get()),
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            new SwerveDriveXMode());
    }
}
// usable for all 4 bc dot autos because they are very