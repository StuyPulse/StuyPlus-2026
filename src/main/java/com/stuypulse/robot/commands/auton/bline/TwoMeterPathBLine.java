package com.stuypulse.robot.commands.auton.bline;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.Path;

public class TwoMeterPathBLine extends SequentialCommandGroup {

    public TwoMeterPathBLine(String... paths) {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(
            swerve.getPathBuilder()
                .withPoseReset(swerve::resetPose)
                .build(new Path(paths[0]))
        );
    }
}