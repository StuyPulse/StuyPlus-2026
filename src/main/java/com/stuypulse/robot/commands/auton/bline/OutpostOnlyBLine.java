package com.stuypulse.robot.commands.auton.bline;

import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.util.BlineUtil;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.Path;

public class OutpostOnlyBLine extends SequentialCommandGroup {
    public OutpostOnlyBLine(String... pathNames) {
        
        Swerve swerve = Swerve.getInstance();

        addCommands(
            swerve.getPathBuilder()
                .withPoseReset(swerve::setPose)
                .build(new Path(BlineUtil.PATHS_DIR, pathNames[0]))
        );
    }
}
