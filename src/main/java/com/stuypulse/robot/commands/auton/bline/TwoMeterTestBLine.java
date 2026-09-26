package com.stuypulse.robot.commands.auton.bline;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.BlineUtil;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.lib.BLine.Path;

public class TwoMeterTestBLine extends SequentialCommandGroup {
    
    public TwoMeterTestBLine(String... pathNames) {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(
            
            swerve.getPathBuilder()
                .withPoseReset(swerve::resetPose)
                .build(new Path(BlineUtil.PATHS_DIR, pathNames[0]))
        );
    }
}
