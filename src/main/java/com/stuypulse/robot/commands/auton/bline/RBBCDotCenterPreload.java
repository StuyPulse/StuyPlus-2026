package com.stuypulse.robot.commands.auton.bline;

import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.Path;

public class RBBCDotCenterPreload extends SequentialCommandGroup{
    
    public RBBCDotCenterPreload(String... pathNames) {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(
            swerve.getPathBuilder()
                .withPoseReset(swerve::resetPose)
                .build(new Path(pathNames[0])),
            new IntakeSetIntake()
        );
    }
}
