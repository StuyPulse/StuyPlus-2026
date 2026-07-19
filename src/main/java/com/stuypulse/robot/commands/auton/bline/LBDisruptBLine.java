package com.stuypulse.robot.commands.auton.bline;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.Path;

public class LBDisruptBLine extends SequentialCommandGroup {

    public LBDisruptBLine(String... pathNames) {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(
            // First path — pose reset baked into build()
            swerve.getPathBuilder()
                .withPoseReset(swerve::resetPose)
                .build(new Path(pathNames[0])),

            // Remaining paths — no reset, cleared explicitly
            swerve.getPathBuilder()
                .withPoseReset(pose -> {})
                .build(new Path(pathNames[1])),

            swerve.getPathBuilder()
                .withPoseReset(pose -> {})
                .build(new Path(pathNames[2])),

            swerve.getPathBuilder()
                .withPoseReset(pose -> {})
                .build(new Path(pathNames[3]))
        );
    }
}