package com.stuypulse.robot.commands.auton.bline;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoMeterTestBLine extends SequentialCommandGroup {
    
    public TwoMeterTestBLine(String... pathNames) {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(
            swerve.followPath(pathNames[0])
        );
    }
}