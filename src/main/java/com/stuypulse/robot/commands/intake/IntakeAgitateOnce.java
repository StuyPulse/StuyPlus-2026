package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeAgitateOnce extends SequentialCommandGroup {
    public IntakeAgitateOnce() {
        addCommands(
            new IntakeSetDown(),
            new WaitCommand(0.25),
            new IntakeSetAgitateUp(),
            new WaitCommand(0.25),
            new IntakeSetDown()
        );
    }
}