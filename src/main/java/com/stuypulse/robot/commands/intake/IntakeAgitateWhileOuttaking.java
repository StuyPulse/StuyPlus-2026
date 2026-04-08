package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeAgitateWhileOuttaking extends SequentialCommandGroup {
    public IntakeAgitateWhileOuttaking() {
        addCommands(
            new IntakeSetOuttake(),
            new WaitCommand(1.5),
            new IntakeSetIdle(),
            new WaitCommand(0.15),
            new IntakeSetOuttake()
        );
    }
}