package com.stuypulse.robot.commands;

import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.intake.IntakeSetOuttake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PreMatch extends SequentialCommandGroup {
    public PreMatch() {
        addCommands(
            new IntakeAgitateOnce(),
            new IntakeSetIntake(),
            new WaitCommand(10),
            new IntakeSetIdle(),
            new WaitCommand(1),
            new IntakeSetOuttake(),
            new WaitCommand(5),
            new IntakeSetIdle()
        );
    }
}