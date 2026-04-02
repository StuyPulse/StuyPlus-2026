package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSetZeroAtBottom extends Command {
    public IntakeSetZeroAtBottom() {
        Intake.getInstance().setPivotZeroAtBottom();
        addRequirements(Intake.getInstance());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
