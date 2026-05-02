package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeSetZeroAtBottom extends InstantCommand {
    private Intake intake;

    public IntakeSetZeroAtBottom() {
        this.intake = Intake.getInstance();

        addRequirements(intake);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        intake.setPivotZeroAtBottom();
        intake.setState(IntakeState.DOWN);
    }
}
