package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeSetZero extends InstantCommand {
    private final Intake intake;

    public IntakeSetZero() {
        this.intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        intake.setPivotZero();
    }
}
