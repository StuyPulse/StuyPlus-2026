package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeSetNinety extends InstantCommand{
    private final Intake intake;

    public IntakeSetNinety() {
        intake = Intake.getInstance();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        intake.setPivotNinety();
    }
}
