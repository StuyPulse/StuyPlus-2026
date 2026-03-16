package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeToggleSwitch extends InstantCommand{
    private Intake intake;
    
    public IntakeToggleSwitch() {
        intake = Intake.getInstance();
    }

    @Override
    public void initialize() {
        switch(intake.getState()) {
            case INTAKE -> intake.setState(IntakeState.IDLE);
            case IDLE -> intake.setState(IntakeState.INTAKE);
            default -> intake.setState(IntakeState.IDLE);
        }
    }
}
