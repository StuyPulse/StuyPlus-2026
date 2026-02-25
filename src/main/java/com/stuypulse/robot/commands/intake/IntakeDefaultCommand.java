package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDefaultCommand extends Command{
    private Intake intake;

    public IntakeDefaultCommand() {
        this.intake = Intake.getInstance();

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setState(IntakeState.INTAKE);
    }
}
