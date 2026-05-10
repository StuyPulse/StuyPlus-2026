package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeSetAgitateSlowUp extends IntakeSetState{
    public IntakeSetAgitateSlowUp() {
        super(IntakeState.DIGEST);
    }
}
