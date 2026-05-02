package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeSetAgitateUp extends IntakeSetState{
    public IntakeSetAgitateUp() {
        super(IntakeState.AGITATE);
    }
}
