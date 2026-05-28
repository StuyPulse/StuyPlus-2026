package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeSetAgitateDown extends IntakeSetState{

    public IntakeSetAgitateDown() {
        super(IntakeState.AGITATE_DOWN);
    }
}
