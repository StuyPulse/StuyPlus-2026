package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeSetHomingDown extends IntakeSetState{
    public IntakeSetHomingDown() {
        super(IntakeState.HOMING_DOWN);
    }
}
