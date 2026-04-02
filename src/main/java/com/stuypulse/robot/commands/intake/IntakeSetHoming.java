package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeSetHoming extends IntakeSetState{
    public IntakeSetHoming() {
        super(IntakeState.HOMING);
    }
}
