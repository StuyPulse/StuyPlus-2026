package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeSetHomingUp extends IntakeSetState{
    public IntakeSetHomingUp() {
        super(IntakeState.HOMING_UP);
    }
}
