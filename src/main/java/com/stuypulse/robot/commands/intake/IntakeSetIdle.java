package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeSetIdle extends IntakeSetState{
    public IntakeSetIdle() {
        super(IntakeState.IDLE);
    }
}