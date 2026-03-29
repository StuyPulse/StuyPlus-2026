package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeSetOutpost extends IntakeSetState{
    public IntakeSetOutpost() {
        super(IntakeState.OUTPOST);
    }
}
