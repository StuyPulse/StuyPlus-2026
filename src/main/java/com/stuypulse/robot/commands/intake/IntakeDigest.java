package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeDigest extends IntakeSetState{
    public IntakeDigest() {
        super(IntakeState.DIGEST);
    }
}
