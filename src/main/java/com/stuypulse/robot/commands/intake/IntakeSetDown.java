package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeSetDown extends IntakeSetState{
    public IntakeSetDown(){
        super(IntakeState.DOWN);
    }
}
