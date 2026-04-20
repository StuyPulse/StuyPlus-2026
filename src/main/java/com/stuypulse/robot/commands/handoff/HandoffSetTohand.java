package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

public class HandoffSetTohand extends HandoffSetState{
    public HandoffSetTohand() {
        super(HandoffState.TOHAND);
    }
}
