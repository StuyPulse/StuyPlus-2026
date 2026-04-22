package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

public class HandoffSetStop extends HandoffSetState{
    public HandoffSetStop() {
        super(HandoffState.STOP);
    }
}
