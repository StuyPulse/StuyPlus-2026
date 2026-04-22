package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

public class HandoffSetFoward extends HandoffSetState{
    public HandoffSetFoward() {
        super(HandoffState.FORWARD);
    }
}
