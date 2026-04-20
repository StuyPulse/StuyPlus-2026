package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

public class handoffSetFoward extends handoffSetState{
    public handoffSetFoward() {
        super(HandoffState.FORWARD);
    }
}
