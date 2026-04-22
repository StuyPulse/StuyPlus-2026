package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

public class HandoffSetReverse extends HandoffSetState{
    public HandoffSetReverse() {
        super(HandoffState.REVERSE);
    }
}
