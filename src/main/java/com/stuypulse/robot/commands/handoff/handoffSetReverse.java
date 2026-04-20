package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

public class handoffSetReverse extends handoffSetState {
    public handoffSetReverse() {
        super(HandoffState.REVERSE);
    }
}
