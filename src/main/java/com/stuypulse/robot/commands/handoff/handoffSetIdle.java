package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

public class handoffSetIdle extends handoffSetState {
    public handoffSetIdle(){
        super(HandoffState.IDLE);
    }
}
