package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

public class HandoffSetIdle extends HandoffSetState {
    public HandoffSetIdle(){
        super(HandoffState.IDLE);
    }
}
