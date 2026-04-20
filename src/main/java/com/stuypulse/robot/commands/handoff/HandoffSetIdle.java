package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff;

public class HandoffSetIdle extends HandoffSetState {
    public HandoffSetIdle(){
        super(Handoff.HandoffState.IDLE);
    }
}
