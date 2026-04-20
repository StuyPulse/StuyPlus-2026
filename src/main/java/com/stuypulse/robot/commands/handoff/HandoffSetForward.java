package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff;

public class HandoffSetForward extends HandoffSetState {
    public HandoffSetForward(){
        super(Handoff.HandoffState.FORWARD);
    }
}
