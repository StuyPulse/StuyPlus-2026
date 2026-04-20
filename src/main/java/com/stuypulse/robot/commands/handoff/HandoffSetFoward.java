package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff;

public class HandoffSetFoward extends HandoffSetState {
    public HandoffSetFoward(){
        super(Handoff.HandoffState.FORWARD);
    }
}
