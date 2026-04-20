package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff;

public class HandoffSetReverse extends HandoffSetState {
    public HandoffSetReverse(){
        super(Handoff.HandoffState.REVERSE);
    }
}
