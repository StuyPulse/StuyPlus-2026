package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

public class HandoffSetActive extends HandoffSetState{
    public HandoffSetActive(){
        super(HandoffState.ACTIVE);
    }
}
