package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

public class HandoffSetForward extends HandoffSetState{
    public HandoffSetForward(){
        super(HandoffState.FORWARD);
    }
}
