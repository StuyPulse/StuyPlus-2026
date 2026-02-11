package com.stuypulse.robot.commands.Feeder;

import com.stuypulse.robot.subsystems.Feeder.Feeder.FeederState;

public class FeederForward extends FeederSetState{
    public FeederForward() {
        super(FeederState.FORWARD);
    }
}