package com.stuypulse.robot.commands.feeder;

import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class FeederForward extends FeederSetState{
    public FeederForward() {
        super(FeederState.FORWARD);
    }
}