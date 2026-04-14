package com.stuypulse.robot.commands.feeder;

import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class FeederSetForward extends FeederSetState{
    public FeederSetForward() {
        super(FeederState.FORWARD);
    }
}
