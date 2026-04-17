package com.stuypulse.robot.commands.feeder;

import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class FeederSetStop extends FeederSetState{
    public FeederSetStop() {
        super(FeederState.STOP);
    }
}
