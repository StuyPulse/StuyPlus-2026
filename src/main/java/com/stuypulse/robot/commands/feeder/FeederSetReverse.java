package com.stuypulse.robot.commands.feeder;

import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class FeederSetReverse extends FeederSetState{
    public FeederSetReverse() {
        super(FeederState.REVERSE);
    }
}
