package com.stuypulse.robot.commands.Feeder;

import com.stuypulse.robot.subsystems.Feeder.Feeder.FeederState;

public class FeederReverse extends FeederSetState{
    public FeederReverse() {
        super(FeederState.REVERSE);
    }
}