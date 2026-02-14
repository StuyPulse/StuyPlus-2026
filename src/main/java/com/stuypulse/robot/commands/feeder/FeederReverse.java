package com.stuypulse.robot.commands.feeder;

import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class FeederReverse extends FeederSetState{
    public FeederReverse() {
        super(FeederState.REVERSE);
    }
}