package com.stuypulse.robot.commands.Feeder;

import com.stuypulse.robot.subsystems.Feeder.Feeder.FeederState;

public class FeederIdle extends FeederSetState{
    public FeederIdle() {
        super(FeederState.STOP);
    }
}