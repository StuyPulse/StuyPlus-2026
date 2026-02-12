package com.stuypulse.robot.commands.feeder;

import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class FeederIdle extends FeederSetState{
    public FeederIdle() {
        super(FeederState.STOP);
    }
}