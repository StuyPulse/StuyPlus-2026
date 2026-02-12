package com.stuypulse.robot.commands.feeder;

import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FeederSetState extends InstantCommand{
    private final Feeder feeder;
    private final FeederState state;

    public FeederSetState(FeederState state) {
        this.feeder = Feeder.getInstance();
        this.state = state;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.setState(state);
    }
}