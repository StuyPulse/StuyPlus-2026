package com.stuypulse.robot.commands.Feeder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.stuypulse.robot.subsystems.Feeder.Feeder;
import com.stuypulse.robot.subsystems.Feeder.Feeder.FeederState;

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