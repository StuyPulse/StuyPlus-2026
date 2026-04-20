package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HandoffSetState extends InstantCommand{
    private final Handoff handoff;
    private final HandoffState handoffState;

    public HandoffSetState(HandoffState state) {
        this.handoff = Handoff.getInstance();
        this.handoffState = state;

        addRequirements(handoff);
    }

    @Override
    public void initialize() {
        handoff.setState(handoffState);
    }
}
