package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HandoffSetState extends InstantCommand{
    private Handoff handoff;
    private HandoffState handoffState;

    public HandoffSetState(HandoffState handoffState){
        this.handoff = Handoff.getInstance();
        this.handoffState = handoffState;

        addRequirements(handoff);
    }
    @Override
    public void initialize(){
        handoff.setState(handoffState);
    }
}
