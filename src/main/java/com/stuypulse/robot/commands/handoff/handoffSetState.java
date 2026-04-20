package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class handoffSetState extends InstantCommand{
    private final Handoff handoff;
    private final HandoffState handoffState;

    public handoffSetState(HandoffState handoffState) {
        this.handoff = Handoff.getInstance();
        this.handoffState = handoffState;
        
        addRequirements(handoff);
    }
    
    @Override

    public void initialize(){
        handoff.setState(handoffState);
    }
}
