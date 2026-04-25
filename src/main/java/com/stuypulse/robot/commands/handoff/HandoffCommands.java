package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HandoffCommands {
    private static final Handoff handoff;

    static {
        handoff = Handoff.getInstance();
    }

    public static Command setForward() {
        return Commands.runOnce(() -> handoff.setState(HandoffState.FORWARD), handoff);
    }

    public static Command setReverse() {
        return Commands.runOnce(() -> handoff.setState(HandoffState.REVERSE), handoff);
    }

    public static Command setIdle() {
        return Commands.runOnce(() -> handoff.setState(HandoffState.IDLE), handoff);
    }
}
