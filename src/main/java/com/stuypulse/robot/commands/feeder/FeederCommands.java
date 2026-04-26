package com.stuypulse.robot.commands.feeder;

import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class FeederCommands {
    private static final Feeder feeder;

    static {
        feeder = Feeder.getInstance();
    }

    public static Command setForward() {
        return Commands.runOnce(() -> feeder.setState(FeederState.FORWARD), feeder).withName("FeederSetForward");
    }

    public static Command setReverse() {
        return Commands.runOnce(() -> feeder.setState(FeederState.REVERSE), feeder).withName("FeederSetReverse");
    }

    public static Command setIdle() {
        return Commands.runOnce(() -> feeder.setState(FeederState.IDLE), feeder).withName("FeederSetIdle");
    }
}