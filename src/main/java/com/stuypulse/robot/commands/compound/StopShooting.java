package com.stuypulse.robot.commands.compound;

import com.stuypulse.robot.commands.feeder.FeederSetIdle;
import com.stuypulse.robot.commands.handoff.HandoffSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetHomingDown;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class StopShooting extends ParallelCommandGroup{
    public StopShooting() {
        addCommands(
            new HandoffSetIdle(),
            new FeederSetIdle(),
            new IntakeSetHomingDown()
        );
    }
}
