package com.stuypulse.robot.commands.feeder;

import com.stuypulse.robot.commands.compound.TunableWaitCommand;
import com.stuypulse.robot.subsystems.feeder.FeederConstants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FeederScramble extends SequentialCommandGroup {
    public FeederScramble() {
        addCommands(
            new FeederSetReverse(),
            new TunableWaitCommand(FeederConstants.FeederSettings.REVERSE_TIME_BEFORE_SHOOT)
        );
    }
}
