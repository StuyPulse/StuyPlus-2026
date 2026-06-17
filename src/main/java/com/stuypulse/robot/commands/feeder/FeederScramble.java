package com.stuypulse.robot.commands.feeder;

import com.stuypulse.robot.commands.compound.TunableWaitCommand;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FeederScramble extends SequentialCommandGroup {
    public FeederScramble() {
        addCommands(
            new FeederSetReverse(),
            new TunableWaitCommand(Settings.Feeder.REVERSE_TIME_BEFORE_SHOOT)
        );
    }
}
