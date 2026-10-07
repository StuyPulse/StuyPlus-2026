package com.stuypulse.robot.commands.auton.bline;

public class TwoMeterTestBLine extends BLineAuton {
    public TwoMeterTestBLine(String... pathNames) {
        addCommands(
            swerve.followBlinePath(pathNames[0])
        );
    }
}