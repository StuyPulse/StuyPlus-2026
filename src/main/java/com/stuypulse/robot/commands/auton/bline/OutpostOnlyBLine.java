package com.stuypulse.robot.commands.auton.bline;

public class OutpostOnlyBLine extends BLineAuton {
    public OutpostOnlyBLine(String... pathNames) {
        addCommands(
            swerve.followBlinePath(pathNames[0])
        );
    }
}
