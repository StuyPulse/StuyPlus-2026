package com.stuypulse.robot.commands.auton.bline;

import com.stuypulse.robot.commands.swerve.SwerveResetPose;

import frc.robot.lib.BLine.Path;

public class TwoMeterTestBLine extends BLineAuton {
    public TwoMeterTestBLine(Path... paths) {
        addCommands(
            new SwerveResetPose(paths[0].getStartPose()),
            swerve.followBlinePath(paths[0])
        );
    }
}