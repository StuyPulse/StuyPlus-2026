package com.stuypulse.robot.commands.auton.bline;

import com.stuypulse.robot.commands.swerve.SwerveResetPose;

import frc.robot.lib.BLine.Path;

public class TwoMeterTestBLine extends BLineAuton {
    public TwoMeterTestBLine(Path... paths) {
        super(paths);

        addCommands(
            resetPoseAtStart(paths[0].getStartPose()),
            swerve.followBlinePath(paths[0])
        );
    }
}