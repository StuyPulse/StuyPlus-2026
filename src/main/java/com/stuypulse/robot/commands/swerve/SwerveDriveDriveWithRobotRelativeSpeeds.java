/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.Robot;

public class SwerveDriveDriveWithRobotRelativeSpeeds extends Command {

    private final Swerve swerve;

    private double velocityX;

    private double velocityY;

    private double angularVelocity;

    public SwerveDriveDriveWithRobotRelativeSpeeds(
            double velocityX, double velocityY, double angularVelocity) {
        this.swerve = Swerve.getInstance();
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.angularVelocity = angularVelocity;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(velocityX, velocityY, -angularVelocity);
        boolean isFlipped = !Robot.isBlue();
        swerve.runVelocity(
                ChassisSpeeds.fromRobotRelativeSpeeds(speeds,
                        isFlipped
                                ? swerve.getRotation().plus(new Rotation2d(Math.PI))
                                : swerve.getRotation()));
    }
}
