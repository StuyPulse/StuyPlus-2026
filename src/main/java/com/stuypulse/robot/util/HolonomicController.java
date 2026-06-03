/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class HolonomicController {

    private PIDController xController;

    private PIDController yController;

    private PIDController angleController;

    public HolonomicController(
            PIDController xController, PIDController yController, PIDController angleController) {
        this.xController = xController;
        this.yController = yController;
        this.angleController = angleController;
    }

    public Pose2d getOutput(Pose2d setpoint, Pose2d measurement) {
        return new Pose2d(
            xController.calculate(setpoint.getX(), measurement.getX()),
            yController.calculate(setpoint.getX(), measurement.getX()),
            Rotation2d.fromDegrees(angleController.calculate(setpoint.getRotation().getDegrees(), measurement.getRotation().getDegrees()))
        );
    }

    public Pose2d getError() {
        return new Pose2d(
                xController.getError(),
                yController.getError(),
                Rotation2d.fromDegrees(angleController.getError()));
    }

    public boolean isDone(
            double xToleranceMeters, double yToleranceMeters, double angleToleranceDegrees) {
        return Math.abs(xController.getError()) < xToleranceMeters
                && Math.abs(yController.getError()) < yToleranceMeters
                && Math.abs(angleController.getError()) < Math.toRadians(angleToleranceDegrees);
    }
}
