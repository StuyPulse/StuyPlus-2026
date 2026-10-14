/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve.driveAligned;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveConstants;
import com.stuypulse.robot.subsystems.swerve.SwerveConstants.SwerveGains;
import com.stuypulse.robot.util.swerve.swerveinput.DriveInputProcessor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveDriveWhileAligned extends Command {

    protected static final Swerve swerve;

    private final CommandXboxController driver;

    private final DriveInputProcessor speed;

    private final Supplier<Pose2d> targetPose;

    private final PIDController headingController;

    public SwerveDriveDriveWhileAligned(CommandXboxController driver, Supplier<Pose2d> targetPose) {
        this.speed = new DriveInputProcessor(
                driver, 
                Drive.DEADBAND, 
                Drive.POWER, 
                SwerveConstants.SwerveSettings.Constraints.MAX_VELOCITY_M_PER_S, 
                SwerveConstants.SwerveSettings.Constraints.MAX_ACCEL_M_PER_S_SQUARED, 
                Drive.RC);
        this.driver = driver;
        this.targetPose = targetPose;
        this.headingController = new PIDController(SwerveGains.Alignment.akP, SwerveGains.Alignment.akI, SwerveGains.Alignment.akD);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    static {
        swerve = Swerve.getInstance();
    }

    public Rotation2d getTargetAngle() {
        Pose2d currentPose = swerve.getPose();
        double atan = Math.atan2(
                targetPose.get().getY() - currentPose.getY(),
                targetPose.get().getX() - currentPose.getX());
        return new Rotation2d(atan);
    }

    @Override
    public void execute() {
        speed.update();

        double omega = headingController.calculate(
                swerve.getRotation().getRadians(),
                getTargetAngle().getRadians()); // rotation.get?

        ChassisSpeeds speeds = new ChassisSpeeds(speed.get().getX(), speed.get().getY(), omega);
        swerve.runVelocity(speeds);
        Logger.recordOutput("Swerve/targetAngle", getTargetAngle().getDegrees());
        Logger.recordOutput("Swerve/Target Pose X", targetPose.get().getX());
        Logger.recordOutput("Swerve/Target Pose Y", targetPose.get().getY());
    }
}
