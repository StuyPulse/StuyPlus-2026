/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.swerve.SwerveConstants.SwerveSettings;
import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.util.swerve.swerveinput.DriveInputProcessor;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveRotate extends Command {

    private final Swerve swerve;

    private Rotation2d rotation;

    private CommandXboxController driver;

    private final DriveInputProcessor speed;

    private final PIDController headingController;

    public SwerveDriveRotate(CommandXboxController driver, Rotation2d rotation) {
        this.swerve = Swerve.getInstance();
        this.rotation = rotation;
        this.driver = driver;
        this.speed = new DriveInputProcessor(
                driver,
                Drive.DEADBAND,
                Drive.POWER,
                SwerveSettings.Constraints.MAX_VELOCITY_M_PER_S,
                SwerveSettings.Constraints.MAX_ACCEL_M_PER_S_SQUARED,
                Drive.RC);
        addRequirements(swerve);

        headingController = new PIDController(
                Gains.Swerve.Alignment.akP,
                Gains.Swerve.Alignment.akI,
                Gains.Swerve.Alignment.akD);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        headingController.reset();
    }

    @Override
    public void execute() {
        speed.update();

        Translation2d velocity = speed.get();

        double omega = headingController.calculate(
                swerve.getRotation().getRadians(),
                rotation.getRadians()); // rotation.get?

        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);

        swerve.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldSpeeds, swerve.getRotation()));

        DogLog.log(
                "Swerve/Angle Minus Target Angle",
                swerve.getPose().getRotation().minus(rotation).getDegrees());
        DogLog.log("Swerve/Facing Angle", swerve.getPose().getRotation().getDegrees());
    }
}
