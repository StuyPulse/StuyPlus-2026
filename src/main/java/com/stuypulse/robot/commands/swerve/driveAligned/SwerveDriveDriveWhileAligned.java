/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve.driveAligned;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.swerve.swerveinput.DriveInputProcessor;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.Supplier;

public class SwerveDriveDriveWhileAligned extends Command {

    protected static final CommandSwerveDrivetrain swerve;

    private final CommandXboxController driver;

    private final DriveInputProcessor speed;

    private final Supplier<Pose2d> targetPose;

    public SwerveDriveDriveWhileAligned(CommandXboxController driver, Supplier<Pose2d> targetPose) {
        this.speed = new DriveInputProcessor(
                driver, 
                Drive.DEADBAND, 
                Drive.POWER, 
                Swerve.Constraints.MAX_VELOCITY_M_PER_S, 
                Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED, 
                Drive.RC);
        this.driver = driver;
        this.targetPose = targetPose;
        addRequirements(swerve);
    }

    static {
        swerve = CommandSwerveDrivetrain.getInstance();
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

        swerve.setControl(
                new SwerveRequest.FieldCentricFacingAngle()
                        .withVelocityX(speed.get().getX())
                        .withVelocityY(speed.get().getY())
                        .withTargetDirection(getTargetAngle())
                        .withHeadingPID(Alignment.akP, Alignment.akI, Alignment.akD));
        DogLog.log("Swerve/targetAngle", getTargetAngle().getDegrees());
        DogLog.log("Swerve/Target Pose X", targetPose.get().getX());
        DogLog.log("Swerve/Target Pose Y", targetPose.get().getY());
    }
}
