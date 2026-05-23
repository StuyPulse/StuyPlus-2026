/************************* PROJECT RON *************************/
/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
/***************************************************************/
package com.stuypulse.robot.commands.swerve.driveAligned;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.Supplier;

// WARNING: I don't know if this refactoring actually works.
public class SwerveDriveDriveWhileAligned extends Command {

    protected static final CommandSwerveDrivetrain swerve;

    private final CommandXboxController driver;

    private final Supplier<Pose2d> targetPose;

    private final SlewRateLimiter slewRateLimiter;

    public SwerveDriveDriveWhileAligned(CommandXboxController driver, Supplier<Pose2d> targetPose) {
        this.driver = driver;
        this.targetPose = targetPose;
        this.slewRateLimiter = new SlewRateLimiter(Settings.Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED);
        addRequirements(swerve);
    }

    static {
        swerve = CommandSwerveDrivetrain.getInstance();
    }

    private Translation2d applyVectorMagnitudeReduction(Translation2d inputVector, double reductionFactor) {
        double magnitude = inputVector.getNorm();
        double reducedMagnitude = magnitude * reductionFactor;

        if (magnitude < 1e-6) {
            return new Translation2d(0, 0);
        }

        double scale = reducedMagnitude / magnitude;
        return inputVector.times(scale);
    }

    private Translation2d getDriverInputAsVelocity() {
        Translation2d inputVector = new Translation2d(-driver.getLeftY(), -driver.getLeftX());
        Translation2d deadzonedVector = applyVectorMagnitudeReduction(inputVector, MathUtil.applyDeadband(inputVector.getNorm(), Settings.Driver.Drive.DEADBAND, 1.0));
        
        double poweredMagnitude = MathUtil.copyDirectionPow(deadzonedVector.getNorm(), Settings.Driver.Drive.POWER);
        Translation2d poweredVector = deadzonedVector.times(poweredMagnitude / (deadzonedVector.getNorm() + 1e-6));
        return poweredVector;
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
        Translation2d driverInput = getDriverInputAsVelocity();
        double limitedDriveMagnitude = slewRateLimiter.calculate(driverInput.getNorm());
        Translation2d limitedDrive = applyVectorMagnitudeReduction(driverInput, limitedDriveMagnitude / (driverInput.getNorm() + 1e-6));

        swerve.setControl(
                new SwerveRequest.FieldCentricFacingAngle()
                        .withVelocityX(limitedDrive.getX())
                        .withVelocityY(limitedDrive.getY())
                        .withTargetDirection(getTargetAngle())
                        .withHeadingPID(Alignment.akP, Alignment.akI, Alignment.akD));
        DogLog.log("Swerve/targetAngle", getTargetAngle().getDegrees());
        DogLog.log("Swerve/Target Pose X", targetPose.get().getX());
        DogLog.log("Swerve/Target Pose Y", targetPose.get().getY());
    }
}
