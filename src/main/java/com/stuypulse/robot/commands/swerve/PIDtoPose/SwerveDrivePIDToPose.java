/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve.PIDtoPose;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveConstants;
import com.stuypulse.robot.util.TranslationMotionProfile;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePIDToPose extends Command {

        private final Swerve swerve;

    private final HolonomicDriveController controller;

        private final Supplier<Pose2d> targetPose;

    // FILTERS
    private final LinearFilter lowPass;
    private final Debouncer debounceRC;

    private double maxVelocity;

        private double maxAcceleration;

        private boolean isMotionProfiled;

    private final BooleanSupplier isAligned;

        private final FieldObject2d targetPose2d;

        private Number xTolerance;

        private Number yTolerance;

        private Number thetaTolerance;

        private Number maxVelocityWhenAligned;

    private Supplier<Translation2d> translationSetpoint;

        private Supplier<Boolean> canEnd;

    public SwerveDrivePIDToPose(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public SwerveDrivePIDToPose(Supplier<Pose2d> targetPose) {
        swerve = Swerve.getInstance();
        controller = new HolonomicDriveController(
            new PIDController(SwerveConstants.SwerveGains.Alignment.XY.kP, SwerveConstants.SwerveGains.Alignment.XY.kI, SwerveConstants.SwerveGains.Alignment.XY.kD),
            new PIDController(SwerveConstants.SwerveGains.Alignment.XY.kP, SwerveConstants.SwerveGains.Alignment.XY.kI, SwerveConstants.SwerveGains.Alignment.XY.kD),
            new ProfiledPIDController(
                SwerveConstants.SwerveGains.Alignment.THETA.kP,
                SwerveConstants.SwerveGains.Alignment.THETA.kI,
                SwerveConstants.SwerveGains.Alignment.THETA.kD,
                new TrapezoidProfile.Constraints(
                    SwerveConstants.SwerveSettings.Alignment.Constraints.DEFAULT_MAX_ANGULAR_VELOCITY,
                    SwerveConstants.SwerveSettings.Alignment.Constraints.DEFAULT_MAX_ANGULAR_ACCELERATION))
        );
        maxVelocity = SwerveConstants.SwerveSettings.Alignment.Constraints.DEFAULT_MAX_VELOCITY;
        maxAcceleration = SwerveConstants.SwerveSettings.Alignment.Constraints.DEFAULT_MAX_ACCELERATION;
        isMotionProfiled = true;
        translationSetpoint = getNewTranslationSetpointGenerator();
        this.targetPose = targetPose;
        targetPose2d = Field.FIELD2D.getObject("Target Pose");
        lowPass = LinearFilter.singlePoleIIR(0.05, 0.02);
        isAligned = () -> isAlignedX()
                && isAlignedY()
                && isAlignedTheta()
                && getVelocityError() < maxVelocityWhenAligned.doubleValue();
        debounceRC =  new Debouncer(SwerveConstants.SwerveSettings.Alignment.Tolerances.ALIGNMENT_DEBOUNCE.in(Seconds), DebounceType.kRising);
        xTolerance = SwerveConstants.SwerveSettings.Alignment.Tolerances.X_TOLERANCE.in(Meters);
        yTolerance = SwerveConstants.SwerveSettings.Alignment.Tolerances.Y_TOLERANCE.in(Meters);
        thetaTolerance = SwerveConstants.SwerveSettings.Alignment.Tolerances.THETA_TOLERANCE.getRadians();
        maxVelocityWhenAligned = SwerveConstants.SwerveSettings.Alignment.Tolerances.MAX_VELOCITY_WHEN_ALIGNED
                .in(MetersPerSecond);
        canEnd = () -> true;
        addRequirements(swerve);
    }

    public SwerveDrivePIDToPose withTolerance(double x, double y, Rotation2d theta) {
        xTolerance = x;
        yTolerance = y;
        thetaTolerance = theta.getRadians();
        return this;
    }

    public SwerveDrivePIDToPose withTranslationalConstraints(
        double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        return this;
    }

    public SwerveDrivePIDToPose withoutMotionProfile() {
        this.isMotionProfiled = false;
        return this;
    }

    public SwerveDrivePIDToPose withCanEnd(Supplier<Boolean> canEnd) {
        this.canEnd = canEnd;
        return this;
    }

    private Supplier<Translation2d> getNewTranslationSetpointGenerator() {
        if (!isMotionProfiled) {
            return () -> targetPose.get().getTranslation();
        } else {
            TranslationMotionProfile profile = new TranslationMotionProfile(
                this.maxVelocity,
                this.maxAcceleration,
                swerve.getPose().getTranslation(),
                new Translation2d());

            return () -> profile.get(targetPose.get().getTranslation());
        }
    }

    private double getVelocityError() {        
        return Math.abs(lowPass.calculate(
            new Translation2d(
                controller.getXController().getError(),
                controller.getYController().getError())
                .getNorm()));
    }

    @Override
    public void initialize() {
        translationSetpoint = getNewTranslationSetpointGenerator();
    }

    public boolean isAlignedX() {
        return Math.abs(targetPose.get().getX() - swerve.getPose().getX()) < xTolerance.doubleValue();
    }

    public boolean isAlignedY() {
        return Math.abs(targetPose.get().getY() - swerve.getPose().getY()) < yTolerance.doubleValue();
    }

    public boolean isAlignedTheta() {
        return Math.abs(
                targetPose.get().getRotation().minus(swerve.getPose().getRotation())
                        .getRadians()) < thetaTolerance.doubleValue();
    }

    @Override
    public void execute() {
        targetPose2d.setPose(
                Robot.isBlue() ? targetPose.get()
                        : Field.transformToOppositeAlliance(targetPose.get()));
        final ChassisSpeeds output = controller.calculate(
            swerve.getPose(),
            new Pose2d(translationSetpoint.get(), targetPose.get().getRotation()),
            0,
            targetPose.get().getRotation());
        swerve.runVelocity(output);
        Logger.recordOutput("Alignment/Target x", targetPose.get().getX());
        Logger.recordOutput("Alignment/Target y", targetPose.get().getY());
        Logger.recordOutput("Alignment/Target Angle", targetPose.get().getRotation().getDegrees());
        Logger.recordOutput(
                "SwerveConstants.SwerveGains.Alignment/Target Velocity Robot Relative X (m/s)",
                output.vxMetersPerSecond);
        Logger.recordOutput(
                "SwerveConstants.SwerveGains.Alignment/Target Velocity Robot Relative Y (m/s)",
                output.vyMetersPerSecond);
        Logger.recordOutput(
                "SwerveConstants.SwerveGains.Alignment/Target Angular Velocity (rad/s)",
                output.omegaRadiansPerSecond);
        Logger.recordOutput("SwerveConstants.SwerveGains.Alignment/Is Aligned", this.isAligned.getAsBoolean());
        Logger.recordOutput("SwerveConstants.SwerveGains.Alignment/Is Aligned X", isAlignedX());
        Logger.recordOutput("SwerveConstants.SwerveGains.Alignment/Is Aligned Y", isAlignedY());
        Logger.recordOutput("SwerveConstants.SwerveGains.Alignment/Is Aligned Theta", isAlignedTheta());
    }

    @Override
    public boolean isFinished() {
        return debounceRC.calculate(this.isAligned.getAsBoolean()) && canEnd.get();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        Field.clearFieldObject(targetPose2d);
    }
}
