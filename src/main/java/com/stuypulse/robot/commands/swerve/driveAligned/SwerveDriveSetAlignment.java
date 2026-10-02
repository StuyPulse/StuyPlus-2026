/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve.driveAligned;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.util.swerve.AlignmentUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveSetAlignment extends Command {

    protected static final Swerve swerve;

    protected final BooleanSupplier isAligned;
    protected final Debouncer alignmentDebouncer;

    private final PIDController headingController;

    private Supplier<Pose2d> pose;

    protected SwerveDriveSetAlignment(Supplier<Pose2d> pose) {
        this.isAligned = () -> Math.abs(swerve.getPose().getRotation().minus(getTargetAngle())
                .getDegrees()) < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE.getDegrees();
        this.alignmentDebouncer = new Debouncer(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE.in(Seconds), DebounceType.kBoth);
        this.pose = pose;
        this.headingController = new PIDController(Alignment.akP, Alignment.akI, Alignment.akD);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    static {
        swerve = Swerve.getInstance();
    }

    public Rotation2d getTargetAngle() {
        return AlignmentUtil.getTargetAlignmentAngle(swerve.getPose(), pose.get());
    }

    @Override
    public boolean isFinished() {
        return alignmentDebouncer.calculate(isAligned.getAsBoolean());
    }

    @Override
    public void execute() {
        double omega = headingController.calculate(
                swerve.getRotation().getRadians(),
                getTargetAngle().getRadians()); // rotation.get?

        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, omega);
        swerve.runVelocity(speeds);
    }
}
