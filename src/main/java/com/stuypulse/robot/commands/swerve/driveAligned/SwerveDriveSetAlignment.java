/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.commands.swerve.driveAligned;

import static edu.wpi.first.units.Units.*;
import java.util.function.Supplier;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

public class SwerveDriveSetAlignment extends Command {

    protected static final CommandSwerveDrivetrain swerve;

    protected final BStream isAligned;

    private Supplier<Pose2d> pose;

    protected SwerveDriveSetAlignment(Supplier<Pose2d> pose) {
        this.isAligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Both(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE.in(Seconds)));
        this.pose = pose;
        addRequirements(swerve);
    }

    static {
        swerve = CommandSwerveDrivetrain.getInstance();
    }

    public Rotation2d getTargetAngle() {
        Pose2d currentPose = swerve.getPose();
        double atan = Math.atan2(pose.get().getY() - currentPose.getY(), pose.get().getX() - currentPose.getX());
        return new Rotation2d((atan));
    }

    private boolean isAligned() {
        return Math.abs(swerve.getPose().getRotation().minus(getTargetAngle()).getDegrees()) < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE.getDegrees();
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }

    @Override
    public void execute() {
        SwerveRequest request = new SwerveRequest.FieldCentricFacingAngle().withTargetDirection(getTargetAngle()).withVelocityX(0).withVelocityY(0).withHeadingPID(Alignment.akP, Alignment.akI, Alignment.akD);
        swerve.setControl(request);
    }
}
