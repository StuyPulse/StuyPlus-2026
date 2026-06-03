/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve.driveAligned;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.swerve.AlignmentUtil;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveSetAlignment extends Command {

    protected static final CommandSwerveDrivetrain swerve;

    protected final Debouncer alignmentDebouncer;

    private Supplier<Pose2d> pose;

    protected SwerveDriveSetAlignment(Supplier<Pose2d> pose) {
        this.alignmentDebouncer = new Debouncer(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE.in(Seconds), Debouncer.DebounceType.kRising);
        this.pose = pose;
        addRequirements(swerve);
    }

    static {
        swerve = CommandSwerveDrivetrain.getInstance();
    }

    public Rotation2d getTargetAngle() {
        return AlignmentUtil.getTargetAlignmentAngle(swerve.getPose(), pose.get());
    }

    private boolean isAligned() {
        return Math.abs(swerve.getPose().getRotation().minus(getTargetAngle())
                .getDegrees()) < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE.getDegrees();
    }

    @Override
    public boolean isFinished() {
        return alignmentDebouncer.calculate(isAligned());
    }

    @Override
    public void execute() {
        SwerveRequest request = new SwerveRequest.FieldCentricFacingAngle()
                .withTargetDirection(getTargetAngle())
                .withVelocityX(0)
                .withVelocityY(0)
                .withHeadingPID(Alignment.akP, Alignment.akI, Alignment.akD);
        swerve.setControl(request);
    }
}
