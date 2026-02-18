package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class SwerveDriveGetAlignment extends Command {
    protected static final CommandSwerveDrivetrain instance;
    protected final BStream isAligned;
    private Pose2d pose;
    
    protected SwerveDriveGetAlignment(Pose2d pose) {
        this.isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE));
        this.pose = pose;
        addRequirements(instance);
    }

    static {
        instance = CommandSwerveDrivetrain.getInstance();
    }
    
    public Rotation2d getTargetAngle(Pose2d pose) {
        Transform2d diff = pose.minus(instance.getPose());
        return new Rotation2d(Math.atan(diff.getY() / diff.getX()));
    };

    private boolean isAligned() {
        Pose2d pose = instance.getPose();
        return Math.abs(pose.getRotation().minus(getTargetAngle(pose)).getDegrees()) < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE.getDegrees();
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }

    @Override
    public void execute() {
        instance.setControl(new SwerveRequest.FieldCentricFacingAngle().withTargetDirection(getTargetAngle(pose)));
    }
}