package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest; 

public class SwerveDriveSetAlignment extends Command {
    protected static final CommandSwerveDrivetrain instance;
    protected final BStream isAligned;
    private Pose2d pose;
    
    protected SwerveDriveSetAlignment(Pose2d pose) {
        this.isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE));
        this.pose = pose;

        addRequirements(instance);
    }

    static {
        instance = CommandSwerveDrivetrain.getInstance();
    }
    
    public Rotation2d getTargetAngle() {
        Pose2d currentPose = instance.getPose();
        double atan = Math.atan2(pose.getY() - currentPose.getY(), pose.getX() - currentPose.getX());
        return new Rotation2d((atan));
    };

    private boolean isAligned() {
        return Math.abs(instance.getPose().getRotation().minus(getTargetAngle()).getDegrees()) < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE.getDegrees();
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }

    @Override
    public void execute() {
        SwerveRequest request = new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(getTargetAngle())
            .withVelocityX(0)
            .withVelocityY(0)
            .withHeadingPID(Alignment.akP, Alignment.akI, Alignment.akD);
        instance.setControl(request);

        SmartDashboard.putNumber("States/SwerveAlignTargetAngle", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("States/SwerveAlignTargetX", pose.getX());
        SmartDashboard.putNumber("States/SwerveAlignTargetY", pose.getY());
    }
}