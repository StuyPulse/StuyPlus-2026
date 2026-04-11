package com.stuypulse.robot.commands.swerve.driveAligned;


import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveDriveWhileAligned extends Command {
    protected static final CommandSwerveDrivetrain swerve;
    private final CommandXboxController driver;
    private final VStream speed;
    private final Supplier<Pose2d> targetPose;

    public SwerveDriveDriveWhileAligned(CommandXboxController driver, Supplier<Pose2d> targetPose) {
        this.speed = VStream.create(this::getDriverInputAsVelocity)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER),
                x -> x.mul(Swerve.Constraints.MAX_VELOCITY_M_PER_S),
                new VRateLimit(Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED),
                new VLowPassFilter(Drive.RC)
            );
        this.driver = driver;
        this.targetPose = targetPose;
        addRequirements(swerve);
    }

    static {
        swerve = CommandSwerveDrivetrain.getInstance();
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(-driver.getLeftY(), -driver.getLeftX());
    }

    public Rotation2d getTargetAngle() {
        Pose2d currentPose = swerve.getPose();
        double atan = Math.atan2(targetPose.get().getY() - currentPose.getY(), targetPose.get().getX() - currentPose.getX());
        return new Rotation2d(atan);
    };

    @Override
    public void execute() {
        swerve.setControl(new SwerveRequest.FieldCentricFacingAngle()
            .withVelocityX(speed.get().x)
            .withVelocityY(speed.get().y)
            .withTargetDirection(getTargetAngle())
            .withHeadingPID(Alignment.akP, Alignment.akI, Alignment.akD)
        );

        SmartDashboard.putNumber("Swerve/targetAngle", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Target Pose X", targetPose.get().getX());
        SmartDashboard.putNumber("Swerve/Target Pose Y", targetPose.get().getY());
    }

}