package com.stuypulse.robot.commands.swerve.driveAligned;

import java.util.Optional;

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

public class SwerveDriveDriveWhileAligned extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final Gamepad driver;
    private final VStream speed;

    public SwerveDriveDriveWhileAligned(Gamepad driver) {
        this.swerve = CommandSwerveDrivetrain.getInstance();

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

        addRequirements(swerve);
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(driver.getLeftStick().y, -driver.getLeftStick().x);
    }

    public Pose2d getTargetPose() {
        return new Pose2d();
    }
    public Rotation2d getTargetAngle() {
        Pose2d currentPose = swerve.getPose();
        double atan = Math.atan2(getTargetPose().getY() - currentPose.getY(), getTargetPose().getX() - currentPose.getX());
        return new Rotation2d((atan));
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
    }
}