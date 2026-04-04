package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveRotate extends Command{
    private final CommandSwerveDrivetrain swerve;
    private Rotation2d rotation;
    private CommandXboxController driver;

    private final VStream speed;

    public SwerveDriveRotate(CommandXboxController driver, Rotation2d rotation) {
        this.swerve = CommandSwerveDrivetrain.getInstance();
        this.rotation = rotation;
        this.driver = driver;

        speed = VStream.create(this::getDriverInputAsVelocity)
        .filtered(
            new VDeadZone(Drive.DEADBAND),
            x -> x.clamp(1),
            x -> x.pow(Drive.POWER),
            x -> x.mul(Swerve.Constraints.MAX_VELOCITY_M_PER_S),
            new VRateLimit(Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED),
            new VLowPassFilter(Drive.RC)
        );

        addRequirements(swerve);
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(-driver.getLeftY(), -driver.getLeftX());
    }

    @Override
    public void execute() {
        SwerveRequest request = new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(rotation)
            .withVelocityX(speed.get().x)
            .withVelocityY(speed.get().y)
            .withHeadingPID(Gains.Swerve.Alignment.akP, Gains.Swerve.Alignment.akI, Gains.Swerve.Alignment.akD);

        swerve.setControl(request);

        SmartDashboard.putNumber("Swerve/Angle Minus Target Angle", swerve.getPose().getRotation().minus(rotation).getDegrees());
        SmartDashboard.putNumber("Swerve/Facing Angle", swerve.getPose().getRotation().getDegrees());
    }
}