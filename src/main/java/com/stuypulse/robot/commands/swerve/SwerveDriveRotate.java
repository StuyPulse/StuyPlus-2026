package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveRotate extends InstantCommand{
    private final CommandSwerveDrivetrain swerve;
    private Rotation2d rotation;
    public SwerveDriveRotate(Rotation2d rotation) {
        this.swerve = CommandSwerveDrivetrain.getInstance();
        this.rotation = rotation;
    }

    @Override
    public boolean isFinished() {
        return swerve.getPose().getRotation().minus(rotation).getDegrees() < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE.getDegrees();
    }

    @Override
    public void execute() {
        SwerveRequest request = new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(rotation)
            .withHeadingPID(Gains.Swerve.Turn.kP, Gains.Swerve.Turn.kI, Gains.Swerve.Turn.kD);

        swerve.setControl(request);
    }
}