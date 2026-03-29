package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveRotate extends InstantCommand{
    private final CommandSwerveDrivetrain swerve;
    private Rotation2d rotation;
    public SwerveDriveRotate(Rotation2d rotation) {
        this.swerve = CommandSwerveDrivetrain.getInstance();
        this.rotation = rotation;

        addRequirements(swerve);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getPose().getRotation().minus(rotation).getDegrees()) < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE.getDegrees();
    }

    @Override
    public void execute() {
        SwerveRequest request = new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(rotation)
            .withVelocityX(0)
            .withVelocityY(0)
            .withHeadingPID(Gains.Swerve.Turn.kP, Gains.Swerve.Turn.kI, Gains.Swerve.Turn.kD);

        swerve.setControl(request);

        SmartDashboard.putNumber("Swerve/Angle Minus Target Angle", swerve.getPose().getRotation().minus(rotation).getDegrees());
        SmartDashboard.putNumber("Swerve/Facing Angle", swerve.getPose().getRotation().getDegrees());
    }
}