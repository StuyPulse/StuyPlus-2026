package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveLeftCorner extends Command {
    private final CommandSwerveDrivetrain swerve;
    // private final SwerveRequest request;

    public SwerveDriveLeftCorner() {
        this.swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(swerve);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getPose().getRotation().minus(Settings.Swerve.Alignment.Targets.HUB_LEFT_CORNER).getDegrees()) < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE.getDegrees();
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Swerve/Target Angle", -45);
    }

    @Override
    public void execute() {
        SwerveRequest request = new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(Settings.Swerve.Alignment.Targets.HUB_LEFT_CORNER)
            .withVelocityX(0)
            .withVelocityY(0)
            .withHeadingPID(Gains.Swerve.Alignment.akP, Gains.Swerve.Alignment.akI, Gains.Swerve.Alignment.akD);
        swerve.setControl(request);
    }
}