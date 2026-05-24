/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.DriveInputProcessor;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveRotate extends Command {

    private final CommandSwerveDrivetrain swerve;

    private Rotation2d rotation;

    private CommandXboxController driver;

    private final DriveInputProcessor speed;

    public SwerveDriveRotate(CommandXboxController driver, Rotation2d rotation) {
        this.swerve = CommandSwerveDrivetrain.getInstance();
        this.rotation = rotation;
        this.driver = driver;
        this.speed = new DriveInputProcessor(
                driver, 
                Drive.DEADBAND, 
                Drive.POWER, 
                Swerve.Constraints.MAX_VELOCITY_M_PER_S, 
                Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED, 
                Drive.RC);
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        speed.update();

        SwerveRequest request = new SwerveRequest.FieldCentricFacingAngle()
                .withTargetDirection(rotation)
                .withVelocityX(speed.get().getX())
                .withVelocityY(speed.get().getY())
                .withHeadingPID(
                        Gains.Swerve.Alignment.akP, Gains.Swerve.Alignment.akI, Gains.Swerve.Alignment.akD);
        swerve.setControl(request);
        DogLog.log(
                "Swerve/Angle Minus Target Angle",
                swerve.getPose().getRotation().minus(rotation).getDegrees());
        DogLog.log("Swerve/Facing Angle", swerve.getPose().getRotation().getDegrees());
    }
}
