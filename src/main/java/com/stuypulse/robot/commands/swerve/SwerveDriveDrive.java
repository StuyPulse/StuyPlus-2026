/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.subsystems.swerve.SwerveConstants.SwerveSettings;
import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.util.swerve.swerveinput.DriveInputProcessor;
import com.stuypulse.robot.util.swerve.swerveinput.DriveTurnInputProcessor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveDrive extends Command {

    private final Swerve swerve;

    private final CommandXboxController driver;

    private final DriveInputProcessor speed;

    private final DriveTurnInputProcessor turn;

    public SwerveDriveDrive(CommandXboxController driver) {
        swerve = Swerve.getInstance();
        this.speed = new DriveInputProcessor(
                driver,
                Drive.DEADBAND,
                Drive.POWER,
                SwerveSettings.Constraints.MAX_VELOCITY_M_PER_S,
                SwerveSettings.Constraints.MAX_ACCEL_M_PER_S_SQUARED,
                Drive.RC);
        turn = new DriveTurnInputProcessor(
                driver,
                Turn.DEADBAND,
                Turn.POWER,
                SwerveSettings.Constraints.MAX_ANGULAR_VEL_RAD_PER_S, Turn.RC);
        this.driver = driver;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        speed.update();
        turn.update();

        ChassisSpeeds speeds = new ChassisSpeeds(speed.get().getX(), speed.get().getY(), -turn.get());
        boolean isFlipped = !Robot.isBlue();
        swerve.runVelocity(
                ChassisSpeeds.fromRobotRelativeSpeeds(speeds,
                        isFlipped
                                ? swerve.getRotation().plus(new Rotation2d(Math.PI))
                                : swerve.getRotation()));
    }
}
