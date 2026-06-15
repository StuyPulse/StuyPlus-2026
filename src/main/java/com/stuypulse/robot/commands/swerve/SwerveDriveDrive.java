/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.swerve.swerveinput.DriveInputProcessor;
import com.stuypulse.robot.util.swerve.swerveinput.DriveTurnInputProcessor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveDrive extends Command {

	private final CommandSwerveDrivetrain swerve;

	private final CommandXboxController driver;

	private final DriveInputProcessor speed;

	private final DriveTurnInputProcessor turn;

	public SwerveDriveDrive(CommandXboxController driver) {
		swerve = CommandSwerveDrivetrain.getInstance();
		this.speed = new DriveInputProcessor(
                driver, 
                Drive.DEADBAND, 
                Drive.POWER, 
                Swerve.Constraints.MAX_VELOCITY_M_PER_S, 
                Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED, 
                Drive.RC);
		turn = new DriveTurnInputProcessor(
			driver, 
			Turn.DEADBAND, 
			Turn.POWER, 
			Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S, Turn.RC);
		this.driver = driver;
		addRequirements(swerve);
	}

	@Override
	public void execute() {
		speed.update();
		turn.update();

		swerve.setControl(
				swerve
						.getFieldCentricSwerveRequest()
						.withVelocityX(speed.get().getX())
						.withVelocityY(speed.get().getY())
						.withRotationalRate(-turn.get()));
	}
}
