/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.LeftBumpFerry;
import com.stuypulse.robot.commands.auton.RightBumpFerry;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetRotation;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.intake.IntakeSetOuttake;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final CommandXboxController operator = new CommandXboxController(Ports.Gamepad.OPERATOR);
    
    // Subsystem

    // @SuppressWarnings("unused")
	private final Intake intake = Intake.getInstance();
    // @SuppressWarnings("unused")
    // @SuppressWarnings("unused")
    // @SuppressWarnings("unused")
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        swerve.configureAutoBuilder();
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
        
        SmartDashboard.putData("Field", Field.FIELD2D);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        //Outtaking
        driver.getDPadRight()
            .whileTrue(new IntakeSetOuttake())
            .onFalse(new IntakeSetIntake());

        //Reset Heading
        driver.getDPadUp()
            .onTrue(new SwerveDriveResetRotation());
        
        //X Mode
        driver.getLeftBumper()
            .onTrue(new SwerveDriveXMode());            

        //Toggle Intake On or Off
        driver.getLeftButton()
            .onTrue(new IntakeSetIdle());

        driver.getRightButton()
            .onTrue(new IntakeSetIntake());
    }

    /**************/
    /*** AUTONS ***/
    /**************/
    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());

        AutonConfig LeftBumpFerry = new AutonConfig("Left Bump Ferry", LeftBumpFerry::new, 
        "Left Bump to Neutral", 
        "N to L.T.", 
        "L.T. Circle Hub", 
        "N to Depot", 
        "Depot to H.P.");
        LeftBumpFerry.register(autonChooser);

        AutonConfig RightBumpFerry = new AutonConfig("Right Bump Ferry", RightBumpFerry::new, 
        "R.B. to R.N.", 
        "N to R.T.", 
        "R.T. Circle Hub", 
        "R.N. to H.P.");
        RightBumpFerry.register(autonChooser);

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
