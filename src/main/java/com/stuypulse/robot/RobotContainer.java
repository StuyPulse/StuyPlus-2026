/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetRotation;
import com.stuypulse.robot.commands.swerve.SwerveDriveRotate;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.PIDtoPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.commands.intake.IntakeAgitateWhileOuttaking;
import com.stuypulse.robot.commands.intake.IntakeSetDown;
import com.stuypulse.robot.commands.intake.IntakeSetHoming;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.intake.IntakeSetOuttake;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {
    // Gamepads
    public final CommandXboxController driver = new CommandXboxController(Ports.Gamepad.DRIVER);
    
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
        //Trigger buttons did not work for some reason so I had to do this
        Trigger leftTrigger = new Trigger(() -> driver.getLeftTriggerAxis() > 1.5);
        Trigger rightTrigger = new Trigger(() -> driver.getRightTriggerAxis() > 0.5);

        leftTrigger
            .whileTrue(new IntakeAgitateWhileOuttaking().repeatedly());
        leftTrigger
            .onFalse(new IntakeSetIntake());

        driver.leftBumper()
            .onTrue(new IntakeSetIdle());
        
        rightTrigger    
            .onTrue(new IntakeSetIntake());

        driver.povDown()
            .whileTrue(new IntakeSetOuttake());
        driver.povDown()
            .onFalse(new IntakeSetIntake());

        driver.povUp()
            .onTrue(new SwerveDriveResetRotation());

        driver.rightBumper()
            .onTrue(new IntakeSetDown());

        //Rotate towards alliance Zone
        //Top Left Paddle
        driver.a()
            .whileTrue(new SwerveDriveRotate(driver, Rotation2d.k180deg));

        //Auto Drive to Outpost
        //Top Right Paddle
        driver.b()
            .whileTrue(new SwerveDrivePIDToPose(Field.outpost).andThen(new IntakeAgitateWhileOuttaking().repeatedly()));
        driver.b() 
            .onFalse(new IntakeSetIntake());
        
        //Bottom Left Paddle
        driver.x()
            .whileTrue(new SwerveDriveXMode());

        driver.y()  
            .onTrue(new IntakeSetHoming());
    }

    /**************/
    /*** AUTONS ***/
    /**************/
    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());

        // AutonConfig LeftBumpFerry = new AutonConfig("Left Bump Ferry", LeftBumpFerry::new, 
        // "Left Bump to Neutral", 
        // "N to L.T.", 
        // "L.T. Circle Hub", 

        // "N to Depot");
        // LeftBumpFerry.register(autonChooser);

        // AutonConfig RightBumpFerry = new AutonConfig("Right Bump Ferry", RightBumpFerry::new, 
        // "R.B. to R.N.", 
        // "N to R.T.", 
        // "R.T. Circle Hub", 
        // "R.N. to H.P.");
        // RightBumpFerry.register(autonChooser);

        autonChooser.addOption("SysID Module Translation Dynamic Forwards", swerve.sysIdDynamic(Direction.kForward));
        autonChooser.addOption("SysID Module Translation Dynamic Backwards", swerve.sysIdDynamic(Direction.kReverse));
        autonChooser.addOption("SysID Module Translation Quasi Forwards", swerve.sysIdQuasistatic(Direction.kForward));
        autonChooser.addOption("SysID Module Translation Quasi Backwards", swerve.sysIdQuasistatic(Direction.kReverse)); 

        autonChooser.addOption("SysID Rotation Translation Dynamic Forwards", swerve.sysidRotationDynamic(Direction.kForward));
        autonChooser.addOption("SysID Rotation Translation Dynamic Backwards", swerve.sysidRotationDynamic(Direction.kReverse));
        autonChooser.addOption("SysID Rotation Translation Quasi Forwards", swerve.sysidRotationQuasiStatic(Direction.kForward));
        autonChooser.addOption("SysID Rotation Translation Quasi Backwards", swerve.sysidRotationQuasiStatic(Direction.kReverse)); 

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
