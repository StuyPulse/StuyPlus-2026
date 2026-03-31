/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import static edu.wpi.first.units.Units.Rotation;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.LeftBumpFerry;
import com.stuypulse.robot.commands.auton.RightBumpFerry;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetRotation;
import com.stuypulse.robot.commands.swerve.SwerveDriveRotate;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
import com.stuypulse.robot.commands.intake.IntakeSetDown;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
        driver.leftTrigger()
            .whileTrue(new IntakeSetOuttake());
        driver.leftTrigger()
            .onFalse(new IntakeSetIntake());
        
        driver.rightTrigger()    
            .onTrue(new IntakeSetIntake());

        driver.rightBumper()
            .onTrue(new IntakeSetDown());

        driver.povUp()
            .onTrue(new IntakeSetIdle());

        // Turn towards alliance Zone
        driver.a()
            .whileTrue(new SwerveDriveRotate(driver, Rotation2d.k180deg));
        
        driver.x()
            .onTrue(new SwerveDriveXMode());
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
        // "N to Depot", 
        // "Depot to H.P.");
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

        autonChooser.addOption("SysID Steer Dynamic Forwards", swerve.sysidSteerDynamic(Direction.kForward));
        autonChooser.addOption("SysID Steer Dynamic Backwards", swerve.sysidSteerDynamic(Direction.kReverse));
        autonChooser.addOption("SysID Steer Quasistatic Forwards", swerve.sysidSteerQuasistatic(Direction.kForward));
        autonChooser.addOption("SysID Steer Quasistatic Backwards", swerve.sysidSteerQuasistatic(Direction.kReverse));


        SysIdRoutine intakeSysId = intake.getIntakeSysIdRoutine();
        autonChooser.addOption("Intake SysId Dynamic Forward", intakeSysId.dynamic(Direction.kForward));
        autonChooser.addOption("Intake SysId Dynamic Backward", intakeSysId.dynamic(Direction.kReverse));
        autonChooser.addOption("Intake SysId Quasistatic Forward", intakeSysId.quasistatic(Direction.kForward));
        autonChooser.addOption("Intake SysId Quasistatic Backward", intakeSysId.quasistatic(Direction.kReverse));

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
