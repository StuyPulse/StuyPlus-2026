/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.shooter.ShooterIdle;
import com.stuypulse.robot.commands.feeder.FeederIdle;
import com.stuypulse.robot.commands.shooter.ShooterShoot;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.feeder.FeederForward;
import com.stuypulse.robot.commands.shooter.ShooterFerry;
import com.stuypulse.robot.commands.feeder.FeederReverse;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shifttimer.ShiftTimer;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.StartingLineShootAndFerry;
import com.stuypulse.robot.commands.auton.topBumpDepotOutpost;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
// import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
// import com.stuypulse.robot.commands.intake.IntakeSetDown;
// import com.stuypulse.robot.commands.intake.IntakeSetIdle;
// import com.stuypulse.robot.commands.intake.IntakeSetIntake;
// import com.stuypulse.robot.commands.intake.IntakeSetOuttake;
// import com.stuypulse.robot.commands.intake.IntakeSetUp;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final CommandXboxController operator = new CommandXboxController(Ports.Gamepad.OPERATOR);
    
    // Subsystem

    private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Feeder feeder = Feeder.getInstance();
    private final ShiftTimer shiftTimer = ShiftTimer.getInstance();
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
        driver.getTopButton()
            .whileTrue(new FeederForward());
        driver.getBottomButton()
            .whileTrue(new FeederReverse());
        driver.getRightBumper()
            .whileTrue(new ShooterShoot());
        driver.getRightButton()
            .whileTrue(new ShooterFerry());
        driver.getDPadDown()
            .whileTrue(new ShooterIdle());
        driver.getLeftBumper()
            .whileTrue(new FeederIdle());
    }

    /**************/
    /*** AUTONS ***/
    /**************/
    public void configureAutons() {
        AutonConfig topBumpDepotOutpostAuton = new AutonConfig("Top Bump Depot Outpost", topBumpDepotOutpost::new, "Left Bump to Left Neutral", "Left Neutral to Left Trench", "Left Trench to Depot", "Rotate at Depot", "Depot to Outpost");
        topBumpDepotOutpostAuton.register(autonChooser);
        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
