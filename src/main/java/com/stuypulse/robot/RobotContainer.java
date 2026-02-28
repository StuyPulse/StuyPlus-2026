/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.DoubleBump;
import com.stuypulse.robot.commands.auton.BumpToNeutralFerry;
import com.stuypulse.robot.commands.shooter.ShooterIdle;
import com.stuypulse.robot.commands.feeder.FeederIdle;
import com.stuypulse.robot.commands.shooter.ShooterShoot;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToAllianceZone;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToHub;
import com.stuypulse.robot.commands.feeder.FeederForward;
import com.stuypulse.robot.commands.shooter.ShooterDefaultCommand;
import com.stuypulse.robot.commands.shooter.ShooterFerry;
import com.stuypulse.robot.commands.feeder.FeederReverse;
import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.intake.IntakeSetOuttake;
import com.stuypulse.robot.commands.led.LEDApplyPattern;
import com.stuypulse.robot.commands.led.LEDDefaultCommand;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.robot.subsystems.shifttimer.ShiftTimer;
import com.stuypulse.robot.commands.auton.LeftBumpDepotOutpost;
import com.stuypulse.robot.commands.auton.DepotOnePointFiveCycle;
import com.stuypulse.robot.commands.auton.OutpostDepotOnePointFiveCycle;
import com.stuypulse.robot.commands.auton.LeftBumpOnePointFiveCycle;
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
    private final Shooter shooter = Shooter.getInstance();
    // @SuppressWarnings("unused")
    private final Feeder feeder = Feeder.getInstance();
    // @SuppressWarnings("unused") 
    private final ShiftTimer shiftTimer = ShiftTimer.getInstance();
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final LEDController leds = LEDController.getInstance();
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
        shooter.setDefaultCommand(new ShooterDefaultCommand());
       // intake.setDefaultCommand(new IntakeDefaultCommand());
        leds.setDefaultCommand(new LEDDefaultCommand());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        //Feeder
        driver.getLeftButton()
            .whileTrue(new FeederForward().alongWith(new IntakeAgitateOnce().repeatedly()));
        driver.getTopButton()
            .whileTrue(new FeederIdle());  
        driver.getRightButton()
            .whileTrue(new FeederReverse().alongWith(new IntakeSetOuttake()));
        
        //Intake
        driver.getLeftBumper()
            .whileTrue(new IntakeSetIntake());
        driver.getRightBumper()
            .whileTrue(new IntakeSetIdle());


        //Shooter Alignment
        driver.getDPadUp()
            .whileTrue(new SwerveDriveAlignedToHub());
        driver.getDPadDown()
            .whileTrue(new SwerveDriveAlignedToAllianceZone());
    }

    /**************/
    /*** AUTONS ***/
    /**************/
    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());


        AutonConfig outpostDepotOnePointFiveAuton = new AutonConfig("Outpost Depot 1.5 cycle", OutpostDepotOnePointFiveCycle::new, 
            "Right Trench to Outpost", 
            "Outpost to Depot", 
            "Rotate at Depot", 
            "Depot to Neutral", 
            "Intake Neutral and Middle Ferry");
        outpostDepotOnePointFiveAuton.register(autonChooser);

        AutonConfig leftBumpDepotOutpostAuton = new AutonConfig("Left Bump Depot Outpost", LeftBumpDepotOutpost::new, 
            "Left Bump to Left Neutral", 
            "Left Neutral to Left Bump", 
            "Left Bump to Depot", 
            "Rotate at Depot", 
            "Depot to Outpost");
        leftBumpDepotOutpostAuton.register(autonChooser);

        AutonConfig depotOnePointFiveAuton = new AutonConfig("Depot 1.5 cycle", DepotOnePointFiveCycle::new, 
            "Left Trench to Depot", 
            "Rotate at Depot", 
            "Depot to Right Bump", 
            "Right Bump to Left Neutral");
        depotOnePointFiveAuton.register(autonChooser);

        AutonConfig doubleBumpAuton = new AutonConfig("Double Bump", DoubleBump::new, 
            "Right Bump to Left Neutral", 
            "Left Neutral to Left Bump");
        doubleBumpAuton.register(autonChooser);

        AutonConfig bumpToNeutralFerry = new AutonConfig("Bump To Neutral Ferry", BumpToNeutralFerry::new,
            "Bump to Neutral",
            "Intake Neutral and Ferry");
        bumpToNeutralFerry.register(autonChooser);

        AutonConfig leftBumpOnePointFiveCycle = new AutonConfig("Left Bump 1.5 cycle", LeftBumpOnePointFiveCycle::new, 
             "Left Bump to Left Neutral", 
             "Left Neutral to Left Bump", 
             "Left Bump to Depot");
         leftBumpOnePointFiveCycle.register(autonChooser);

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
