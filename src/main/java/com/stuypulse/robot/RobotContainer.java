/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.DoubleBump;
import com.stuypulse.robot.commands.auton.BumpToNeutralFerry;
import com.stuypulse.robot.commands.feeder.FeederIdle;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToAllianceZone;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToHub;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveWhileAligned;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveFOTM;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveSOTM;
import com.stuypulse.robot.commands.feeder.FeederForward;
import com.stuypulse.robot.commands.shooter.ShooterDefaultCommand;
import com.stuypulse.robot.commands.shooter.ShooterFOTM;
import com.stuypulse.robot.commands.shooter.ShooterSOTM;
import com.stuypulse.robot.commands.feeder.FeederReverse;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.intake.IntakeSetOuttake;
import com.stuypulse.robot.commands.led.LEDDefaultCommand;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.robot.commands.auton.LeftBumpDepotOutpost;
import com.stuypulse.robot.commands.auton.LeftBumpNeutralTwoX;
import com.stuypulse.robot.commands.auton.Depot;
import com.stuypulse.robot.commands.auton.OutpostDepotOnePointFiveCycle;
import com.stuypulse.robot.commands.auton.RightBumpNeutralTwoX;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

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
        leds.setDefaultCommand(new LEDDefaultCommand());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        //TODO:Get actual button bindings from our driver, these are just random buttons

        //SOTM which automatically switches between SOTM and FOTM based on position
        driver.getLeftButton()
            .whileTrue(
                new ConditionalCommand(
                    new SwerveSOTM(driver).alongWith(new ShooterSOTM(), new FeederForward(), new IntakeAgitateOnce().repeatedly()),
                    new SwerveFOTM(driver).alongWith(new ShooterFOTM(), new FeederForward(), new IntakeAgitateOnce().repeatedly()),
                    () -> Field.inAllianceZone()
                ))
            .onFalse(new FeederIdle().alongWith(new IntakeSetIdle()));

        //Outtaking
        driver.getRightButton()
            .whileTrue(new FeederReverse().alongWith(new IntakeSetOuttake()))
            .onFalse(new FeederIdle().alongWith(new IntakeSetIdle()));
        
        //Intaking
        driver.getLeftBumper()
            .whileTrue(new IntakeSetIntake())
            .onFalse(new IntakeSetIdle());

        //Shoot or Ferry while stationary
        driver.getRightBumper()
            .whileTrue(new ConditionalCommand(
                new SwerveDriveAlignedToHub().andThen(new SwerveDriveXMode().alongWith(new FeederForward(), new IntakeAgitateOnce().repeatedly())),
                new SwerveDriveAlignedToAllianceZone().andThen(new SwerveDriveXMode().alongWith(new FeederForward(), new IntakeAgitateOnce().repeatedly())),
                () -> Field.inAllianceZone()))
            .onFalse(new IntakeSetIdle().alongWith(new FeederIdle()));


        //Drive while aligned, just for testing purposes
        driver.getDPadUp()
            .whileTrue(new SwerveDriveDriveWhileAligned(driver, () -> Field.getHubPose()));
        driver.getDPadDown()
            .whileTrue(new SwerveDriveDriveWhileAligned(driver, () -> Field.getFerryZonePose(swerve.getPose().getTranslation())));
    }

    /**************/
    /*** AUTONS ***/
    /**************/
    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());

        AutonConfig outpostDepotOnePointFiveAuton = new AutonConfig("Outpost Depot 1.5 cycle", OutpostDepotOnePointFiveCycle::new, 
            "Right Trench to Outpost", 
            "Outpost to Depot", 
            "depot shoot", 
            "Left Bump Face Right", 
            "Left Bump to Neutral");
        outpostDepotOnePointFiveAuton.register(autonChooser);

        AutonConfig leftBumpDepotOutpostAuton = new AutonConfig("Left Bump Depot Outpost", LeftBumpDepotOutpost::new, 
            "Left Bump to Neutral", 
            "Right Facing Neutral to Left Bump", 
            "Left Bump to Depot", 
            "depot shoot",
            "Left Bump Shooting to Outpost", 
            "Outpost Shoot");
        leftBumpDepotOutpostAuton.register(autonChooser);

        AutonConfig depotAuton = new AutonConfig("Depot", Depot::new, 
            "Left Bump to Neutral", 
            "Right Facing Neutral to Left Bump", 
            "Left Bump to Depot", 
            "depot shoot");
        depotAuton.register(autonChooser);

        AutonConfig doubleBumpAuton = new AutonConfig("Double Bump", DoubleBump::new, 
            "Right Bump to Left Neutral", 
            "Left Neutral to Left Bump",
            "Left Bump Left Facing Aim at Hub");
        doubleBumpAuton.register(autonChooser);

        AutonConfig bumpToNeutralFerryAuton = new AutonConfig("Bump To Neutral Ferry", BumpToNeutralFerry::new,
            "Left Bump to Neutral");
        bumpToNeutralFerryAuton.register(autonChooser);

        AutonConfig leftBumpNeutral2XAuton = new AutonConfig("Left Bump Neutral 2X", LeftBumpNeutralTwoX::new, 
            "Left Bump to Neutral",
            "Right Facing Neutral to Left Bump",
            "Left Bump Aim at Hub",
            "Left Bump Face Right",
            "Left Bump Circle Hub",
            "Left Neutral to Left Bump");
        leftBumpNeutral2XAuton.register(autonChooser);

        AutonConfig rightBumpNeutral2XAuton = new AutonConfig("Right Bump Neutral 2X", RightBumpNeutralTwoX::new,
            "Right Bump to Neutral",
            "Neutral to Right Bump",
            "Right Bump Aim at Hub",
            "Right Bump Face Left",
            "Right Bump Circle by Hub",
            "Right Neutral to Left Bump");
        rightBumpNeutral2XAuton.register(autonChooser);

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
