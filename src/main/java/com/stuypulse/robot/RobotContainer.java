/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.LBDisrupt;
import com.stuypulse.robot.commands.auton.LBFerry;
import com.stuypulse.robot.commands.auton.LBMid;
import com.stuypulse.robot.commands.auton.LBStraight;
import com.stuypulse.robot.commands.auton.LTDisrupt;
import com.stuypulse.robot.commands.auton.LBMidlineSweep;
import com.stuypulse.robot.commands.auton.LBOuttake;
import com.stuypulse.robot.commands.auton.LBStraight;
import com.stuypulse.robot.commands.auton.OutpostOnly;
import com.stuypulse.robot.commands.auton.RBDisrupt;
import com.stuypulse.robot.commands.auton.RBFerry;
import com.stuypulse.robot.commands.auton.RBMid;
import com.stuypulse.robot.commands.auton.RBStraight;
import com.stuypulse.robot.commands.auton.RTDisrupt;
import com.stuypulse.robot.commands.auton.RBMidlineSweep;
import com.stuypulse.robot.commands.auton.RBOuttake;
import com.stuypulse.robot.commands.auton.RBStraight;
import com.stuypulse.robot.commands.auton.TwoMeterPath;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetRotation;
import com.stuypulse.robot.commands.swerve.SwerveDriveRotate;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
// import com.stuypulse.robot.commands.intake.IntakeAgitateWhileOuttaking;
import com.stuypulse.robot.commands.intake.IntakeSetDown;
import com.stuypulse.robot.commands.intake.IntakeSetHomingDown;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.shooter.ShooterSetShoot;
// import com.stuypulse.robot.commands.intake.IntakeSetIntake;
// import com.stuypulse.robot.commands.intake.IntakeSetOuttake;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // Gamepads
    public final CommandXboxController driver = new CommandXboxController(Ports.Gamepad.DRIVER);
    public final CommandXboxController operator = new CommandXboxController(Ports.Gamepad.OPERATOR);
    
    // Subsystem

    @SuppressWarnings("unused")
	private final Feeder feeder = Feeder.getInstance();
    @SuppressWarnings("unused")
	private final Intake intake = Intake.getInstance();
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    @SuppressWarnings("unused")
    private final LimelightVision vision = LimelightVision.getInstance();

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
        Trigger leftTrigger = new Trigger(() -> driver.getLeftTriggerAxis() > 0.5);
        Trigger rightTrigger = new Trigger(() -> driver.getRightTriggerAxis() > 0.5);
 
        // leftTrigger
        //     .whileTrue(new IntakeSetOuttake());
        // leftTrigger
        //     .onFalse(new IntakeSetHomingDown());
        leftTrigger.onTrue(new IntakeSetIdle());

        // driver.leftBumper()
        //     .onTrue(new IntakeSetIdle());
        
        // rightTrigger    
        //     .onTrue(new IntakeSetHomingDown()
        //         .andThen(new WaitUntilCommand(() -> intake.getState() == IntakeState.DOWN))
        //         .andThen(new IntakeSetIntake()));
        rightTrigger.onTrue(new IntakeSetDown());

        //Outtake with agitation
        //Top Left Paddle
        // driver.a()
        //     .whileTrue(new IntakeAgitateWhileOuttaking().repeatedly());
        driver.a()
            .onTrue(new ShooterSetShoot());

        driver.povUp()
            .onTrue(new SwerveDriveResetRotation());

        driver.rightBumper()
            .onTrue(new IntakeSetHomingDown());

        //Rotate towards alliance Zone
        //Bottom Right Paddle
        driver.y()
            .whileTrue(new SwerveDriveRotate(driver, Rotation2d.k180deg));

        //Auto Drive to Outpost
        //Top Right Paddle
        // driver.b()
        //     .whileTrue(new SwerveDrivePIDToPose(Field.outpost).andThen(new IntakeSetOuttake()));
        // driver.b() 
        //     .onFalse(new IntakeSetHomingDown());
        
        //Bottom Left Paddle
        driver.x()
            .whileTrue(new SwerveDriveXMode());

        // SOTM switches between SOTM and FOTM based on whether we're in the alliance zone or not
        driver.start()
        .whileTrue(
            new ConditionalCommand(
                new SwerveSOTM(driver).alongWith(new ShooterSOTM(), new IntakeAgitateOnce().repeatedly()),
                new SwerveFOTM(driver).alongWith(new ShooterFOTM(), new IntakeAgitateOnce().repeatedly()),
                () -> Field.inAllianceZone()))
        .onFalse(new IntakeSetIntake());

        //Manual Shooting
        operator.b()
            .whileTrue(new SwerveDriveRightCorner())
            .onFalse(new IntakeSetIntake());
        operator.x()
            .whileTrue(new SwerveDriveLeftCorner())
            .onFalse(new IntakeSetIntake());
        operator.a()
            .whileTrue(new ShooterHub().alongWith(new IntakeAgitateOnce().repeatedly()))
            .onFalse(new IntakeSetIntake());

        //Shoot or Ferry while stationary
        operator.y()
            .whileTrue(new ConditionalCommand(
                new SwerveDriveAlignedToHub().andThen(new SwerveDriveXMode().alongWith(new IntakeAgitateOnce().repeatedly())),
                new SwerveDriveAlignedToAllianceZone().andThen(new SwerveDriveXMode().alongWith(new IntakeAgitateOnce().repeatedly())),
                () -> Field.inAllianceZone()))
            .onFalse(new IntakeSetIntake());
    }

    /**************/
    /*** AUTONS ***/
    /**************/
     public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());

         
        AutonConfig LBFerry = new AutonConfig("LB Ferry", LBFerry::new, 
            "LB to N Ferry", 
            "N to LT Ferry", 
            "LT Hub Ferry", 
            "N to Depot Ferry");
        LBFerry.register(autonChooser);

        AutonConfig RBFerry = new AutonConfig("RB Ferry", RBFerry::new, 
            "RB to N Ferry", 
            "N to RT Ferry", 
            "RT Hub Ferry", 
            "N to Outpost Ferry");
        RBFerry.register(autonChooser);

        AutonConfig LBMid = new AutonConfig("LB Mid", LBMid::new,
            "LB to N Mid",
            "LB Return Mid");
        LBMid.register(autonChooser);

        AutonConfig LBStraight = new AutonConfig("LB Straight", LBStraight::new,
            "LB to N Straight",
            "N to LB Straight");
        LBStraight.register(autonChooser);

        AutonConfig RBStraight = new AutonConfig("RB Straight", RBStraight::new,
            "RB to N Straight",
            "N to RB Straight");
        RBStraight.register(autonChooser);

        AutonConfig RBMid = new AutonConfig("RB Mid", RBMid::new, 
            "RB to N Mid",
            "RB Return Mid",
            "RB to Outpost Mid");
        RBMid.register(autonChooser);

        AutonConfig TwoMeterPath = new AutonConfig("Two Meter Path", TwoMeterPath::new,
            "2 meter path");
        TwoMeterPath.register(autonChooser);

        AutonConfig OutpostOnly = new AutonConfig("Outpost Only", OutpostOnly::new, 
            "Outpost");
        OutpostOnly.register(autonChooser);

        AutonConfig RBOuttake = new AutonConfig("RB Outtake", RBOuttake::new, 
            "RB to N Outtake",
            "N to RB Outtake");
        RBOuttake.register(autonChooser);

        AutonConfig LBOuttake = new AutonConfig("LB Outtake", LBOuttake::new, 
            "LB to N Outtake",
            "N to LB Outtake");
        LBOuttake.register(autonChooser);

        AutonConfig LBMidlineSweep = new AutonConfig("LB Midline Sweep", LBMidlineSweep::new,
            "LB to N Midline",
            "LN Sweep Midline");
        LBMidlineSweep.register(autonChooser);

        AutonConfig RBMidlineSweep = new AutonConfig("RB Midline Sweep", RBMidlineSweep::new,
            "RB to N Midline",
            "RN Sweep Midline");
        RBMidlineSweep.register(autonChooser);

        AutonConfig LBDisrupt = new AutonConfig("LB Disrupt", LBDisrupt::new, 
            "LB to CN Disrupt",
            "LN Circle Disrupt",
            "LN Circle Disrupt",
            "LB Disrupt Return");
        LBDisrupt.register(autonChooser);

        AutonConfig RBDisrupt = new AutonConfig("RB Disrupt", RBDisrupt::new, 
            "RB to CN Disrupt",
            "RN Circle Disrupt",
            "RN Circle Disrupt",
            "RB Disrupt Return");
        RBDisrupt.register(autonChooser);

        AutonConfig LT_Disrupt = new AutonConfig("LT Disrupt", LTDisrupt::new, 
            "LT to N Disrupt",
            "LT Circle Disrupt",
            "LT Circle Disrupt",
            "LT Side Push Disrupt",
            "LT Around Disrupt",
            "LT Back Push Disrupt");
        LT_Disrupt.register(autonChooser);

        AutonConfig RT_Disrupt = new AutonConfig("RT Disrupt", RTDisrupt::new, 
            "RT to N Disrupt",
            "RT Circle Disrupt",
            "RT Circle Disrupt",
            "RT Side Push Disrupt",
            "RT Around Disrupt",
            "RT Back Push Disrupt");
        RT_Disrupt.register(autonChooser);
        

        // autonChooser.addOption("SysID Module Translation Dynamic Forwards", swerve.sysIdDynamic(Direction.kForward));
        // autonChooser.addOption("SysID Module Translation Dynamic Backwards", swerve.sysIdDynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Module Translation Quasi Forwards", swerve.sysIdQuasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Module Translation Quasi Backwards", swerve.sysIdQuasistatic(Direction.kReverse)); 

        // autonChooser.addOption("SysID Rotation Translation Dynamic Forwards", swerve.sysidRotationDynamic(Direction.kForward));
        // autonChooser.addOption("SysID Rotation Translation Dynamic Backwards", swerve.sysidRotationDynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Rotation Translation Quasi Forwards", swerve.sysidRotationQuasiStatic(Direction.kForward));
        // autonChooser.addOption("SysID Rotation Translation Quasi Backwards", swerve.sysidRotationQuasiStatic(Direction.kReverse)); 

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
