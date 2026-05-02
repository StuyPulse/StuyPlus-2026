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
import com.stuypulse.robot.commands.auton.OutpostOnly;
import com.stuypulse.robot.commands.auton.RBDisrupt;
import com.stuypulse.robot.commands.auton.RBFerry;
import com.stuypulse.robot.commands.auton.RBMid;
import com.stuypulse.robot.commands.auton.RBStraight;
import com.stuypulse.robot.commands.auton.RTDisrupt;
import com.stuypulse.robot.commands.auton.RBMidlineSweep;
import com.stuypulse.robot.commands.auton.RBOuttake;
import com.stuypulse.robot.commands.auton.TwoMeterPath;
import com.stuypulse.robot.commands.feeder.FeederCommands;
import com.stuypulse.robot.commands.handoff.HandoffCommands;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetRotation;
import com.stuypulse.robot.commands.swerve.SwerveDriveRotate;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignToFerryZone;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignToHub;
import com.stuypulse.robot.commands.intake.IntakeCommands;
// import com.stuypulse.robot.commands.intake.IntakeAgitateWhileOuttaking;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.commands.shooter.ShooterCommands;
// import com.stuypulse.robot.commands.intake.IntakeSetIntake;
// import com.stuypulse.robot.commands.intake.IntakeSetOuttake;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.PathUtil.AutonConfig;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class RobotContainer {
    // Gamepads
    public final CommandXboxController driver = new CommandXboxController(Ports.Gamepad.DRIVER);
    
    // Subsystem

	private final Feeder feeder = Feeder.getInstance();
	private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final LimelightVision vision = LimelightVision.getInstance();
    private final LEDController leds = LEDController.getInstance();
    private final Handoff handoff = Handoff.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        swerve.configureAutoBuilder();
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        configureIntakeLogic();
        configureFeederLogic();
        configureHandoffLogic();
        
        SmartDashboard.putData("Field", Field.FIELD2D);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        leds.setDefaultCommand(new LEDDefaultCommand());
    }

    //Subsystem automatic logic

    private void configureIntakeLogic() {
        intake.pivotStalling()
            .onTrue(IntakeCommands.setZeroAtBottom());
    }

    private void configureFeederLogic() {
        swerve.notAlignedToHub().and(() -> shooter.getState() == ShooterState.SHOOT)
            .whileTrue(FeederCommands.setIdle().repeatedly());

        swerve.notAlignedToFerryZone().and(() -> shooter.getState() == ShooterState.FERRY)
            .whileTrue(FeederCommands.setIdle().repeatedly());
    }

    private void configureHandoffLogic() {
        Trigger cannotShoot =  swerve.notAlignedToHub().and(() -> shooter.getState() == ShooterState.SHOOT);
        Trigger cannotFerry = swerve.notAlignedToFerryZone().and(() -> shooter.getState() == ShooterState.FERRY);

        cannotShoot
            .whileTrue(HandoffCommands.setIdle().repeatedly());

        cannotFerry
            .whileTrue(HandoffCommands.setIdle().repeatedly());

        handoff.handoffStalling()
            .whileTrue(HandoffCommands.setReverse().repeatedly())
            .onFalse(Commands.either(HandoffCommands.setIdle(), 
                                     HandoffCommands.setForward(), 
                                     cannotShoot.or(cannotFerry)));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        //Trigger buttons did not work for some reason so I had to do this
        Trigger leftTrigger = new Trigger(() -> driver.getLeftTriggerAxis() > 0.5);
        Trigger rightTrigger = new Trigger(() -> driver.getRightTriggerAxis() > 0.5);
 
        leftTrigger.onTrue(IntakeCommands.setIntake());

        driver.leftBumper()
            .whileTrue(IntakeCommands.setOuttake());
        driver.leftBumper()
            .onFalse(IntakeCommands.setIntake());

        rightTrigger
            .onTrue(new SwerveDriveAlignToHub()
                    .andThen(new SwerveDriveXMode()
                    .alongWith(ShooterCommands.setShoot()
                        .andThen(HandoffCommands.setForward()
                        .alongWith(FeederCommands.setForward(), IntakeCommands.agitateOnce().repeatedly())))));

        //Top Left Paddle
        driver.a()
            .onTrue(IntakeCommands.setIdle());

        driver.povUp()
            .onTrue(new SwerveDriveResetRotation());

        driver.rightBumper()
            .onTrue(new SwerveDriveAlignToFerryZone()
                    .andThen(new SwerveDriveXMode()
                    .alongWith(ShooterCommands.setFerry()
                        .andThen(HandoffCommands.setForward()
                        .alongWith(FeederCommands.setForward(), IntakeCommands.agitateOnce().repeatedly())))));

        //Bottom Right Paddle
        //Manual shooting possibly from in front of the hub
        driver.y()
            .onTrue(new SwerveDriveXMode()
                    .alongWith(ShooterCommands.setManual()
                                .andThen(HandoffCommands.setForward()
                                        .alongWith(FeederCommands.setForward(), IntakeCommands.agitateOnce().repeatedly()))));

        //Top Right Paddle
        driver.b()
            .onTrue(HandoffCommands.setIdle()
                    .alongWith(FeederCommands.setIdle(), IntakeCommands.setIntake(), Commands.runOnce(() -> new SwerveDriveXMode().cancel())));
        
        //Bottom Left Paddle
        driver.x()
            .whileTrue(new SwerveDriveXMode());
    }

    /**************/
    /*** AUTONS ***/
    /**************/
     public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());

         
        // AutonConfig LBFerry = new AutonConfig("LB Ferry", LBFerry::new, 
        //     "LB to N Ferry", 
        //     "N to LT Ferry", 
        //     "LT Hub Ferry", 
        //     "N to Depot Ferry");
        // LBFerry.register(autonChooser);

        // AutonConfig RBFerry = new AutonConfig("RB Ferry", RBFerry::new, 
        //     "RB to N Ferry", 
        //     "N to RT Ferry", 
        //     "RT Hub Ferry", 
        //     "N to Outpost Ferry");
        // RBFerry.register(autonChooser);

        // AutonConfig LBMid = new AutonConfig("LB Mid", LBMid::new,
        //     "LB to N Mid",
        //     "LB Return Mid");
        // LBMid.register(autonChooser);

        // AutonConfig LBStraight = new AutonConfig("LB Straight", LBStraight::new,
        //     "LB to N Straight",
        //     "N to LB Straight");
        // LBStraight.register(autonChooser);

        // AutonConfig RBStraight = new AutonConfig("RB Straight", RBStraight::new,
        //     "RB to N Straight",
        //     "N to RB Straight");
        // RBStraight.register(autonChooser);

        // AutonConfig RBMid = new AutonConfig("RB Mid", RBMid::new, 
        //     "RB to N Mid",
        //     "RB Return Mid",
        //     "RB to Outpost Mid");
        // RBMid.register(autonChooser);

        // AutonConfig TwoMeterPath = new AutonConfig("Two Meter Path", TwoMeterPath::new,
        //     "2 meter path");
        // TwoMeterPath.register(autonChooser);

        // AutonConfig OutpostOnly = new AutonConfig("Outpost Only", OutpostOnly::new, 
        //     "Outpost");
        // OutpostOnly.register(autonChooser);

        // AutonConfig RBOuttake = new AutonConfig("RB Outtake", RBOuttake::new, 
        //     "RB to N Outtake",
        //     "N to RB Outtake");
        // RBOuttake.register(autonChooser);

        // AutonConfig LBOuttake = new AutonConfig("LB Outtake", LBOuttake::new, 
        //     "LB to N Outtake",
        //     "N to LB Outtake");
        // LBOuttake.register(autonChooser);

        // AutonConfig LBMidlineSweep = new AutonConfig("LB Midline Sweep", LBMidlineSweep::new,
        //     "LB to N Midline",
        //     "LN Sweep Midline");
        // LBMidlineSweep.register(autonChooser);

        // AutonConfig RBMidlineSweep = new AutonConfig("RB Midline Sweep", RBMidlineSweep::new,
        //     "RB to N Midline",
        //     "RN Sweep Midline");
        // RBMidlineSweep.register(autonChooser);

        // AutonConfig LBDisrupt = new AutonConfig("LB Disrupt", LBDisrupt::new, 
        //     "LB to CN Disrupt",
        //     "LN Circle Disrupt",
        //     "LN Circle Disrupt",
        //     "LB Disrupt Return");
        // LBDisrupt.register(autonChooser);

        // AutonConfig RBDisrupt = new AutonConfig("RB Disrupt", RBDisrupt::new, 
        //     "RB to CN Disrupt",
        //     "RN Circle Disrupt",
        //     "RN Circle Disrupt",
        //     "RB Disrupt Return");
        // RBDisrupt.register(autonChooser);

        // AutonConfig LT_Disrupt = new AutonConfig("LT Disrupt", LTDisrupt::new, 
        //     "LT to N Disrupt",
        //     "LT Circle Disrupt",
        //     "LT Circle Disrupt",
        //     "LT Side Push Disrupt",
        //     "LT Around Disrupt",
        //     "LT Back Push Disrupt");
        // LT_Disrupt.register(autonChooser);

        // AutonConfig RT_Disrupt = new AutonConfig("RT Disrupt", RTDisrupt::new, 
        //     "RT to N Disrupt",
        //     "RT Circle Disrupt",
        //     "RT Circle Disrupt",
        //     "RT Side Push Disrupt",
        //     "RT Around Disrupt",
        //     "RT Back Push Disrupt");
        // RT_Disrupt.register(autonChooser);
        

        // autonChooser.addOption("SysID Module Translation Dynamic Forwards", swerve.sysIdDynamic(Direction.kForward));
        // autonChooser.addOption("SysID Module Translation Dynamic Backwards", swerve.sysIdDynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Module Translation Quasi Forwards", swerve.sysIdQuasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Module Translation Quasi Backwards", swerve.sysIdQuasistatic(Direction.kReverse)); 

        // autonChooser.addOption("SysID Rotation Translation Dynamic Forwards", swerve.sysidRotationDynamic(Direction.kForward));
        // autonChooser.addOption("SysID Rotation Translation Dynamic Backwards", swerve.sysidRotationDynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Rotation Translation Quasi Forwards", swerve.sysidRotationQuasiStatic(Direction.kForward));
        // autonChooser.addOption("SysID Rotation Translation Quasi Backwards", swerve.sysidRotationQuasiStatic(Direction.kReverse)); 

        autonChooser.addOption("Intake Pivot SysID  Quasi Forwards", intake.getIntakeSysIdRoutine().quasistatic(Direction.kForward));
        autonChooser.addOption("Intake Pivot SysID  Quasi Reverse", intake.getIntakeSysIdRoutine().quasistatic(Direction.kReverse));
        autonChooser.addOption("Intake Pivot SysID  Dynamic Forwards", intake.getIntakeSysIdRoutine().dynamic(Direction.kForward));
        autonChooser.addOption("Intake Pivot SysID  Dynamic Reverse", intake.getIntakeSysIdRoutine().dynamic(Direction.kReverse));

        autonChooser.addOption("Shooter SysID Quasi Forwards ", shooter.getShooterSysIdRoutine().quasistatic(Direction.kForward));
        autonChooser.addOption("Shooter SysID Quasi Reverse ", shooter.getShooterSysIdRoutine().quasistatic(Direction.kReverse));
        autonChooser.addOption("Shooter SysID Dynamic Forward ", shooter.getShooterSysIdRoutine().dynamic(Direction.kForward));
        autonChooser.addOption("Shooter SysID Dynamic Reverse ", shooter.getShooterSysIdRoutine().dynamic(Direction.kReverse));

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}