/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.LBDisrupt;
import com.stuypulse.robot.commands.auton.LBFerry;
import com.stuypulse.robot.commands.auton.RBDisrupt;
import com.stuypulse.robot.commands.auton.RBFerry;
import com.stuypulse.robot.commands.auton.depot.CenterDepot;
import com.stuypulse.robot.commands.auton.shooting.FrontHubShootPreloads;
import com.stuypulse.robot.commands.auton.shooting.LBDumpy;
import com.stuypulse.robot.commands.auton.shooting.RBDumpy;
import com.stuypulse.robot.commands.compound.StopShooting;
import com.stuypulse.robot.commands.feeder.FeederSetForward;
import com.stuypulse.robot.commands.feeder.FeederSetReverse;
import com.stuypulse.robot.commands.handoff.HandoffSetForward;
import com.stuypulse.robot.commands.intake.IntakeAgitateFastOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.intake.IntakeSetOuttake;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.commands.shooter.ShooterAddToBonusVelocity;
import com.stuypulse.robot.commands.shooter.ShooterFirstShotIncrease;
import com.stuypulse.robot.commands.shooter.ShooterResetBonusVelocity;
import com.stuypulse.robot.commands.shooter.ShooterSetFerry;
import com.stuypulse.robot.commands.shooter.ShooterSetManual;
import com.stuypulse.robot.commands.shooter.ShooterSetShoot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForSpinUp;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetRotation;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignToFerryZone;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignToHub;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.PathUtil.AutonConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * <h2>Robot Container Class</h2>
 * 
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.configureAutoBuilder();
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
        SmartDashboard.putData("Field", Field.FIELD2D);
    }

    /** ************ */
    /** DEFAULTS ** */
    /** ************ */
    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        leds.setDefaultCommand(new LEDDefaultCommand());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    /**
     * This method is used to configure button bindings for controlling the robot.
   */
    private void configureButtonBindings() {
        // Trigger buttons did not work for some reason so I had to do this
        Trigger leftTrigger = new Trigger(() -> driver.getLeftTriggerAxis() > 0.5);
        Trigger rightTrigger = new Trigger(() -> driver.getRightTriggerAxis() > 0.5);

        leftTrigger.onTrue(new IntakeSetIntake());
        driver.leftBumper().whileTrue(new IntakeSetOuttake());
        driver.leftBumper().onFalse(new IntakeSetIntake());

        // rightTrigger.whileTrue(
        //         new WaitCommand(1.5).raceWith(new SwerveDriveAlignToHub())
        //         .andThen(new SwerveDriveXMode())
        //         .andThen(new ShooterSetShoot())
        //         .andThen(new FeederSetReverse())
        //         .andThen(new ShooterWaitForSpinUp())
        //         .andThen(new HandoffSetForward()
        //         .alongWith(new FeederSetForward(),
        //             new IntakeAgitateFastOnce().repeatedly(),
        //             new ShooterFirstShotIncrease())));
        // rightTrigger.onFalse(
        //     new StopShooting()
        // );

        // Top Left Paddle
        driver.y().onTrue(new IntakeSetIdle());
        
        // driver.rightBumper()
        //     .whileTrue(
        //         new WaitCommand(1.5).raceWith(new SwerveDriveAlignToFerryZone())
        //             .andThen(new SwerveDriveXMode())
        //             .andThen(new ShooterSetFerry())
        //             .andThen(new FeederSetReverse())
        //             .andThen(new ShooterWaitForSpinUp())
        //             .andThen(new HandoffSetForward())
        //             .alongWith(new FeederSetForward(), 
        //                 new IntakeAgitateFastOnce().repeatedly(),
        //                 new ShooterFirstShotIncrease()));
        // driver.rightBumper()
        //     .onFalse(
        //         new StopShooting()
        //     );
        // Manual shooting possibly from in front of the tower
        driver.rightTrigger()
            .whileTrue(
                new SwerveDriveXMode()
                    .andThen(new ShooterSetManual())
                    .andThen(new FeederSetReverse())
                    .andThen(new ShooterWaitForSpinUp())
                    .andThen(new HandoffSetForward().repeatedly())
                    .alongWith(new FeederSetForward().repeatedly(), 
                        new IntakeAgitateFastOnce().repeatedly(),
                        new ShooterFirstShotIncrease()));
        driver.rightTrigger()
            .onFalse(
                new StopShooting()
            );

        // driver.b()  
        //     .whileTrue(
        //         new SwerveDriveXMode()
        //             .andThen(new ShooterSetShoot())
        //             .andThen(new FeederSetReverse())
        //             .andThen(new ShooterWaitForSpinUp())
        //             .andThen(new HandoffSetForward().repeatedly())
        //             .alongWith(new FeederSetForward().repeatedly(), 
        //                 new IntakeAgitateFastOnce().repeatedly(),
        //                 new ShooterFirstShotIncrease())
        //     );
        // driver.b()
        //     .onFalse(
        //         new StopShooting()
        //     );

        // Bottom Left Paddle
        // driver.x().whileTrue(new SwerveDriveXMode());

        // driver.povLeft().onTrue(new SwerveDriveResetRotation());
        
        // driver.povUp()
        //     .onTrue(new ShooterAddToBonusVelocity(50));
        // driver.povDown()
        //     .onTrue(new ShooterAddToBonusVelocity(-50));
        // driver.povRight()
        //     .onTrue(new ShooterResetBonusVelocity());

    }

    /**************/
    /*** AUTONS ***/
    /**************/

    /**
     * This method is used to configure the autonomous commands.
     */
     public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());
        // SHOOT

        AutonConfig centerDepot = new AutonConfig("Center Depot", CenterDepot::new,
            "Hub to Depot",
            "Tower shoot");
        centerDepot.register(autonChooser);

        AutonConfig frontHubShootPreloads = new AutonConfig("Front Hub Shoot Preloads", FrontHubShootPreloads::new,
            "Front Hub Shoot Preloads"
        );
        frontHubShootPreloads.register(autonChooser);

        AutonConfig LB_Dumpy = new AutonConfig("LB Dumpy", LBDumpy::new,
            "LB to N Dumpy",
            "LB Intake Dumpy",
            "LB Backsweep Dumpy",
            "LB Shoot Dumpy",
            "LB Shoot to Depot");
        LB_Dumpy.register(autonChooser);

        AutonConfig RB_Dumpy = new AutonConfig("RB Dumpy", RBDumpy::new, 
            "RB to N Dumpy",
            "RB Intake Dumpy",
            "RB Backsweep Dumpy",
            "RB Shoot Dumpy"
        );
        RB_Dumpy.register(autonChooser);

        // FERRY
        AutonConfig LB_Ferry = new AutonConfig("LB Disrupt", LBFerry::new, 
            "LB to N Ferry",
            "N to LT Ferry",
            "LT Hub Ferry",
            "N to Depot Ferry"
        );
        LB_Ferry.register(autonChooser);

        AutonConfig RB_Ferry = new AutonConfig("LB Disrupt", RBFerry::new, 
            "RB to N Ferry",
            "N to RT Ferry",
            "RT Hub Ferry",
            "N to Outpost Ferry"
        );
        RB_Ferry.register(autonChooser);

        // DISRUPT
        AutonConfig LB_Disrupt = new AutonConfig("LB Disrupt", LBDisrupt::new, 
            "LB to CN Disrupt",
            "LN Disrupt Circle",
            "LN Disrupt Circle",
            "LB Disrupt Return"
        );
        LB_Disrupt.register(autonChooser);

        AutonConfig RB_Disrupt = new AutonConfig("RB Disrupt", RBDisrupt::new, 
            "RB to CN Disrupt",
            "RN Circle Disrupt",
            "RN Circle Disrupt",
            "RB Disrupt Return"
        );
        RB_Disrupt.register(autonChooser);

        // autonChooser.addOption("SysID Module Translation Dynamic Forwards",
        // swerve.sysIdDynamic(Direction.kForward));
        // autonChooser.addOption("SysID Module Translation Dynamic Backwards",
        // swerve.sysIdDynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Module Translation Quasi Forwards",
        // swerve.sysIdQuasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Module Translation Quasi Backwards",
        // swerve.sysIdQuasistatic(Direction.kReverse));
        // autonChooser.addOption("SysID Rotation Translation Dynamic Forwards",
        // swerve.sysidRotationDynamic(Direction.kForward));
        // autonChooser.addOption("SysID Rotation Translation Dynamic Backwards",
        // swerve.sysidRotationDynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Rotation Translation Quasi Forwards",
        // swerve.sysidRotationQuasiStatic(Direction.kForward));
        // autonChooser.addOption("SysID Rotation Translation Quasi Backwards",
        // swerve.sysidRotationQuasiStatic(Direction.kReverse));
        SmartDashboard.putData("Autonomous", autonChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}