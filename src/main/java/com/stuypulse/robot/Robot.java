/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import com.ctre.phoenix6.SignalLogger;
import com.stuypulse.robot.commands.vision.VisionCommands;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision.MegaTagMode;
import com.stuypulse.robot.util.LoggedSignals;
import com.stuypulse.robot.util.simulation.RobotVisualizer;
import com.stuypulse.robot.util.simulation.Simulation;
import com.stuypulse.robot.util.simulation.SimulationConstants;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * <h2>Robot Class</h2>
 * 
 * This is the main class for robot code, instantiated in {@link com.stuypulse.robot.Main} It extends TimedRobot, meaning that the methods in this
 * class are called automatically during specific states of the robot.
 * 
 * This robot is structured using the <a href="https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html">CommandBased</a> framework.
 */
public class Robot extends TimedRobot {

    private RobotContainer robot;

    private Command auto;

    private static Alliance alliance;

    /**
     * Checks the alliance the robot is on
     * @return true if the robot is on the blue alliance, false if the robot is on the red alliance
     */
    public static boolean isBlue() {
        return alliance == Alliance.Blue;
    }

    /** ********************* */
    /** ROBOT SCHEDULEING ** */
    /** ********************* */
    @Override
    public void robotInit() {
        robot = new RobotContainer();
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        } else {
            alliance = Alliance.Blue;
        }
        DataLogManager.start();
        DataLogManager.logNetworkTables(true);
        System.out.println("]LOGGING DIRECTORY]: " + DataLogManager.getLogDir());
        SignalLogger.start();
        // SmartDashboard.putData(CommandScheduler.getInstance());
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withNtTunables(true).withLogExtras(true));
    }

    /**
     * This method runs when the robot connects to Driver Station.
     * 
     * It is used to update the robot's current alliance.
     */
    @Override
    public void driverStationConnected() {
        alliance = DriverStation.getAlliance().get();
    }

    /**
     * This function is called every 20ms, regardless of the robot mode.
     */
    @Override
    public void robotPeriodic() {
        LoggedSignals.refreshAll();
        CommandScheduler.getInstance().run();
        DogLog.forceNt.log("Bot/Alliance", alliance.name());
        DogLog.forceNt.log("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    }

    /******************/
    /*** SIMULATION ***/
    /******************/

    /**
     * This method is called when the robot first starts in simulation mode.
     */
    @Override
    public void simulationInit() {
        // start off in a convenient spot
        CommandSwerveDrivetrain.getInstance()
            .resetPose(SimulationConstants.ROBOTS_STARTING_POSITIONS[0]);
    }

    /**
     * This method is called every 20ms during simulation mode.
     */
    @Override
    public void simulationPeriodic() {
        Simulation.getInstance().update();
        RobotVisualizer.getInstance().update();
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    /**
     * This method is called each time when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().schedule(VisionCommands.setIMUMode(1));
        CommandScheduler.getInstance().schedule(VisionCommands.setMegaTagMode(MegaTagMode.MEGATAG1));
    }

    /**
     * This method is called every 20ms when the robot is disabled.
     */
    @Override
    public void disabledPeriodic() {

    }

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/

    /**
     * This method is called at the start of autonomous mode.
     */
    @Override
    public void autonomousInit() {
        auto = robot.getAutonomousCommand();

        CommandScheduler.getInstance().schedule(VisionCommands.setMegaTagMode(MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(VisionCommands.setIMUMode(4));
        CommandScheduler.getInstance().schedule(VisionCommands.whitelistAllTags());

        if (auto != null) {
            CommandScheduler.getInstance().schedule(auto);
        }
    }

    /**
     * This method is called every 20ms during autonomous mode.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This method is called when autonomous mode ends.
     */
    @Override
    public void autonomousExit() {
        boolean autonWon = DriverStation.getGameSpecificMessage()
            .equals(String.valueOf(alliance.name().charAt(0)).toUpperCase());

        SmartDashboard.putBoolean("Auton Won", autonWon);
    }

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    /**
     * This method is called at the start of teleop mode.
     */
    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().schedule(VisionCommands.setMegaTagMode(MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(VisionCommands.setIMUMode(4));
        CommandScheduler.getInstance().schedule(VisionCommands.whitelistAllTags());
        if (auto != null) {
            auto.cancel();
        }
        CommandScheduler.getInstance().schedule(VisionCommands.enable());
        Boolean autonWon = DriverStation.getGameSpecificMessage()
                .equals(String.valueOf(alliance.name().charAt(0)).toUpperCase());
        DogLog.log("Auton Won", autonWon);
    }

    /**
     * This method is called every 20ms in teleop mode.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * This method is called when teleop mode ends.
     */
    @Override
    public void teleopExit() {
    }

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    /**
     * This method is called at the start of test mode.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This method is called every 20ms in test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This method is called when test mode ends.
     */
    @Override
    public void testExit() {
    }
}
