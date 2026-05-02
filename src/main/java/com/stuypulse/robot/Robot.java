/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.ctre.phoenix6.SignalLogger;
import com.stuypulse.robot.commands.vision.SetIMUMode;
import com.stuypulse.robot.commands.vision.SetMegaTagMode;
import com.stuypulse.robot.commands.vision.SetVisionEnabled;
import com.stuypulse.robot.commands.vision.WhitelistAllTags;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.subsystems.vision.LimelightVision.MegaTagMode;

import com.stuypulse.robot.util.LoggedSignals;
import com.stuypulse.robot.util.RobotVisualizer;
import com.stuypulse.robot.util.simulation.Simulation;
import com.stuypulse.robot.util.simulation.SimulationConstants;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * <h2>Robot Class</h2>
 * 
 * This is the main class for robot code, instantiated in {@link Main.java} It extends TimedRobot, meaning that the methods in this
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

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    /** 
     * This method is called when the robot is first started up.
    */
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

        SmartDashboard.putString("Bot/Alliance", alliance.name());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
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
        CommandSwerveDrivetrain
                .getInstance()
                .resetPose(SimulationConstants.ROBOTS_STARTING_POSITIONS[0]); // start off in a convenient spot
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
        CommandScheduler.getInstance().schedule(new SetIMUMode(1));
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(MegaTagMode.MEGATAG1));
    }

    /**
     * This method is called every 20ms when the robot is disabled.
     */
    @Override
    public void disabledPeriodic() {
        CommandScheduler.getInstance().schedule(new SetIMUMode(1));
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(MegaTagMode.MEGATAG1));
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

        CommandScheduler.getInstance().schedule(new SetMegaTagMode(MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(new SetIMUMode(4));
        CommandScheduler.getInstance().schedule(new WhitelistAllTags());

        LimelightVision.getInstance().disable();

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
    }

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    /**
     * This method is called at the start of teleop mode.
     */
    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(new SetIMUMode(4));
        CommandScheduler.getInstance().schedule(new WhitelistAllTags());

        if (auto != null) {
            auto.cancel();
        }

        CommandScheduler.getInstance().schedule(new SetVisionEnabled());

        Boolean autonWon = DriverStation.getGameSpecificMessage()
                .equals(String.valueOf(alliance.name().charAt(0)).toUpperCase());

        SmartDashboard.putBoolean("Auton Won", autonWon);
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
