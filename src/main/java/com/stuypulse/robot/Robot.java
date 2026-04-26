/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.ctre.phoenix6.SignalLogger;
import com.stuypulse.robot.commands.vision.VisionCommands;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
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

public class Robot extends TimedRobot {

    private RobotContainer robot;
    private Command auto;
    private static Alliance alliance;

    public static boolean isBlue() {
        return alliance == Alliance.Blue;
    }

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

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

    @Override
    public void driverStationConnected() {
        alliance = DriverStation.getAlliance().get();
    }

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
    @Override
    public void simulationInit() {
        CommandSwerveDrivetrain
                .getInstance()
                .resetPose(SimulationConstants.ROBOTS_STARTING_POSITIONS[0]); // start off in a convenient spot
    }

    @Override
    public void simulationPeriodic() {
        Simulation.getInstance().update();
        RobotVisualizer.getInstance().update();
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().schedule(VisionCommands.setIMUMode(1));
        CommandScheduler.getInstance().schedule(VisionCommands.setMegaTagMode(MegaTagMode.MEGATAG1));
    }

    @Override
    public void disabledPeriodic() {

    }

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/

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

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

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

        SmartDashboard.putBoolean("Auton Won", autonWon);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {

        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
