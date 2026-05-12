/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot;

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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;

public class Robot extends TimedRobot {

    private RobotContainer robot;

    private Command auto;

    private static Alliance alliance;

    public static boolean isBlue() {
        return alliance == Alliance.Blue;
    }

    /**
     * *********************
     */
    /**
     * ROBOT SCHEDULEING **
     */
    /**
     * *********************
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
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    }

    @Override
    public void driverStationConnected() {
        alliance = DriverStation.getAlliance().get();
    }

    @Override
    public void robotPeriodic() {
        LoggedSignals.refreshAll();
        CommandScheduler.getInstance().run();
        DogLog.log("Bot/Alliance", alliance.name());
        DogLog.log("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    }

    /**
     * **************
     */
    /**
     * SIMULATION **
     */
    /**
     * **************
     */
    @Override
    public void simulationInit() {
        // start off in a convenient spot
        CommandSwerveDrivetrain.getInstance().// start off in a convenient spot
        resetPose(SimulationConstants.ROBOTS_STARTING_POSITIONS[0]);
    }

    @Override
    public void simulationPeriodic() {
        Simulation.getInstance().update();
        RobotVisualizer.getInstance().update();
    }

    /**
     * *****************
     */
    /**
     * DISABLED MODE **
     */
    /**
     * *****************
     */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().schedule(new SetIMUMode(1));
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(MegaTagMode.MEGATAG1));
    }

    @Override
    public void disabledPeriodic() {
        CommandScheduler.getInstance().schedule(new SetIMUMode(1));
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(MegaTagMode.MEGATAG1));
    }

    /**
     * *******************
     */
    /**
     * AUTONOMOUS MODE **
     */
    /**
     * *******************
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

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    /**
     * ***************
     */
    /**
     * TELEOP MODE **
     */
    /**
     * ***************
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
        Boolean autonWon = DriverStation.getGameSpecificMessage().equals(String.valueOf(alliance.name().charAt(0)).toUpperCase());
        DogLog.log("Auton Won", autonWon);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    /**
     * *************
     */
    /**
     * TEST MODE **
     */
    /**
     * *************
     */
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
