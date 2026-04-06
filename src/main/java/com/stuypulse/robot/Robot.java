/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.vision.SetIMUMode;
import com.stuypulse.robot.commands.vision.SetMegaTagMode;
import com.stuypulse.robot.commands.vision.WhitelistAllTags;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision.MegaTagMode;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.simulation.Simulation;
import com.stuypulse.robot.util.simulation.SimulationConstants;

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
        // SmartDashboard.putData(CommandScheduler.getInstance());
    }

    @Override
    public void driverStationConnected() {
        alliance = DriverStation.getAlliance().get();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putString("Bot/Alliance", alliance.name());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    /******************/
    /*** SIMULATION ***/
    /******************/
    @Override
    public void simulationInit() {
        CommandSwerveDrivetrain
            .getInstance()
            .getMapleSimDrive()
            .setSimulationWorldPose(Robot.isBlue() ? SimulationConstants.ROBOTS_STARTING_POSITIONS[0] : Field.transformToOppositeAlliance(SimulationConstants.ROBOTS_STARTING_POSITIONS[0])); // start off in a convenient spot
    }

    @Override
    public void simulationPeriodic() {
        Simulation.getInstance().update();
    }


    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().schedule(new SetIMUMode(1));
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(MegaTagMode.MEGATAG1));
    }

    @Override
    public void disabledPeriodic() {}

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/

    @Override
    public void autonomousInit() {
        auto = robot.getAutonomousCommand();

        CommandScheduler.getInstance().schedule(new SetMegaTagMode(MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(new SetIMUMode(4));
        CommandScheduler.getInstance().schedule(new WhitelistAllTags());

        if (auto != null) {
            CommandScheduler.getInstance().schedule(auto);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(new SetIMUMode(4));
        CommandScheduler.getInstance().schedule(new WhitelistAllTags());

        if (auto != null) {
            auto.cancel();
        }

        Boolean autonWon = DriverStation.getGameSpecificMessage().equals(String.valueOf(alliance.name().charAt(0)).toUpperCase());

        SmartDashboard.putBoolean("Auton Won", autonWon);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {

        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
