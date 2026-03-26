/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 0.020;
    boolean DEBUG_MODE = true;
    CANBus CANIVORE = new CANBus("swerve");

    public interface EnabledSubsystems {
        // SmartBoolean FEEDER = new SmartBoolean("Enabled Subsystems/Feeder", true);
        SmartBoolean INTAKE = new SmartBoolean("Enabled Subsystems/Intake", true);
        // SmartBoolean LED = new SmartBoolean("Enabled Subsystems/LED", true);
        // SmartBoolean SHOOTER = new SmartBoolean("Enabled Subsystems/Shooter", true);
        SmartBoolean VISION = new SmartBoolean("Enabled Subsystems/Vision", true);
        SmartBoolean SWERVE = new SmartBoolean("Enabled Subsystems/Swerve", true);
    }

    public interface Intake {
        double PIVOT_LENGTH = 1.0; // TODO: get actual values
        double ROLLER_MAX_ACCEL = 0;
        double ROLLER_MAX_VEL = 0;
        double PIVOT_INITIAL_ANGLE = 30;

        double IDLE_DUTY_CYCLE = 0;

        Rotation2d INTAKE_ANGLE = Rotation2d.fromDegrees(150);
        double INTAKE_DUTY_CYCLE = 0.8;

        Rotation2d OUTTAKE_ANGLE = Rotation2d.fromDegrees(150);
        double OUTTAKE_DUTY_CYCLE = -0.8;

        Rotation2d IDLE_ANGLE = Rotation2d.fromDegrees(30);

        Rotation2d AGITATE_UP_ANGLE = Rotation2d.fromDegrees(115);
        Rotation2d AGITATE_DOWN_ANGLE = Rotation2d.fromDegrees(67);

        double J_KG_METERS_SQUARED = 0.1;
        double PIVOT_MIN_ANGLE = 0.0;
        double PIVOT_MAX_ANGLE = 2 * Math.PI;
        double PIVOT_GEAR_RATIO = 20.0;
        double ROLLER_GEAR_RATIO = 20.0;

        double RAMP_RATE = 2;
        double STEP_VOLTAGE = 6; // volts

        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.5); // degrees
    }

    // public interface Feeder {
    //     double FEEDER_REVERSE = -1900;
    //     double FEEDER_FORWARD = 1900;
    // }

    // public interface LED {
    //     int LED_LENGTH = 60; // TODO: ask Plus-ME for LED Length
 
    //     //shooter
    //     LEDPattern SHOOTING = LEDPattern.solid(Color.kOrange);
    //     LEDPattern FERRYING = LEDPattern.solid(Color.kPurple);

    //     //feeder
    //     LEDPattern FEEDER_FORWARD = LEDPattern.solid(Color.kBlue);
    //     LEDPattern FEEDER_REVERSE = LEDPattern.solid(Color.kRed);

    //     //take
    //     LEDPattern INTAKING = LEDPattern.solid(Color.kYellow);
    //     LEDPattern OUTTAKING = LEDPattern.solid(Color.kGreen);
    //     LEDPattern AGITATING = LEDPattern.solid(Color.kCyan);
        
    //     //states
    //     LEDPattern DISABLED = LEDPattern.solid(Color.kGray);
    // }

    // public interface Shooter {
    //     double BOTTOM_MOTOR_RPM = 3000;
    //     double SHOOT_TIME_AUTO = 1.5;
    //     double RAMP_RATE = 0.25;
    //     double STEP_VOLTAGE = 900;

    //     double CORNER = 2700; // TODO: Test RPM
    //     double HUB = 2500;
    // }

    public interface Swerve {
        double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;
        
        public interface Constraints {    
            double MAX_VELOCITY_M_PER_S = 4.3;
            double MAX_ACCEL_M_PER_S_SQUARED = 15.0;
            double MAX_ANGULAR_VEL_RAD_PER_S = Units.degreesToRadians(400.0);
            double MAX_ANGULAR_ACCEL_RAD_PER_S = Units.degreesToRadians(900.0);
    
            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY_M_PER_S,
                    MAX_ACCEL_M_PER_S_SQUARED,
                    MAX_ANGULAR_VEL_RAD_PER_S,
                    MAX_ANGULAR_ACCEL_RAD_PER_S);
        }

        public interface Alignment {
            public interface Constraints {
                double DEFAULT_MAX_VELOCITY = 4.3;
                double DEFAULT_MAX_ACCELERATION = 15.0;
                double DEFAULT_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(400.0);
                double DEFAULT_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(900.0);
            }

            public interface Tolerances {
                double X_TOLERANCE = Units.inchesToMeters(2.0); 
                double Y_TOLERANCE = Units.inchesToMeters(2.0);
                Rotation2d THETA_TOLERANCE = Rotation2d.fromDegrees(1);

                Pose2d POSE_TOLERANCE = new Pose2d(
                    Units.inchesToMeters(2.0), 
                    Units.inchesToMeters(2.0), 
                    Rotation2d.fromDegrees(2.0));

                double MAX_VELOCITY_WHEN_ALIGNED = 0.15;

                double ALIGNMENT_DEBOUNCE = 0.15;
            }

            public interface Targets {
                Rotation2d HUB_LEFT_CORNER = Rotation2d.fromDegrees(45); //TODO: Get actual angle
                Rotation2d HUB_RIGHT_CORNER = Rotation2d.fromDegrees(-45);
            }
        }
    }
    public interface Driver {
        double BUZZ_TIME = 1.0;
        double BUZZ_INTENSITY = 1.0;

        public interface Drive {
            double DEADBAND = 0.05;

            double RC = 0.05;
            double POWER = 2.0;
        }

        public interface Turn {
            double DEADBAND = 0.05;

            double RC = 0.05;
            double POWER = 2.0;
        }
    }
}
