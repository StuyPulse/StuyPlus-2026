/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
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
    CANBus CANIVORE = new CANBus("rio");

    public interface EnabledSubsystems {
        // SmartBoolean FEEDER = new SmartBoolean("Enabled Subsystems/Feeder", true);
        SmartBoolean INTAKE = new SmartBoolean("Enabled Subsystems/Intake", false);
        // SmartBoolean LED = new SmartBoolean("Enabled Subsystems/LED", true);
        // SmartBoolean SHOOTER = new SmartBoolean("Enabled Subsystems/Shooter", true);
        SmartBoolean VISION = new SmartBoolean("Enabled Subsystems/Vision", false);
        SmartBoolean SWERVE = new SmartBoolean("Enabled Subsystems/Swerve", true);
    }

    public interface Vision {
        //TODO: These numbers are temporary, may need testing
        public final Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        public final Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694.0);

        public final Translation2d INVALID_POSITION = new Translation2d(8.2705, 4.0345);
        public final double INVALID_POSITION_TOLERANCE_M = 0.05;
        public final double MAX_ANGULAR_VELOCITY_RAD_SEC = 2 * Math.PI;

    }

    public interface Intake {
        double PIVOT_LENGTH = 1.0; // TODO: get actual values
        double ROLLER_MAX_ACCEL = 0;
        double ROLLER_MAX_VEL = 0;
        double PIVOT_STALL_CURRENT = 55; // TODO: set
        double PIVOT_STALL_DEBOUNCE_SEC = 1;
        double ROLLER_STALL_CURRENT = 55; // TODO: set
        double ROLLER_STALL_DEBOUNCE_SEC = 1;
        Rotation2d PIVOT_INITIAL_ANGLE = Rotation2d.fromDegrees(0);

        Rotation2d IDLE_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d PIVOT_DOWN_ANGLE = Rotation2d.fromDegrees(122);

        double HOMING_VOLTAGE = -3;

        double IDLE_DUTY_CYCLE = 0;
        double INTAKE_DUTY_CYCLE = 1;
        double OUTTAKE_DUTY_CYCLE = -1;

        double J_KG_METERS_SQUARED = 0.1;
        double PIVOT_MIN_ANGLE = 0.0;
        double PIVOT_MAX_ANGLE = 2 * Math.PI;
        double PIVOT_GEAR_RATIO = 60.0;
        double ROLLER_GEAR_RATIO = 16/27;

        double RAMP_RATE = 2;
        double STEP_VOLTAGE = 6; // volts

        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.5); 

        Rotation2d PUSHDOWN_THRESHOLD = Rotation2d.fromDegrees(107); //TODO:Temporary, needs testing
        double PUSHDOWN_VOLTAGE = 0;
    }

    // public interface Feeder {
    //     double FEEDER_REVERSE = -1900;
    //     double FEEDER_FORWARD = 1900;
    // }

    // public interface LED {
    //     int LED_LENGTH = 60; 
 
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

    //     double CORNER = 2700; 
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
