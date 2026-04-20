/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    Time DT = Seconds.of(0.020);
    boolean DEBUG_MODE = true;
    CANBus CANIVORE = new CANBus("rio");

    public interface EnabledSubsystems {
        SmartBoolean FEEDER = new SmartBoolean("Enabled Subsystems/Feeder", true);
        SmartBoolean INTAKE = new SmartBoolean("Enabled Subsystems/Intake", true);
        // SmartBoolean LED = new SmartBoolean("Enabled Subsystems/LED", true);
        SmartBoolean SHOOTER = new SmartBoolean("Enabled Subsystems/Shooter", true);
        SmartBoolean VISION = new SmartBoolean("Enabled Subsystems/Vision", true);
        SmartBoolean SWERVE = new SmartBoolean("Enabled Subsystems/Swerve", true);
        SmartBoolean HANDOFF = new SmartBoolean("Enableed Subsystems/Handoff", true);
    }

    public interface Vision {
        //TODO: These numbers are temporary, may need testing
        public final Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        public final Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694.0);

        public final Translation2d INVALID_POSITION = new Translation2d(8.2705, 4.0345);
        public final Distance INVALID_POSITION_TOLERANCE = Meters.of(0.05);
        public final double MAX_ANGULAR_VELOCITY_RAD_SEC = 2 * Math.PI;

    }

    public interface Intake {
        public interface Pivot {
            // state angles
            Rotation2d INITIAL_ANGLE = Rotation2d.fromDegrees(0);
            Rotation2d IDLE_ANGLE = Rotation2d.fromDegrees(0);
            Rotation2d DOWN_ANGLE = Rotation2d.fromDegrees(102);

            // misc
            Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.5);
            Rotation2d PUSHDOWN_THRESHOLD = Rotation2d.fromDegrees(85);
            SmartNumber PUSHDOWN_CURRENT = new SmartNumber("Intake/Pivot/Pushdown Voltage Tuning Amps", -75.0);
            Current STALL_CURRENT = Amps.of(25); // amps
            Time STALL_DEBOUNCE_SEC = Seconds.of(0.0); // TODO: set this up?
            Voltage HOMING_DOWN_VOLTAGE = Volts.of(3);

            // sysid
            Velocity<VoltageUnit> RAMP_RATE = Volts.of(2).per(Second);
            Voltage STEP_VOLTAGE = Volts.of(6);

            // sim
            Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
            Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(102.0);
            double GEAR_RATIO = 60.0;
            MomentOfInertia J = KilogramSquareMeters.of(0.001);
        }

        public interface Roller {
            Current STALL_CURRENT = Amps.of(50);
            Time STALL_DEBOUNCE_SEC = Seconds.of(0.1);

            double GEAR_RATIO = 16.0 / 27.0;
            MomentOfInertia J = KilogramSquareMeters.of(0.001);

            double IDLE_DUTY_CYCLE = 0;
            double INTAKE_DUTY_CYCLE = 1;
            double OUTTAKE_DUTY_CYCLE = -1;
        }
    }

    public interface Feeder {
        double FEEDER_REVERSE_DUTY_CYCLE = -1;
        double FEEDER_FORWARD_DUTY_CYCLE = 1;

        double GEAR_RATIO = 1; // TODO: get from mec
        MomentOfInertia J = KilogramSquareMeters.of(0.001);
    }

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

    public interface Handoff {
        double IDLE_DUTY = 0;
        double FORWARD = 1;
        double BACKWARD = -1;
    }

    public interface Shooter {
        Time SHOOT_TIME_AUTO = Seconds.of(1.5);
        Velocity<VoltageUnit> RAMP_RATE = Volts.of(0.25).per(Second);
        Voltage STEP_VOLTAGE = Volts.of(900);

        Distance WHEEL_RADIUS = Inches.of(4);

        double SHOOT_DUTY = 1;
        double FERRY_DUTY = 1;
        double SOTM_DUTY = 0.8;
        double FOTM_DUTY = 0.8;
        double IDLE_DUTY = 0;
        // Sim
        MomentOfInertia J = KilogramSquareMeters.of(0.1);
        double GEAR_RATIO = 0.1;

        public interface RPMInterpolation{
            double[][] distanceRPMInterpolationValues = {
                {1.0, 1000.0},
                {2.0, 1500.0},
                {3.0, 2000.0},
                {4.0, 2500.0},
                {5.0, 3000.0}
            };
        }// These values are placeholders and should be replaced with actual data from testing

        public interface TOFInterpolation{
            double[][] distanceTOFInterpolationValues = {
                {1.0, 0.5},
                {2.0, 0.75},
                {3.0, 1.0},
                {4.0, 1.25},
                {5.0, 1.5}
            };
        }// These values are placeholders and should be replaced with actual data from testing

        public interface FerryRPMInterpolation {
            double[][] ferryDistanceRPMInterpolation = {
                {1.0, 1000.0},
                {2.0, 1500.0},
                {3.0, 2000.0},
                {4.0, 2500.0},
                {5.0, 3000.0}
            };
        }// These values are placeholders and should be replaced with actual data from testing

        public interface FerryTOFInterpolation {
            double [][] FerryTOFInterpolationInterpolation = {
                {1.0, 0.5},
                {2.0, 0.75},
                {3.0, 1.0},
                {4.0, 1.25},
                {5.0, 1.5}
            };
        }// These values are placeholders and should be replaced with actual data from testing
    }

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
                Distance X_TOLERANCE = Inches.of(2.0);; 
                Distance Y_TOLERANCE = Inches.of(2.0);
                Rotation2d THETA_TOLERANCE = Rotation2d.fromDegrees(1);

                Pose2d POSE_TOLERANCE = new Pose2d(
                    X_TOLERANCE.in(Meters), 
                    Y_TOLERANCE.in(Meters), 
                    THETA_TOLERANCE);

                LinearVelocity MAX_VELOCITY_WHEN_ALIGNED = MetersPerSecond.of(0.15);

                Time ALIGNMENT_DEBOUNCE = Seconds.of(0.15);
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