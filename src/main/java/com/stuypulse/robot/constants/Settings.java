/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import com.ctre.phoenix6.CANBus;
import dev.doglog.DogLog;

import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.robot.Robot;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use DogLog's tunables in order to have tunable
 * values that we can edit on whatever dashboard we
 * are using.
 */
public interface Settings {

    Time DT = Seconds.of(0.020);

    boolean DEBUG_MODE = true;

    CANBus CANBUS = new CANBus("rio");

    Mode SIM_MODE = Mode.SIM;

    Mode CURRENT_MODE = Robot.isReal() ? Mode.REAL : SIM_MODE;

    enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public interface EnabledSubsystems {

        BooleanSubscriber FEEDER = DogLog.tunable("Enabled Subsystems/Feeder", true);

        BooleanSubscriber INTAKE = DogLog.tunable("Enabled Subsystems/Intake", true);

        // BooleanSubscriber INTAKE_ROLLERS = DogLog.tunable("Enabled Subsystems/Intake/Rollers", true);

        // BooleanSubscriber INTAKE_PIVOT = DogLog.tunable("Enabled Subsystems/Intake/Pivot", true);

        BooleanSubscriber LED = DogLog.tunable("Enabled Subsystems/LED", false);

        BooleanSubscriber HANDOFF = DogLog.tunable("Enabled Subsystems/Handoff", true);

        BooleanSubscriber SHOOTER = DogLog.tunable("Enabled Subsystems/Shooter", true);

        BooleanSubscriber VISION = DogLog.tunable("Enabled Subsystems/Vision", true);

        BooleanSubscriber SWERVE = DogLog.tunable("Enabled Subsystems/Swerve", true);
    }

    public interface Vision {

        // TODO: These numbers are temporary, may need testing
        public final Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);

        public final Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694.0);

        public final Pose2d INVALID_POSITION = Pose2d.kZero;

        public final double MAX_ANGULAR_VELOCITY_RAD_SEC = 2 * Math.PI;
    }

    public interface Swerve {

        double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;

        double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;

        public interface Constraints {

            double MAX_VELOCITY_M_PER_S = 4.3;

            // TODO: revert to 15.0
            double MAX_ACCEL_M_PER_S_SQUARED = 20.0;

            double MAX_ANGULAR_VEL_RAD_PER_S = Units.degreesToRadians(400.0);

            // TODO: revert to 900
            double MAX_ANGULAR_ACCEL_RAD_PER_S = Units.degreesToRadians(300.0);

            PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(MAX_VELOCITY_M_PER_S, MAX_ACCEL_M_PER_S_SQUARED, MAX_ANGULAR_VEL_RAD_PER_S, MAX_ANGULAR_ACCEL_RAD_PER_S);
        }

        public interface Alignment {

            public interface Constraints {

                double DEFAULT_MAX_VELOCITY = 4.3;

                double DEFAULT_MAX_ACCELERATION = 15.0;

                double DEFAULT_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(400.0);

                double DEFAULT_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(900.0);
            }

            public interface Tolerances {

                Distance X_TOLERANCE = Inches.of(2.0);

                Distance Y_TOLERANCE = Inches.of(2.0);

                Rotation2d THETA_TOLERANCE = Rotation2d.fromDegrees(8);

                Pose2d POSE_TOLERANCE = new Pose2d(X_TOLERANCE.in(Meters), Y_TOLERANCE.in(Meters), THETA_TOLERANCE);

                LinearVelocity MAX_VELOCITY_WHEN_ALIGNED = MetersPerSecond.of(0.15);

                Time ALIGNMENT_DEBOUNCE = Seconds.of(0.15);
            }

            public interface Targets {

                // TODO: Get actual angle
                Rotation2d HUB_LEFT_CORNER = Rotation2d.fromDegrees(45);

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

            double DEADBAND = 0.07;

            double RC = 0.05;

            double POWER = 2.0;
        }
    }
}
