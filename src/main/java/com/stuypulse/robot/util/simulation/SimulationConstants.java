/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.simulation;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.TunerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

/**
 * <h2>SimulationConstants</h2>
 * <p>A collection of constants used within our MapleSim code and other related sim classes.
 * <p>This includes things like component offsets, physical constants, and any other values 
 * that are relevant to the simulation</p>
 */
public interface SimulationConstants {
    /**
     * Contains simulation constants related to the intake subsystem
     */
    public interface Intake {

        double INTAKE_WIDTH = 0.5;

        double INTAKE_LENGTH = 0.15;

        double PIVOT_END_X = 0;

        public Offsets PIVOT_OFFSETS = new Offsets(
                0.2393388152,
                0.0, // CAD zero angle offset degrees
                0.19685,
                Degrees.of(-40),
                Degrees.of(0),
                Degrees.of(90));

        public Offsets ROLLER_OFFSETS = new Offsets(0.022, 0, 0.2152848, Degrees.of(90), Degrees.of(0), Degrees.of(90));

        public Offsets OUTTAKE_OFFSETS = new Offsets(0.4, 0, 0);
    }

    /**
     * Contains simulation constants related to our physical hopper.
     * <p>The hopper is not necessarily its own subsystem, but it has some important properties.
     */
    public interface Hopper {

        int FUEL_CAPACITY = 54;

        public Offsets OFFSETS = new Offsets(-0.06, 0, 0.25, Degrees.of(90), Degrees.of(0), Degrees.of(90));

        int FUEL_LAYERS = 4;

        Pose3d VISIBLE_POSE = new Pose3d(0, 0, 0, new Rotation3d(1.55, 0, 1.5));

        Pose3d HIDDEN_POSE = new Pose3d(1000, 1000, 1000, new Rotation3d());
    }

    /**
     * Contains simulation constants related to the shooter subsystem
     */
    public interface Shooter {

        double BPS = 8;

        double COMPRESSION_METRES = Units.inchesToMeters(1.379342);

        public static double angularVelocityToMps(AngularVelocity angularVelocity) {
            return ((Settings.Shooter.WHEEL_RADIUS.in(Meters) * 2 - COMPRESSION_METRES)
                    * (angularVelocity.in(RPM))
                    * Math.PI)
                    / 60.0;
        }

        public Offsets OFFSETS = new Offsets(Units.inchesToMeters(-7.836), 0, 0.7);
    }

    /**
     * Contains simulation constants related to our drivetrain and its components
     */
    public interface Drivetrain {

        // alignment
        PIDConstants XY = new PIDConstants(2.2, 0, 0.0);

        PIDConstants THETA = new PIDConstants(3, 0, 0.0);

        Distance LENGTH = Inches.of(27);

        Distance WIDTH = Inches.of(25.5);

        // TODO: Get actual
        double WHEEL_COF = 1.2;

        Time SIMULATION_STEP_TIME = Seconds.of(0.005);

        Mass ROBOT_WEIGHT = Pounds.of(65);

        Mass RED_BUMPER_WEIGHT = Pounds.of(16.8);

        Mass BLUE_BUMPER_WEIGHT = Pounds.of(15.4);

        Supplier<Mass> TOTAL_WEIGHT = () -> {
            if (Robot.isBlue()) {
                return ROBOT_WEIGHT.plus(BLUE_BUMPER_WEIGHT);
            } else {
                return ROBOT_WEIGHT.plus(RED_BUMPER_WEIGHT);
            }
        };

        @SuppressWarnings("unchecked")
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] MODULE_CONSTANTS = new SwerveModuleConstants[] {
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight
        };

        public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
                new Translation2d(MODULE_CONSTANTS[0].LocationX, MODULE_CONSTANTS[0].LocationY),
                new Translation2d(MODULE_CONSTANTS[1].LocationX, MODULE_CONSTANTS[1].LocationY),
                new Translation2d(MODULE_CONSTANTS[2].LocationX, MODULE_CONSTANTS[2].LocationY),
                new Translation2d(MODULE_CONSTANTS[3].LocationX, MODULE_CONSTANTS[3].LocationY)
        };
    }

    /**
     * Starting positions for the robots in the simulation.
     */
	public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
			new Pose2d(12.5, 0.5, Rotation2d.fromDegrees(90)), // depot side trench facing hub
			new Pose2d(12.5, 7.777, Rotation2d.fromDegrees(270)),
			new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
			new Pose2d(1.6, 6, new Rotation2d()),
			new Pose2d(1.6, 4, new Rotation2d())
	};

    /**
     * Boolean for whether {@link org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt} efficiency mode is enabled. When true, less game pieces will be spawned.
     * If false, the normal amount will spawn.
     */
    public static final Boolean SPAWN_GAMEPIECES_SPARSELY = true; // whether to spawn a decreased set of gamepieces to conserve processing power
}