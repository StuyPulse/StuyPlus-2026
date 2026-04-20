/************************ PROJECT STUYPLUS *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.    */
/* Use of this source code is governed by an MIT-style license    */
/* that can be found in the repository LICENSE file.              */
/*******************************************************************/

package com.stuypulse.robot.util.simulation;

import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public interface SimulationConstants {
    /**
     * <h2>Record that holds CAD Offsets</h2>
     * <p>Sourced from CAD exports to hold a component's positional offsets from their position in CAD to their position in sim</p>
     * <pre>{@code
     * Offsets SHOOTER = new Offsets(-0.1016, 0.2032, 0.3255,
     *     Degrees.of(90), Degrees.of(0), Degrees.of(-90));
     *
     * Pose3d shooterPose = SHOOTER.withRotation(
     *     new Rotation3d(0, 0, turretSim.getAngle().getRadians())
     * );
     * }</pre>
     * @param x X translation from the robot origin (meters)
     * @param y Y translation from the robot origin (meters)
     * @param z Z translation from the robot origin (meters)
     * @param roll Rotation about the X axis
     * @param pitch Rotation about the Y axis
     * @param yaw Rotation about the Z axis
     */
    public static record Offsets(Distance x, Distance y, Distance z,
        Angle roll, Angle pitch, Angle yaw) {

        /**
         * <h2>Constructs an Offsets instance with no rotation</h2>
         * @param x X translation from the robot origin (meters)
         * @param y Y translation from the robot origin (meters)
         * @param z Z translation from the robot origin (meters)
         */
        public Offsets(double x, double y, double z) {
            this(Meters.of(x), Meters.of(y), Meters.of(z), Radians.of(0), Radians.of(0), Radians.of(0));
        }

        /**
         * <h2>Constructs an Offsets instance with no rotation, using your unit of choice</h2>
         * @param x X translation from the robot origin
         * @param y Y translation from the robot origin
         * @param z Z translation from the robot origin
         */
        public Offsets(Distance x, Distance y, Distance z) {
            this(x.in(Meters), y.in(Meters), z.in(Meters));
        }

        /**
         * <h2>Constructs an Offsets instance with no translation</h2>
         * @param roll Rotation about the X axis
         * @param pitch Rotation about the Y axis
         * @param yaw Rotation about the Z axis
         */
        public Offsets(Angle roll, Angle pitch, Angle yaw) {
            this(Meters.of(0), Meters.of(0), Meters.of(0), roll, pitch, yaw);
        }

        /**
         * <h2>Constructs an Offsets instance with no rotation</h2>
         * @param x X translation from the robot origin (meters)
         * @param y Y translation from the robot origin (meters)
         * @param z Z translation from the robot origin (meters)
         * @param roll Rotation about the X axis
         * @param pitch Rotation about the Y axis
         * @param yaw Rotation about the Z axis
         */
        public Offsets(double x, double y, double z, Angle roll, Angle pitch, Angle yaw) {
            this(Meters.of(x), Meters.of(y), Meters.of(z), roll, pitch, yaw);
        }

        /**
         * <h2>Translational component as a {@link Translation3d}</h2>
         * @return translation from the robot origin
         */
        public Translation3d toTranslation3d() {
            return new Translation3d(x.in(Meters), y.in(Meters), z.in(Meters));
        }

        /**
         * <h2>Rotational component as a {@link Rotation3d}</h2>
         * @return Rotation3d of the roll, pitch, and yaw components of the offset
         */
        public Rotation3d toRotation3d() {
            return new Rotation3d(roll, pitch, yaw);
        }

        /**
         * <h2>This offset as a {@link Pose3d}.</h2>
         * <p>Useful for components whose pose is fully static and requires no adjustments</p>
         * @return pose at this offset's position and orientation
         */
        public Pose3d toPose3d() {
            return new Pose3d(toTranslation3d(), toRotation3d());
        }

        /**
         * <h2>Applies this offset's translation and rotation onto an existing {@link Pose3d}</h2>
         * @param pose the base pose to offset
         * @return a new pose with this offset applied to both translation and rotation
         */
        public Pose3d applyToPose3d(Pose3d pose) {
            return new Pose3d(
                pose.getMeasureX().plus(x), pose.getMeasureY().plus(y), pose.getMeasureZ().plus(z),
                applyToRotation3d(pose.getRotation())
            );
        }

        /**
         * <h2>Applies this offset's translation and rotation onto an existing {@link Pose3d}</h2>
         * <p>Translation is applied, but it is <b>robot robot relative</b>
         * @param pose the base pose to offset
         * @return a new pose with this offset applied to both translation and rotation
         */
        public Pose3d applyToPose3dRobotRelative(Pose3d pose) {
            Translation3d rotatedOffset = toTranslation3d().rotateBy(pose.getRotation());
            return new Pose3d(
                pose.getTranslation().plus(rotatedOffset),
                applyToRotation3d(pose.getRotation())
            );
        }

        /**
         * <h2>Composes this offset's rotation onto an existing {@link Rotation3d}</h2>
         * @param rotation the base rotation to offset
         * @return a new rotation with this offset's rotation applied on top
         */
        public Rotation3d applyToRotation3d(Rotation3d rotation) {
            return new Rotation3d(
                rotation.getX() + roll.in(Radians),
                rotation.getY() + pitch.in(Radians),
                rotation.getZ() + yaw.in(Radians)
            );
        }

        /**
         * <h2>Pose at this offset's translation with an additional rotation</h2>
         * <p>Applies a rotation to this offsets rotation
         * <pre>{@code
         * SHOOTER_OFFSETS.withRotation(
         *     new Rotation3d(0, 0, turretSim.getAngle().getRadians())
         * );
         * }</pre>
         * @param rotation the additional rotation to add to this offset's rotation
         * @return a new pose at this offset's translation with the combined rotation
         */
        public Pose3d withRotation(Rotation3d rotation) {
            return new Pose3d(toTranslation3d(), toRotation3d().plus(rotation));
        }

        /**
         * <h2>Applies this offset's X/Y translation and yaw onto an existing {@link Pose2d}</h2>
         * <p>Roll and pitch are ignored because they have no 2D equivalent
         * @param pose the base 2D pose to offset
         * @return a new pose of this offset's X, Y, and yaw
         */
        public Pose2d applyToPose2d(Pose2d pose) {
            return new Pose2d(pose.getMeasureX().plus(x), pose.getMeasureY().plus(y),
                applyToRotation2d(pose.getRotation()));
        }

        /**
         * <h2>Applies this offset's X/Y translation and yaw onto an existing {@link Pose2d}</h2>
         * <p>Translation is applied, but it is <b>robot robot relative</b>
         * @param pose the base 2D pose to offset
         * @return a new pose with this offset applied
         */
        public Pose2d applyToPose2dRobotRelative(Pose2d pose) {
            Translation2d rotatedOffset = new Translation2d(x, y).rotateBy(pose.getRotation());
            return new Pose2d(
                pose.getTranslation().plus(rotatedOffset),
                applyToRotation2d(pose.getRotation())
            );
        }

        /**
         * <h2>Applies this offset's yaw onto an existing {@link Rotation2d}</h2>
         * <p>Roll and pitch are ignored because they have no 2D equivalent
         * @param rotation the base 2D rotation to offset
         * @return a new rotation with this offset's yaw applied on top
         */
        public Rotation2d applyToRotation2d(Rotation2d rotation) {
            return new Rotation2d(
                rotation.getRadians() + yaw.in(Radians)
            );
        }
    }

    public interface Intake {
        double INTAKE_WIDTH = 0.5;
        double INTAKE_LENGTH = 0.15;

        double PIVOT_ARM_LENGTH = 0.1439822;
        double PIVOT_END_X = 0;

        public Offsets PIVOT_OFFSETS = new Offsets(
            0.2393388152,
            0.0,
            0.19685, // CAD zero angle offset degrees
            Degrees.of(-40),
            Degrees.of(0),
            Degrees.of(90)
        );

        public Offsets ROLLER_OFFSETS = new Offsets(0.022, 0, 0.2152848, Degrees.of(90), Degrees.of(0), Degrees.of(90));
        public Offsets OUTTAKE_OFFSETS = new Offsets(0.4, 0, 0);
    }

    public interface Hopper {
        int FUEL_CAPACITY = 54;

        public Offsets OFFSETS = new Offsets(-0.06, 0, 0.25, Degrees.of(90), Degrees.of(0), Degrees.of(90));

        int FUEL_LAYERS = 4;
        Pose3d VISIBLE_POSE = new Pose3d(0, 0, 0, new Rotation3d(1.55, 0, 1.5));
        Pose3d HIDDEN_POSE = new Pose3d(1000, 1000, 1000, new Rotation3d());
    }

    public interface Shooter {
        double BPS = 8;

        double COMPRESSION_METRES = Units.inchesToMeters(1.379342);

        public static double angularVelocityToMps(AngularVelocity angularVelocity) {
            return ((Settings.Shooter.WHEEL_RADIUS.in(Meters) * 2 - COMPRESSION_METRES) * (angularVelocity.in(RPM)) * Math.PI) / 60.0;
        }

        public Offsets OFFSETS = new Offsets(Units.inchesToMeters(-7.836), 0, 0.7);
    }

    public interface Drivetrain {
        PIDConstants XY = new PIDConstants(2.2, 0, 0.0); // alignment
        PIDConstants THETA = new PIDConstants(3, 0, 0.0);

        Distance LENGTH = Inches.of(27);
        Distance WIDTH = Inches.of(25.5);
        double WHEEL_COF = 1.2; // TODO: Get actual

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

        int PIGEON_ID = 0; // TODO: get actual

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

    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
			new Pose2d(-6, 0, new Rotation2d()),
			new Pose2d(-5, 0, new Rotation2d()),
			new Pose2d(-4, 0, new Rotation2d()),
			new Pose2d(-3, 0, new Rotation2d()),
			new Pose2d(-2, 0, new Rotation2d())
	};

	public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
			new Pose2d(12.5, 0.5, Rotation2d.fromDegrees(90)), // depot side trench facing hub
			new Pose2d(12.5, 7.777, Rotation2d.fromDegrees(270)),
			new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
			new Pose2d(1.6, 6, new Rotation2d()),
			new Pose2d(1.6, 4, new Rotation2d())
	};

    public static final Boolean SPAWN_GAMEPIECES_SPARSELY = true; // whether to spawn a decreased set of gamepieces to conserve processing power
}