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

    public interface Intake {

        public interface Pivot {

            // state angles
            // TODO:Get new pivot angles
            Angle INITIAL_ANGLE = Degrees.of(-102);

            Angle STOW_ANGLE = Degrees.of(-102);

            Angle DEPLOY_ANGLE = Degrees.of(-22);

            Angle AGITATE_UP_ANGLE = Degrees.of(-62);

            Angle DIGEST_ANGLE = Degrees.of(-92);

            Angle AGITATE_DOWN_ANGLE = Degrees.of(-22);

            // misc
            Angle ANGLE_TOLERANCE = Degrees.of(0.5);

            Angle PUSHDOWN_THRESHOLD = Degrees.of(-30);

            DoubleSubscriber PUSHDOWN_CURRENT = DogLog.tunable("Intake/Pivot/Pushdown Current Tuning Amps", 13.0);

            // amps
            Current STALL_CURRENT = Amps.of(25);

            // TODO: set this up?
            Time STALL_DEBOUNCE_SEC = Seconds.of(0.0);

            Voltage HOMING_DOWN_VOLTAGE = Volts.of(3);

            // sysid
            Velocity<VoltageUnit> RAMP_RATE = Volts.of(1).per(Second);

            Voltage STEP_VOLTAGE = Volts.of(1);

            // sim
            Angle MIN_ANGLE = Degrees.of(0);

            Angle MAX_ANGLE = Degrees.of(-102.0);

            double GEAR_RATIO = 60.0;

            Distance PIVOT_ARM_LENGTH = Meters.of(0.1439822);

            // mass in kg
            MomentOfInertia MOI = KilogramSquareMeters.of(SingleJointedArmSim.estimateMOI(PIVOT_ARM_LENGTH.in(Meters), 1));
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

        Voltage REVERSE_VOLTAGE = Volts.of(-10.0); // TODO: get

        Voltage FORWARD_VOLTAGE = Volts.of(10.0);

        // TODO: get from mec
        double GEAR_RATIO = 34/14; // (34/14) : 1

        MomentOfInertia J = KilogramSquareMeters.of(0.001);
    }

    public interface LED {

        // TODO: Get actual length of led, along with length of individual sections
        int LED_LENGTH = 80;

        // Buffer Views {Starting Index, Ending Index}
        int[] SHOOTER_BUFFER = { 0, 19 };

        int[] FEEDER_BUFFER = { 20, 39 };

        int[] INTAKE_BUFFER = { 40, 59 };

        int[] HANDOFF_BUFFER = { 60, 79 };

        // shooter
        LEDPattern SHOOTING = LEDPattern.solid(Color.kOrange);

        LEDPattern FERRYING = LEDPattern.solid(Color.kPurple);

        LEDPattern MANUAL = LEDPattern.solid(Color.kPeru);

        // feeder
        LEDPattern FEEDER_FORWARD = LEDPattern.solid(Color.kBlue);

        LEDPattern FEEDER_REVERSE = LEDPattern.solid(Color.kRed);

        // intake
        LEDPattern INTAKING = LEDPattern.solid(Color.kYellow);

        LEDPattern OUTTAKING = LEDPattern.solid(Color.kGreen);

        LEDPattern HOMING_DOWN = LEDPattern.solid(Color.kGainsboro);

        LEDPattern AGITATING = LEDPattern.solid(Color.kCyan);

        // handoff
        LEDPattern HANDOFF_FORWARD = LEDPattern.solid(Color.kDarkOrange);

        // mmm papaya whip
        LEDPattern HANDOFF_REVERSE = LEDPattern.solid(Color.kPapayaWhip);

        // states
        LEDPattern DISABLED = LEDPattern.solid(Color.kGray);
    }

    public interface Handoff {
        Voltage IDLE_VOLTAGE = Volts.of(0.0);

        Voltage FORWARD_VOLTAGE = Volts.of(12.0);

        Voltage REVERSE_VOLTAGE = Volts.of(-10.0);

        double STALL_CURRENT = 67;

        // TODO: get and maybe convert to wpilib units
        double STALL_DEBOUNCE = 67;

        double J_KG_METERS_SQUARED = 1;

        double GEAR_RATIO = 1.0 / 3.0; // 1:3
    }

    public interface Shooter {
        DoubleSubscriber FIRST_SHOT_BONUS = DogLog.tunable("Shooter/First_shot_bonus_RPM", 250.0);
        Time FIRST_SHOT_DEBOUNCE = Seconds.of(6.7);

        Time SHOOT_TIME_AUTO = Seconds.of(1.5);

        Velocity<VoltageUnit> RAMP_RATE = Volts.of(1).per(Second);

        Voltage STEP_VOLTAGE = Volts.of(7);

        Distance WHEEL_RADIUS = Inches.of(4);

        // Sim
        MomentOfInertia J = KilogramSquareMeters.of(0.1);

        double GEAR_RATIO = 0.1;

        // TODO: get
        Distance FLYWHEEL_RADIUS = Inches.of(3);

        DoubleSubscriber BABY_RPM = DogLog.tunable("Shooter/Baby Shot RPM", 2700.0);

        // TODO: Test for manual shooting RPM
        DoubleSubscriber MANUAL_HUB_RPM = DogLog.tunable("Shooter/Manual Shot Tuning RPM", 1825.0);

        AngularVelocity MIN_SHOOTER_VELOCITY = RPM.of(870);

        DoubleSubscriber SHOOT_TUNING_RPM = DogLog.tunable("Shooter/Shoot Tuning RPM", 0.0);
        DoubleSubscriber FERRY_TUNING_RPM = DogLog.tunable("Shooter/Ferry Tuning RPM", 0.0);

        AngularVelocity SHOOTER_SPUN_UP_TOLERANCE = RPM.of(100);
        public interface RPMInterpolation {

            double[][] distanceRPMInterpolationValues = {
                {1.46, 1300},
                {2.07, 1575},
                {3.0, 1816.5},
                {3.13, 1850},
                {3.45, 1966.5},
                {4.13, 2100}
                //TODO: These numbers don't make sense
                // { 4.895367348608047, 3250.0 },
                // { 6.1322461808798705, 3487.0 } 
            };
        }

        // These values are placeholders and should be replaced with actual data from testing
        public interface TOFInterpolation {

            double[][] distanceTOFInterpolationValues = {
                { 1.0, 0.5 },
                { 2.0, 0.75 },
                { 3.0, 1.0 },
                { 4.0, 1.25 },
                { 5.0, 1.5 } };
        }

        // These values are placeholders and should be replaced with actual data from testing
        public interface FerryRPMInterpolation {

            double[][] ferryDistanceRPMInterpolation = {
                { 1.0, 1150.0 },
                { 2.0, 1400.0 },
                { 3.0, 1650.0 },
                { 4.0, 1900.0 },
                { 5.0, 2750.0 } };
        }

        // These values are placeholders and should be replaced with actual data from testing
        public interface FerryTOFInterpolation {

            double[][] FerryTOFInterpolationInterpolation = {
                { 1.0, 0.5 },
                { 2.0, 0.75 },
                { 3.0, 1.0 },
                { 4.0, 1.25 },
                { 5.0, 1.5 } };
        }
        // These values are placeholders and should be replaced with actual data from testing
    }

    public interface Swerve {

        double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;

        double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;

        public interface Constraints {

            double MAX_VELOCITY_M_PER_S = 1.0;

            // TODO: revert to 15.0
            double MAX_ACCEL_M_PER_S_SQUARED = 67.0;

            double MAX_ANGULAR_VEL_RAD_PER_S = Units.degreesToRadians(100.0);

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
